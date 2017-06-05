#include "MovementSystem.hpp"

MovementSystem::MovementSystem(int argc, char* argv[], ros::NodeHandle& nodeHandle)
{
	_mapSubcriber = nodeHandle.subscribe("rtabmap/grid_map", 100, &MovementSystem::_mapCallback, this);
	_odometrySubcriber = nodeHandle.subscribe("rtabmap/odom", 1000, &MovementSystem::_odometryCallback, this);
	_goalSubcriber = nodeHandle.subscribe("move_base_simple/goal", 1000, &MovementSystem::_goalCallback, this);
	_pathPublisher = nodeHandle.advertise<nav_msgs::Path>("autonomusCar/Path", 1000);

	_engine.angle = 0;
	_engine.speed = 0;

	std::thread t(&MovementSystem::_moveByPath, this);
	t.detach();
}



MovementSystem::~MovementSystem()
{}



MovementSystem::Engine& MovementSystem::getEngine()
{
	return _engine;
}



void MovementSystem::_findPath()
{
	std::lock(_dataMutex, _pathMutex);
	std::unique_lock<boost::shared_mutex> uDLock(_dataMutex, std::adopt_lock);
	std::lock_guard<std::mutex> uPLock(_pathMutex, std::adopt_lock);

	uint32_t mapWidth =_pMap->info.width;
	uint32_t mapHeight = _pMap->info.height;
	double mapResolution = _pMap->info.resolution;

	double robotAbsolutePositionX = _pOdometry->pose.pose.position.x - _pMap->info.origin.position.x;
	double robotAbsolutePositionY = _pOdometry->pose.pose.position.y - _pMap->info.origin.position.y;

	double goalAbsolutePositionX = _pGoal->pose.position.x - _pMap->info.origin.position.x;
	double goalAbsolutePositionY = _pGoal->pose.position.y - _pMap->info.origin.position.y;

	//uint32_t robotOrientation = _pOdometry->pose.pose.orientation.z;
	uint32_t robotCellPositionX = robotAbsolutePositionY / mapResolution; // changed axes
	uint32_t robotCellPositionY = robotAbsolutePositionX / mapResolution; // changed axes

	//uint32_t goalOrientation = _pGoal->pose.orientation.z;
	uint32_t goalCellPositionX = goalAbsolutePositionY / mapResolution; // changed axes
	uint32_t goalCellPositionY = goalAbsolutePositionX / mapResolution; // changed axes

	std::cout << std::endl << "goal: " << goalAbsolutePositionX << ' ' << goalAbsolutePositionY << std::endl;
	std::cout << "robot " << robotAbsolutePositionX << ' ' << robotAbsolutePositionY << std::endl;

	std::vector<std::vector<MapCellInfo>> map(mapHeight, std::vector<MapCellInfo>(mapWidth, { false, 0, { 0, 0 } }));
	for (uint32_t i = 0, k = 0; i < mapHeight; ++i) {
		for (uint32_t j = 0; j < mapWidth; ++j) {
			map[i][j].isFree = (_pMap->data[k++] == 0);
//			if (i == robotCellPositionX && j == robotCellPositionY)
//				std::cout << 'R';
//			else
//				if (i == goalCellPositionX && j == goalCellPositionY)
//					std::cout << 'G';
//				else
//					std::cout << map[i][j].isFree;
		}
//		std::cout << std::endl;
	}

	uDLock.unlock();

	auto sqr = [](auto a) { return a * a; };
	auto isChecked = [&](uint32_t x, uint32_t y) { return map[x][y].parent.x != 0 || map[x][y].parent.y != 0; };
	auto h = [&](uint32_t x, uint32_t y) { return static_cast<uint32_t>(sqrt(sqr(x - goalCellPositionX) + sqr(y - goalCellPositionY))); };
	auto f = [&](uint32_t x, uint32_t y) { return map[x][y].wayLength + h(x, y); };
	auto hasEnoughSpace = [&](uint32_t x, uint32_t y) { uint32_t nFreeCells = 0;
							    for (int i = -10; i <= 10; ++i)
								for (int j = -10; j <= 10; ++j)
									nFreeCells += map[x + i][y + j].isFree;
							    return nFreeCells > 80; };

	const std::pair<uint32_t, uint32_t> startPose(robotCellPositionX, robotCellPositionY);
	const std::pair<uint32_t, uint32_t> finishPose(goalCellPositionX, goalCellPositionY);
	std::queue<std::pair<uint32_t, uint32_t>> poseQueue;
	poseQueue.push(startPose);
	map[robotCellPositionX][robotCellPositionY].parent.x = robotCellPositionX;
	map[robotCellPositionX][robotCellPositionY].parent.y = robotCellPositionY;

	while (!poseQueue.empty()) {
		uint32_t x = poseQueue.front().first;
		uint32_t y = poseQueue.front().second;
		poseQueue.pop();

		//std::cout << x << ' ' << y << std::endl;

		uint32_t xMin, yMin, fMin = UINT32_MAX;
		for (int i = -1; i <= 1; ++i)
			for (int j = -1; j <= 1; ++j)
				if (!isChecked(x + i, y + j) && map[x + i][y + j].isFree /*
				    hasEnoughSpace(x + i, y + j)*/)
					if (f(x + i, y + j) < fMin) {
						xMin = x + i;
						yMin = y + j;
						fMin = f(xMin, yMin);
					}

		if (fMin != UINT32_MAX) {
			poseQueue.push({ xMin, yMin });
			map[xMin][yMin].wayLength = map[x][y].wayLength + 1;
			map[xMin][yMin].parent.x = x;
			map[xMin][yMin].parent.y = y;

		}
	}

	_path.clear();

	std::vector<std::pair<double, double>> path;

	std::function<void(std::pair<uint32_t, uint32_t>)> getPath;
	getPath = [&](std::pair<uint32_t, uint32_t> currentPose) { if (currentPose == startPose) path.push_back(currentPose);
						else { getPath({map[currentPose.first][currentPose.second].parent.x,
								map[currentPose.first][currentPose.second].parent.y});
							if (currentPose != finishPose) path.push_back(currentPose);
							} };

	if (map[goalCellPositionX][goalCellPositionY].parent.x != 0 || map[goalCellPositionX][goalCellPositionY].parent.y != 0) {
		getPath(finishPose);

		for (uint32_t i = 1; i < path.size(); i += 20) {
			path[i].first *= mapResolution;
			path[i].second *= mapResolution;
			std::swap(path[i].first, path[i].second);
			if (i % 20 == 0)
				_path.push_back(path[i]);
		}
		_path.push_back({ goalAbsolutePositionX, goalAbsolutePositionY });
		_path.push_back({ goalAbsolutePositionX, goalAbsolutePositionY });

		for (auto it : _path)
			std::cout << it.first << ' ' << it.second << " -> ";
		std::cout << std::endl;

		_pathCondVar.notify_one();
	}
}



void MovementSystem::_checkPath()
{
	std::lock(_dataMutex, _pathMutex);
	std::unique_lock<boost::shared_mutex> uDLock(_dataMutex, std::adopt_lock);
	std::unique_lock<std::mutex> uPLock(_pathMutex, std::adopt_lock);

	bool isPathGood = true;

	for (auto it : _path) {
		uint32_t k = (it.second / _pMap->info.resolution) * _pMap->info.width + (it.first / _pMap->info.resolution);
		isPathGood &= (_pMap->data[k] == 0);
	}

	uPLock.unlock();
	uDLock.unlock();

	if (!isPathGood) {		
		_findPath();
	}
	else
		std::cout << "OK" << std::endl;
}



void MovementSystem::_moveByPath()
{
	while (ros::ok()) {
		std::unique_lock<std::mutex> uPLock(_pathMutex);
		_pathCondVar.wait( uPLock, [&, this]{ return !_path.empty(); });
		auto currentGoal = _path.front();
		_path.pop_front();
		uPLock.unlock();

		std::unique_lock<boost::shared_mutex> uDLock(_dataMutex);

		double robotA = _toEulerianAngle();
		double robotX = _pOdometry->pose.pose.position.x - _pMap->info.origin.position.x;
		double robotY = _pOdometry->pose.pose.position.y - _pMap->info.origin.position.y;

		uDLock.unlock();

		auto sqr = [](auto a) { return a * a; };
		auto distance = [&]() { return (sqrt(sqr(robotX - currentGoal.first) + sqr(robotY - currentGoal.second))); };

		while ((distance() > 0.5) && !_path.empty()) {
			double k = (currentGoal.second - robotY) / (currentGoal.first - robotX);
			int angle = (std::atan(k) * 180) / M_PI;

			_engine.angle = -(angle - robotA);
			if (_engine.angle < -90)
				_engine.angle = -90;
			if (_engine.angle > 90)
				_engine.angle = 90;
			_engine.speed = 1;

//			uDLock.lock();
			robotA = _toEulerianAngle();
			robotX = _pOdometry->pose.pose.position.x - _pMap->info.origin.position.x;
			robotY = _pOdometry->pose.pose.position.y - _pMap->info.origin.position.y;
//			uDLock.unlock();
		}
		_engine.angle = 0;
		_engine.speed = 0;
	}
}



void MovementSystem::_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& pMap)
{
	boost::shared_lock<boost::shared_mutex> sl(_dataMutex);
	_pMap = pMap;
	sl.unlock();
	_checkPath();
}



void MovementSystem::_goalCallback(const geometry_msgs::PoseStamped::ConstPtr& pGoal)
{
	boost::shared_lock<boost::shared_mutex> sl(_dataMutex);
	_pGoal = pGoal;
	sl.unlock();
	_findPath();
}



void MovementSystem::_odometryCallback(const nav_msgs::Odometry::ConstPtr& pOdometry)
{
	boost::shared_lock<boost::shared_mutex> sl(_dataMutex);
	_pOdometry = pOdometry;
}



double MovementSystem::_toEulerianAngle()
{
	double w = _pOdometry->pose.pose.orientation.w;
	double x = _pOdometry->pose.pose.orientation.x;
	double y = _pOdometry->pose.pose.orientation.y;
	double z = _pOdometry->pose.pose.orientation.z;

	// roll (x-axis rotation)
	double t0 = 2.0 * (w * x + y * z);
	double t1 = 1.0 - 2.0 * (x * x + y * y);
	double roll = std::atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (w * y - z * x);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	double pitch = std::asin(t2);

	// yaw (z-axis rotation)
	double t3 = 2.0 * (w * z + x * y);
	double t4 = 1.0 - 2.0 * (y * y + z * z);
	double yaw = std::atan2(t3, t4);

	auto RadToDegrees = [](double rad) { return (rad * 180) / M_PI; };

	roll = RadToDegrees(roll);
	pitch = RadToDegrees(pitch);
	yaw = RadToDegrees(yaw);

	return yaw;
}
