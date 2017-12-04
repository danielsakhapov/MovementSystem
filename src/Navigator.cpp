#include "Navigator.h"

Navigator::Navigator(int argc, char* argv[], ros::NodeHandle& nodeHandle)
{
	_hasNewMap = false;
	_hasNewGoal = false;
	_hasNewOdometry = false;

	_engine.angle = 0;
	_engine.speed = 0;
	_engine.direction = 0;

	_mapSubcriber = nodeHandle.subscribe("rtabmap/grid_map", 100, &Navigator::_mapCallback, this);
	_odometrySubcriber = nodeHandle.subscribe("rtabmap/odom", 1000, &Navigator::_odometryCallback, this);
	_goalSubcriber = nodeHandle.subscribe("move_base_simple/goal", 1000, &Navigator::_goalCallback, this);
	_pathPublisher = nodeHandle.advertise<nav_msgs::Path>("Navigator/Path", 1000);

	std::thread findPathThread(&Navigator::_findPath, this);
	findPathThread.detach();

	//std::thread checkPathThread(&Navigator::_checkPath, this);
	//checkPathThread.detach();

	std::thread followPathThread(&Navigator::_followPath, this);
	followPathThread.detach();
}



Navigator::~Navigator()
{}



Engine& Navigator::getEngine()
{
	boost::shared_lock<boost::shared_mutex> sLock(_engineMutex);
	return _engine;
}



void Navigator::_findPath()
{
	while (ros::ok())
	{
		std::unique_lock<boost::shared_mutex> uDLock(_dataMutex);
		_goalCondVar.wait( uDLock, [&, this]{ return _hasNewGoal; });
		_hasNewGoal = false;
		_hasNewPath = true;

		std::unique_lock<boost::shared_mutex> uELock(_engineMutex);
		_engine.angle = 0;
		_engine.speed = 0;
		_engine.direction = 0;
		uELock.unlock();

		u32 mapWidth =_pMap->info.width;
		u32 mapHeight = _pMap->info.height;
		double mapResolution = _pMap->info.resolution;

		double mapAbsolutePositionX = _pMap->info.origin.position.x;
		double mapAbsolutePositionY = _pMap->info.origin.position.y;

		double robotAbsolutePositionX = _pOdometry->pose.pose.position.x - _pMap->info.origin.position.x;
		double robotAbsolutePositionY = _pOdometry->pose.pose.position.y - _pMap->info.origin.position.y;

		double goalAbsolutePositionX = _pGoal->pose.position.x - _pMap->info.origin.position.x;
		double goalAbsolutePositionY = _pGoal->pose.position.y - _pMap->info.origin.position.y;

		uPosition robotCellPosition;
		robotCellPosition.x = robotAbsolutePositionY / mapResolution; // changed axes
		robotCellPosition.y = robotAbsolutePositionX / mapResolution; // changed axes
		robotCellPosition.theta = _getAngleFromQuaternion(_pOdometry->pose.pose.orientation);

		uPosition goalCellPosition;
		goalCellPosition.x = goalAbsolutePositionY / mapResolution; // changed axes
		goalCellPosition.y = goalAbsolutePositionX / mapResolution; // changed axes
		goalCellPosition.theta = _getAngleFromQuaternion(_pGoal->pose.orientation);

		vec_vec_bool map(mapHeight, vec_bool(mapWidth, false));
		for(u32 i = 0, k = 0; i < mapHeight; ++i)
			for (u32 j = 0; j < mapWidth; ++j)
				map[i][j] = (_pMap->data[k++] == 0);

		uDLock.unlock();

		vec_uPosition path = _buildPath(map, robotCellPosition, goalCellPosition);

		if (!path.empty()) {
			std::lock_guard<boost::shared_mutex> uPLock(_pathMutex);
			_path.clear();
			nav_msgs::Path rosPath;
			rosPath.header.frame_id = "odom";
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = "odom";
			for (auto it: path) {
				pose.pose.position.x = it.y * mapResolution + mapAbsolutePositionX;
				pose.pose.position.y = it.x * mapResolution + mapAbsolutePositionY;
				_path.push_back({it.y * mapResolution, it.x * mapResolution, it.theta});
				rosPath.poses.push_back(pose);
			}
			_pathPublisher.publish(rosPath);
			_pathCondVar.notify_one();
		}
	}
}



void Navigator::_checkPath()
{
	while (ros::ok())
	{
		std::unique_lock<boost::shared_mutex> uDLock(_dataMutex);
		_mapCondVar.wait( uDLock, [&, this]{ return _hasNewMap; });
		_hasNewMap = false;
		boost::shared_lock<boost::shared_mutex> sPLock(_pathMutex);

		bool isPathGood = true;
		for (auto it : _path) {
			u32 k = (it.y / _pMap->info.resolution) * _pMap->info.width + (it.x / _pMap->info.resolution);
			isPathGood &= (_pMap->data[k] == 0);
		}

		sPLock.unlock();
		uDLock.unlock();

		if (!isPathGood) {
			_hasNewGoal = true;
			_goalCondVar.notify_one();
		}
	}
}



void Navigator::_followPath()
{
	while (ros::ok())
	{
		std::unique_lock<boost::shared_mutex> uPLock(_pathMutex);
		_pathCondVar.wait( uPLock, [&, this]{ return !_path.empty(); });
		_hasNewPath = false;
		dPosition localGoal = _path.front();
		_path.pop_front();
		uPLock.unlock();

		dPosition robotPosition;
		auto sqr = [](auto a) { return a * a; };
		std::unique_lock<boost::shared_mutex> uDLock(_dataMutex, std::defer_lock);
		std::unique_lock<boost::shared_mutex> uELock(_engineMutex, std::defer_lock);
		auto distance = [&]() { return (sqrt(sqr(robotPosition.x - localGoal.x) + sqr(robotPosition.y - localGoal.y))); };
		while (!_hasNewPath) {
			uDLock.lock();
			_odometryCondVar.wait( uDLock, [&, this]{ return _hasNewOdometry; });

			robotPosition.x = _pOdometry->pose.pose.position.x - _pMap->info.origin.position.x;
			robotPosition.y = _pOdometry->pose.pose.position.y - _pMap->info.origin.position.y;
			robotPosition.theta = _getAngleFromQuaternion(_pOdometry->pose.pose.orientation);
			uDLock.unlock();

			if (distance() < 1)
				break;

			uELock.lock();
			_engine.angle = -(localGoal.theta - robotPosition.theta);
			_engine.speed = 1;
			_engine.direction = 1;
			uELock.unlock();
		}
		uELock.lock();
		_engine.angle = 0;
		_engine.speed = 0;
		_engine.direction = 0;
		uELock.unlock();
	}
}



vec_uPosition Navigator::_buildPath(vec_vec_bool& map, uPosition& robotPosition, uPosition& goalPosition)
{
	vec_uPosition path;
	path.push_back({robotPosition.x, robotPosition.y, 0});
	path.push_back({goalPosition.x, goalPosition.y, 0});
	return path;
}



void Navigator::_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& pMap)
{
	boost::shared_lock<boost::shared_mutex> sDLock(_dataMutex);
	_hasNewMap = true;
	_pMap = pMap;
	_mapCondVar.notify_one();
}



void Navigator::_goalCallback(const geometry_msgs::PoseStamped::ConstPtr& pGoal)
{
	boost::shared_lock<boost::shared_mutex> sDLock(_dataMutex);
	_hasNewGoal = true;
	_pGoal = pGoal;
	_goalCondVar.notify_one();
}



void Navigator::_odometryCallback(const nav_msgs::Odometry::ConstPtr& pOdometry)
{
	boost::shared_lock<boost::shared_mutex> sDLock(_dataMutex);
	_hasNewOdometry = true;
	_pOdometry = pOdometry;
	_odometryCondVar.notify_one();
}



double Navigator::_getAngleFromQuaternion(const geometry_msgs::Quaternion& quaternion)
{
	double w = quaternion.w;
	double x = quaternion.x;
	double y = quaternion.y;
	double z = quaternion.z;

	double t3 = 2.0 * (w * z + x * y);
	double t4 = 1.0 - 2.0 * (y * y + z * z);
	double yaw = std::atan2(t3, t4);
	yaw = (yaw * 180) / M_PI;

	return yaw;
}
