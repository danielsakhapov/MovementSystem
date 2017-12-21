#include "Navigator.h"

Navigator::Navigator(int argc, char* argv[], ros::NodeHandle& nodeHandle)
{
	hasNewMap_ = false;
	hasNewGoal_ = false;
	hasNewOdometry_ = false;

	engine_.angle = 0;
	engine_.speed = 0;
	engine_.direction = 0;

	mapSubcriber_ = nodeHandle.subscribe("rtabmap/grid_map", 100, &Navigator::mapCallback_, this);
	odometrySubcriber_ = nodeHandle.subscribe("rtabmap/odom", 1000, &Navigator::odometryCallback_, this);
	goalSubcriber_ = nodeHandle.subscribe("move_base_simple/goal", 1000, &Navigator::goalCallback_, this);
	pathPublisher_ = nodeHandle.advertise<nav_msgs::Path>("Navigator/Path", 1000);

	std::thread findPathThread(&Navigator::findPath_, this);
	findPathThread.detach();

	//std::thread checkPathThread(&Navigator::_checkPath, this);
	//checkPathThread.detach();

	std::thread followPathThread(&Navigator::followPath_, this);
	followPathThread.detach();
}



Navigator::~Navigator()
{}



Engine& Navigator::getEngine()
{
	boost::shared_lock<boost::shared_mutex> sLock(engineMutex_);
	return engine_;
}



void Navigator::findPath_()
{
	while (ros::ok())
	{
		std::unique_lock<boost::shared_mutex> uDLock(dataMutex_);
		goalCondVar_.wait( uDLock, [&, this]{ return hasNewGoal_; });
		hasNewGoal_ = false;
		hasNewPath_ = true;

		std::unique_lock<boost::shared_mutex> uELock(engineMutex_);
		engine_.angle = 0;
		engine_.speed = 0;
		engine_.direction = 0;
		uELock.unlock();

		u32 mapWidth = pMap_->info.width;
		u32 mapHeight = pMap_->info.height;
		double mapResolution = pMap_->info.resolution;

		double mapAbsolutePositionX = pMap_->info.origin.position.x;
		double mapAbsolutePositionY = pMap_->info.origin.position.y;

		double robotAbsolutePositionX = pOdometry_->pose.pose.position.x - pMap_->info.origin.position.x;
		double robotAbsolutePositionY = pOdometry_->pose.pose.position.y - pMap_->info.origin.position.y;

		double goalAbsolutePositionX = pGoal_->pose.position.x - pMap_->info.origin.position.x;
		double goalAbsolutePositionY = pGoal_->pose.position.y - pMap_->info.origin.position.y;

		uPosition robotCellPosition;
		robotCellPosition.x = robotAbsolutePositionY / mapResolution; // changed axes
		robotCellPosition.y = robotAbsolutePositionX / mapResolution; // changed axes
		robotCellPosition.theta = getAngleFromQuaternion_(pOdometry_->pose.pose.orientation);

		uPosition goalCellPosition;
		goalCellPosition.x = goalAbsolutePositionY / mapResolution; // changed axes
		goalCellPosition.y = goalAbsolutePositionX / mapResolution; // changed axes
		goalCellPosition.theta = getAngleFromQuaternion_(pGoal_->pose.orientation);

		vec_vec_bool map(mapHeight, vec_bool(mapWidth, false));
		for(u32 i = 0, k = 0; i < mapHeight; ++i)
			for (u32 j = 0; j < mapWidth; ++j)
				map[i][j] = (pMap_->data[k++] == 0);

		uDLock.unlock();

		vec_uPosition path = buildPath_(map, robotCellPosition, goalCellPosition);

		if (!path.empty()) {
			std::lock_guard<boost::shared_mutex> uPLock(pathMutex_);
			path_.clear();
			nav_msgs::Path rosPath;
			rosPath.header.frame_id = "odom";
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = "odom";
			for (auto it: path) {
				pose.pose.position.x = it.y * mapResolution + mapAbsolutePositionX;
				pose.pose.position.y = it.x * mapResolution + mapAbsolutePositionY;
				path_.push_back({it.y * mapResolution, it.x * mapResolution, it.theta});
				rosPath.poses.push_back(pose);
			}
			pathPublisher_.publish(rosPath);
			pathCondVar_.notify_one();
		}
	}
}



void Navigator::checkPath_()
{
	while (ros::ok())
	{
		std::unique_lock<boost::shared_mutex> uDLock(dataMutex_);
		mapCondVar_.wait( uDLock, [&, this]{ return hasNewMap_; });
		hasNewMap_ = false;
		boost::shared_lock<boost::shared_mutex> sPLock(pathMutex_);

		bool isPathGood = true;
		for (auto it : path_) {
			u32 k = (it.y / pMap_->info.resolution) * pMap_->info.width + (it.x / pMap_->info.resolution);
			isPathGood &= (pMap_->data[k] == 0);
		}

		sPLock.unlock();
		uDLock.unlock();

		if (!isPathGood) {
			hasNewGoal_ = true;
			goalCondVar_.notify_one();
		}
	}
}



void Navigator::followPath_()
{
	while (ros::ok())
	{
		std::unique_lock<boost::shared_mutex> uPLock(pathMutex_);
		pathCondVar_.wait( uPLock, [&, this]{ return !path_.empty(); });
		hasNewPath_ = false;
		dPosition localGoal = path_.front();
		path_.pop_front();
		uPLock.unlock();

		dPosition robotPosition;
		auto sqr = [](auto a) { return a * a; };
		std::unique_lock<boost::shared_mutex> uDLock(dataMutex_, std::defer_lock);
		std::unique_lock<boost::shared_mutex> uELock(engineMutex_, std::defer_lock);
		auto distance = [&]() { return (sqrt(sqr(robotPosition.x - localGoal.x) + sqr(robotPosition.y - localGoal.y))); };
		while (!hasNewPath_) {
			uDLock.lock();
			odometryCondVar_.wait( uDLock, [&, this]{ return hasNewOdometry_; });

			robotPosition.x = pOdometry_->pose.pose.position.x - pMap_->info.origin.position.x;
			robotPosition.y = pOdometry_->pose.pose.position.y - pMap_->info.origin.position.y;
			robotPosition.theta = getAngleFromQuaternion_(pOdometry_->pose.pose.orientation);
			uDLock.unlock();

			if (distance() < 0.1)
				break;	

			uELock.lock();
			engine_.angle = robotPosition.theta - localGoal.theta;
			engine_.speed = 1;
			engine_.direction = 1;
			uELock.unlock();
		}
		uELock.lock();
		engine_.angle = 0;
		engine_.speed = 0;
		engine_.direction = 0;
		uELock.unlock();
	}
}



vec_uPosition Navigator::buildPath_(vec_vec_bool& map, uPosition& robotPosition, uPosition& goalPosition)
{
	vec_uPosition path;
	auto cmp = [](auto t1, auto t2){ return std::get<0>(t1) > std::get<0>(t2); };
	std::priority_queue<std::tuple<double, u32, u32>, std::vector<std::tuple<double, u32, u32>>, decltype(cmp)> queue(cmp);
	vec_vec_pair_u32x p(map.size(), vec_pair_u32x(map[0].size(), { 0, 0 }));
	std::vector<std::vector<char>> v(map.size(), std::vector<char>(map[0].size(), false));
	std::vector<std::vector<u32>> g(map.size(), std::vector<u32>(map[0].size(), UINT32_MAX));
	const int minFreedomRadius = 0;
	auto h = [&](u32 x, u32 y) { return sqrt(sqr(goalPosition.x - x) + sqr(goalPosition.y - y)); };
	auto f = [&](u32 x, u32 y) { return g[x][y] + h(x, y); };
	auto check = [&](int x, int y) { return 0 <= x && x < map.size() && 0 <= y && y < map[0].size(); };
	auto hasEnoughSpace = [&](u32 x, u32 y) { 
		u32 f = 0; 
		for (int i = -minFreedomRadius; i <= minFreedomRadius; ++i) 
			for (int j = -minFreedomRadius; j <= minFreedomRadius; ++j)
				f += map[x + i][y + j];
		return f > 0.92 * minFreedomRadius * minFreedomRadius;
	};

	bool found = false;
	queue.push(std::make_tuple(0.0, robotPosition.x, robotPosition.y ));
	v[robotPosition.x][robotPosition.y] = true;
	g[robotPosition.x][robotPosition.y] = 0;
	while (!queue.empty()) {
		auto cur = queue.top();
		queue.pop();

		if (std::get<1>(cur) == goalPosition.x && std::get<2>(cur) == goalPosition.y) {
			found = true;
			break;
		}

		for (int i = -1; i <= 1; ++i)
			for (int j = -1; j <= 1; ++j)
				if (check(std::get<1>(cur) + i, std::get<2>(cur) + j) && map[std::get<1>(cur) + i][std::get<2>(cur) + j])
					if (!v[std::get<1>(cur) + i][std::get<2>(cur) + j] && hasEnoughSpace(std::get<1>(cur) + i, std::get<2>(cur) + j)) {
						if (g[std::get<1>(cur) + i][std::get<2>(cur) + j] == UINT32_MAX) {
							g[std::get<1>(cur) + i][std::get<2>(cur) + j] = g[std::get<1>(cur)][std::get<2>(cur)] + 1;
						}
						queue.push(std::make_tuple( f(std::get<1>(cur) + i, std::get<2>(cur) + j), std::get<1>(cur) + i, std::get<2>(cur) + j ));
						v[std::get<1>(cur) + i][std::get<2>(cur) + j] = true;			
						p[std::get<1>(cur) + i][std::get<2>(cur) + j] = { std::get<1>(cur), std::get<2>(cur) };
					}

	}

	if (found) {
		auto x = goalPosition.x;
		auto y = goalPosition.y;
		while (x != robotPosition.x || y != robotPosition.y) {
			auto x0 = x;
			auto y0 = y;
			path.push_back({x0, y0, 0});
			x = p[x0][y0].first;
			y = p[x0][y0].second;
		}
	}

	std::reverse(path.begin(), path.end()); 

	return path;
}



void Navigator::mapCallback_(const nav_msgs::OccupancyGrid::ConstPtr& pMap)
{
	boost::shared_lock<boost::shared_mutex> sDLock(dataMutex_);
	hasNewMap_ = true;
	pMap_ = pMap;
	mapCondVar_.notify_one();
}



void Navigator::goalCallback_(const geometry_msgs::PoseStamped::ConstPtr& pGoal)
{
	boost::shared_lock<boost::shared_mutex> sDLock(dataMutex_);
	hasNewGoal_ = true;
	pGoal_ = pGoal;
	goalCondVar_.notify_one();
}



void Navigator::odometryCallback_(const nav_msgs::Odometry::ConstPtr& pOdometry)
{
	boost::shared_lock<boost::shared_mutex> sDLock(dataMutex_);
	hasNewOdometry_ = true;
	pOdometry_ = pOdometry;
	odometryCondVar_.notify_one();
}



double Navigator::getAngleFromQuaternion_(const geometry_msgs::Quaternion& quaternion)
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
