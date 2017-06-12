#pragma once

#define _USE_MATH_DEFINES

#include <cmath>
#include <deque>
#include <vector>
#include <mutex>
#include <thread>
#include <cstdint>
#include <utility>
#include <boost/thread.hpp>
#include <condition_variable>
#include <boost/thread/shared_mutex.hpp>

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"

#include "NavigatorStructures.hpp"

class Navigator
{
public:
	Navigator(int, char* [], ros::NodeHandle&);
	~Navigator();

	Navigator(const Navigator&) = delete;
	Navigator& operator=(const Navigator&) = delete;

	Navigator(Navigator&&) = delete;
	Navigator& operator=(Navigator&&) = delete;

	Engine& getEngine();

private:
	bool _hasNewMap;
	bool _hasNewGoal;
	bool _hasNewPath;
	bool _hasNewOdometry;

	Engine _engine;

	std::deque<dPosition> _path;

	boost::shared_mutex _dataMutex;
	boost::shared_mutex _pathMutex;
	boost::shared_mutex _engineMutex;

	std::condition_variable_any _pathCondVar;
	std::condition_variable_any _mapCondVar;
	std::condition_variable_any _goalCondVar;
	std::condition_variable_any _odometryCondVar;

	nav_msgs::OccupancyGrid::ConstPtr _pMap;
	nav_msgs::Odometry::ConstPtr _pOdometry;
	geometry_msgs::PoseStamped::ConstPtr _pGoal;

	ros::Publisher _pathPublisher;

	ros::Subscriber _mapSubcriber;
	ros::Subscriber _goalSubcriber;
	ros::Subscriber _odometrySubcriber;

	void _findPath();
	void _checkPath();
	void _followPath();

	vec_uPosition _buildPath(vec_vec_bool&, uPosition&, uPosition&);
	double _getAngleFromQuaternion(const geometry_msgs::Quaternion&);

	// Callback functions
	void _mapCallback(const nav_msgs::OccupancyGrid::ConstPtr&);
	void _odometryCallback(const nav_msgs::Odometry::ConstPtr&);
	void _goalCallback(const geometry_msgs::PoseStamped::ConstPtr&);
};
