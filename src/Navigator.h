#pragma once

#define _USE_MATH_DEFINES

#include <cmath>
#include <deque>
#include <mutex>
#include <queue>
#include <tuple>
#include <vector>
#include <thread>
#include <cstdint>
#include <utility>
#include <algorithm>
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
	bool hasNewMap_;
	bool hasNewGoal_;
	bool hasNewPath_;
	bool hasNewOdometry_;

	Engine engine_;

	std::deque<dPosition> path_;

	boost::shared_mutex dataMutex_;
	boost::shared_mutex pathMutex_;
	boost::shared_mutex engineMutex_;

	std::condition_variable_any pathCondVar_;
	std::condition_variable_any mapCondVar_;
	std::condition_variable_any goalCondVar_;
	std::condition_variable_any odometryCondVar_;

	nav_msgs::OccupancyGrid::ConstPtr pMap_;
	nav_msgs::Odometry::ConstPtr pOdometry_;
	geometry_msgs::PoseStamped::ConstPtr pGoal_;

	ros::Publisher pathPublisher_;

	ros::Subscriber mapSubcriber_;
	ros::Subscriber goalSubcriber_;
	ros::Subscriber odometrySubcriber_;

	void findPath_();
	void checkPath_();
	void followPath_();

	vec_uPosition buildPath_(vec_vec_bool&, uPosition&, uPosition&);
	double getAngleFromQuaternion_(const geometry_msgs::Quaternion&);

	// Callback functions
	void mapCallback_(const nav_msgs::OccupancyGrid::ConstPtr&);
	void odometryCallback_(const nav_msgs::Odometry::ConstPtr&);
	void goalCallback_(const geometry_msgs::PoseStamped::ConstPtr&);
};
