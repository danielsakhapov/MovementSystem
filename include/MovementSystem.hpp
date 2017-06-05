#pragma once

#include <mutex>
#include <queue>
#include <thread>
#define _USE_MATH_DEFINES

#include <cmath>
#include <deque>
#include <vector>
#include <utility>
#include <cstdint>
#include <boost/thread.hpp>
#include <condition_variable>
#include <boost/thread/shared_mutex.hpp>

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"

class MovementSystem
{
	struct MapCellInfo {
		bool isFree;
		uint32_t wayLength;
		struct {
			uint32_t x;
			uint32_t y;
		} parent;
	};

public:
	struct Engine {
		int angle;
		double speed;
	};

	MovementSystem(int, char* [], ros::NodeHandle&);
	~MovementSystem();

	MovementSystem(const MovementSystem&) = delete;
	MovementSystem& operator=(const MovementSystem&) = delete;

	MovementSystem(MovementSystem&&) = delete;
	MovementSystem& operator=(MovementSystem&MovementSystem) = delete;

	MovementSystem::Engine& getEngine();

private:
	Engine _engine;

	std::condition_variable _pathCondVar;

	std::mutex _pathMutex;
	boost::shared_mutex _dataMutex;

	std::deque<std::pair<double, double>> _path;

	nav_msgs::OccupancyGrid::ConstPtr _pMap;
	nav_msgs::Odometry::ConstPtr _pOdometry;
	geometry_msgs::PoseStamped::ConstPtr _pGoal;

	ros::Publisher _pathPublisher;

	ros::Subscriber _mapSubcriber;
	ros::Subscriber _goalSubcriber;
	ros::Subscriber _odometrySubcriber;

	void _findPath();
	void _checkPath();
	void _moveByPath();

	double _toEulerianAngle();

	void _mapCallback(const nav_msgs::OccupancyGrid::ConstPtr&);
	void _odometryCallback(const nav_msgs::Odometry::ConstPtr&);
	void _goalCallback(const geometry_msgs::PoseStamped::ConstPtr&);
};

