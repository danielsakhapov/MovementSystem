#include <memory>

#include "MovementSystem.hpp"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "autonomusCarMovementSystem");

	ros::NodeHandle nodeHandle;

	std::unique_ptr<MovementSystem> pMovementSystem(new MovementSystem(argc, argv, nodeHandle));

	while (ros::ok())
	{
		std::cout << "E: " << pMovementSystem->getEngine().angle << ' ' << pMovementSystem->getEngine().speed << std::endl;

		ros::spinOnce();
	}

	return 0;
}
