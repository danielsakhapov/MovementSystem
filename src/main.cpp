#include <memory>
#include <string>

#include <std_msgs/String.h>

#include "Navigator.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "Navigator");

	ros::NodeHandle nodeHandler;

	std::unique_ptr<Navigator> pNavigator(new Navigator(argc, argv, nodeHandler));

	ros::AsyncSpinner spinner(3);
	spinner.start();

	ros::Publisher pub = nodeHandler.advertise<std_msgs::String>("Navigator/arduinoCommands", 1000);

	ros::Rate loop_rate(10);

	std_msgs::String msg;

	while (ros::ok())
	{
		Engine engine = pNavigator->getEngine();		

		std::stringstream ss;
		if (engine.speed == 0)
			ss << "0 0 ";
		else {
			if (abs(engine.angle) < 15)
				ss << "1 1 ";
			else
				if (0 < engine.angle)
					ss << "1 0 ";
				else
					ss << "0 1 ";
		}
		msg.data = ss.str();

		pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
		
		std::cout << ss.str() << ' ' << engine.angle << ' ' << engine.speed << std::endl;
	}

	return 0;
}
