#include <memory>

#include "arduino.hpp"
#include "Navigator.hpp"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "autonomusCarMovementSystem");

	ros::NodeHandle nodeHandle;

	std::unique_ptr<Navigator> pNavigator(new Navigator(argc, argv, nodeHandle));

	ros::AsyncSpinner spinner(3);
	spinner.start();

	ArduinoCtrl arduinoFW("/dev/ttyACM0");

	int16_t angle = 0;
	uint16_t speed = 0;
	uint8_t direction = 0;

	while (ros::ok())
	{
		Engine engine = pNavigator->getEngine();

		angle = engine.angle;
		direction = engine.speed;

		char bufBits[6];
		bufBits[0] = 'M';
		bufBits[1] = (angle&0xff);
		bufBits[2] = (angle>>8);
		bufBits[3] = (char)(speed&0xff);
		bufBits[4] = (char)(speed>>8);
		bufBits[5] = (char)(direction);

		arduinoFW.sendCommand(bufBits, 6);

		std::cout << engine.angle << ' ' << engine.speed << ' ' << engine.direction << std::endl;
	}

	return 0;
}
