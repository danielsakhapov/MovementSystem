#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;

ros::Publisher chatter("Navigator/arduinoResponse", &str_msg);

void messageCb( const std_msgs::String& arduinoCommand) {
int nParams = 1;
for (int i = 0; i < strlen(arduinoCommand); ++i)
	if (arduinoCommand[i] == ' ')
		++nParams;

int j = 0, len = 0;
char s[strlen(arduinoCommand) + 1];
int* params = new int[nParams];
for (int i = 0; i < strlen(arduinoCommand); ++i) {
	if (arduinoCommand[i] != ' ' && i != strlen(arduinoCommand) - 1) {
		s[len] = arduinoCommand[i];
		++len;
		if (i == strlen(arduinoCommand) - 2) {
			s[len] = arduinoCommand[i + 1];
			++len;
		}
	}
	else {			
		int num = 0, f = 1;
		for (int k = 0; k < len; ++k)
			f *= 10;
		f /= 10;
		bool minus = false;
		for (int k = 0; k < len; ++k) {
			if (s[k] == '-') {
				minus = true;
				f /= 10;
				continue;
			}
			num += (s[k] - '0') * f;
			f /= 10;
		}
		if (minus)
			num *= -1;
		params[j] = num;
		++j;
		len = 0;
	}
}

        chatter.publish( &arduinoCommand );
        nh.spinOnce();
}

ros::Subscriber<std_msgs::String> sub("Navigator/arduinoCommands", &messageCb );

void setup()
{
        nh.initNode();
        nh.advertise(chatter);
        nh.subscribe(sub);
}

void loop()
{
        nh.spinOnce();
        delay(1);
}


