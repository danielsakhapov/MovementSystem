#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

int left_weel_port_f = 24;
int left_weel_port_b = 25;

int right_weel_port_f = 22;
int right_weel_port_b = 23;

char* s = NULL;
int* params = NULL;
int nParams = 1;

std_msgs::String str_msg;

ros::Publisher chatter("Navigator/arduinoResponse", &str_msg);

void messageCb( const std_msgs::String& arduinoCommand) {
        if (s == NULL) {
                for (int i = 0; i < strlen(arduinoCommand.data); ++i)
        	        if (arduinoCommand.data[i] == ' ')
                		++nParams;
        }
        
        int j = 0, len = 0;
        if (s == NULL)
                s = new char[strlen(arduinoCommand.data) + 1];
        if (params == NULL)
                params = new int[nParams];
        for (int i = 0; i < strlen(arduinoCommand.data); ++i) {
        	if (arduinoCommand.data[i] != ' ' && i != strlen(arduinoCommand.data) - 1) {
        		s[len] = arduinoCommand.data[i];
        		++len;
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
        
        if (params[0] == 1) {
                digitalWrite (left_weel_port_f, HIGH);
                digitalWrite (left_weel_port_b, LOW);
        }
        if (params[0] == -1) {
                digitalWrite (left_weel_port_f, LOW);
                digitalWrite (left_weel_port_b, HIGH);
        }
        if (params[0] == 0) {
                digitalWrite (left_weel_port_f, LOW);
                digitalWrite (left_weel_port_b, LOW);
        }
        if (params[1] == 1) {
                digitalWrite (right_weel_port_f, HIGH);
                digitalWrite (right_weel_port_b, LOW);
        }
        if (params[1] == -1) {
                digitalWrite (right_weel_port_f, LOW);
                digitalWrite (right_weel_port_b, HIGH);
        }
        if (params[1] == 0) {
                digitalWrite (right_weel_port_f, LOW);
                digitalWrite (right_weel_port_b, LOW);
        }
        
        char st[4];
        st[0] = params[0] + '0';
        st[1] = ' ';
        st[2] = params[1] + '0';
        st[3] = '\0';
        
        str_msg.data = st;

        chatter.publish( &str_msg );
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


