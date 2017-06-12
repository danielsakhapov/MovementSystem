#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <cmath>
#include <cstdio>
#include <stdlib.h>
#include <linux/serial.h>


class ArduinoCtrl
{
protected:
int arduino_fd; //дескриптор устройства
//char message[128];
bool connectionStatus;

bool connect(const char* arduinoPort);
bool disconnect();

public:
int feedback(int *sensors);
void sendCommand(const char *message,size_t size);
void deinit();
bool isConnected();
ArduinoCtrl(const char* arduino_port);
};

void* arduino_fnc(void *ptr);

