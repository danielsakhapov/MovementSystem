
#include "arduino.hpp"

//#define LOW_LATENCY

bool ArduinoCtrl::connect(const char* arduinoPort)
{
	arduino_fd = open(arduinoPort, O_RDWR | O_NOCTTY | O_NDELAY); // Открывает последовательный порт
	if(arduino_fd<0) //произошла ошибка при открытии порта
	{
		return false;
	}

	struct termios options; //структура содержащая настройки порта
	tcgetattr(arduino_fd, &options); //считать текущие настройки порта

	//установить скорость соединения (115200 бодов в секунду)
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	options.c_cflag &= ~PARENB; //выключить проверку четности
	options.c_cflag &= ~CSTOPB; //1 стопбит
	options.c_cflag &= ~CSIZE; //выключение битовой маски
	options.c_cflag |= CS8; //режим: 8 бит
	options.c_cflag |= ( CLOCAL | CREAD );
	//fcntl(arduino_fd, F_SETFL, FNDELAY); //0

	tcsetattr(arduino_fd, TCSANOW, &options); //применить новые настройки порта
	
	#ifdef LOW_LATENCY
	struct serial_struct serial;
	ioctl(arduino_fd, TIOCGSERIAL, &serial); 
	serial.flags |= ASYNC_LOW_LATENCY; // (0x2000)
	ioctl(arduino_fd, TIOCSSERIAL, &serial);
	#endif
	
	tcflush(arduino_fd, TCIOFLUSH);

	return true;
}

bool ArduinoCtrl::disconnect()
{
	if(close(arduino_fd)==0) //закрыть соединение
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*
 * Функция deinit() закрывает соединение с микроконтроллером.
 */
void ArduinoCtrl::deinit()
{
	if(connectionStatus)
	{
		if(disconnect())
		{
			connectionStatus = false;
		}
	}

	return;
}

/*
 * Обеспечивает обратную связь с Arduino
 * В частности получает реальную скорость движения робота от arduino
 * sensors массив из n элементов
 */
int ArduinoCtrl::feedback(int *sensors)
{
	char buffer[10];

	char current;
	int sz = read(arduino_fd, &current, 1);
	if(sz>0)
	{
		if(current == 'F')
		{
			int i=0;
			bool end_flag = false;
			int s=0;
			for(; !end_flag;i++)
			{
				while(read(arduino_fd, &current,1)!=1);
				switch(current)
				{
					case 'E':
							end_flag = true;
							break;
					case ',':
							buffer[i+1] = '\0';
							sensors[s] = atoi(buffer);
							s++;
							i=0;
							break;
					default:
							buffer[i] = current;
							break;
				}
			}
			return 1;
		}
		else if(current == 'B')
		{
			int i=0;
			bool end_flag = false;
			int s=0;
			for(; !end_flag;i++)
			{
				while(read(arduino_fd, &current,1)!=1);
				switch(current)
				{
					case 'E':
							end_flag = true;
							break;
					case ',':
							buffer[i+1] = '\0';
							sensors[s] = atoi(buffer);
							s++;
							i=0;
							break;
					default:
							buffer[i] = current;
							break;
				}
			}
			return 1;
		}
	}
	return -1;
}


/*
 * Функция send_command() отправляет параметры движения робота на микроконтроллер.
 * @engine - структура содержащая основные параметры движения робота
 */
void ArduinoCtrl::sendCommand(const char* msg, size_t size)
{
	int bytes_written = write(arduino_fd, msg, size);

	if(bytes_written<(int)size)
	{
		//LOG("[E]: Arduino: Sending data error"); //todo: ->define debug
	}

	//ioctl(arduino_fd, TCSBRK, 1);
	return;
}

/*
 * Функция isconnected() возвращает состояние соединения с микроконтроллером.
 * Возвращаемые значения:
 * "истина", если соединение открыто, "ложь" в ином случае
 */
bool ArduinoCtrl::isConnected()
{
	return connectionStatus;
}

/*
 * Конструктор класса ArduinoCtrl.
 * Отвечает за корректное создание объекта класса и открытие соединения с микроконтроллером.
 */
ArduinoCtrl::ArduinoCtrl(const char* arduino_port)
{
	arduino_fd = -1;

	if(connect(arduino_port))
	{
		connectionStatus = true;
	}
	else
	{
		printf("[E]: Can't open serial port\n");
		connectionStatus = false;
	}
}

/*
 * Функция arduino_fnc() реализует поток взаимодействия с микроконтроллером.
 * @ptr - указатель на структуру System.
 * Устанавливает соединение с микроконтроллером и начинает процесс передачи параметров движения.
 */
void* arduino_fnc(void *ptr)
{
	//////////
	unsigned int angle = 90;
	unsigned int direction = 1;
	unsigned int speed = 100;

	////////
	char message[128]; //буфер для отправки сообщения на ардушилд
	ArduinoCtrl arduinoFW("/dev/ttyUSB0");
	ArduinoCtrl arduinoBW("/dev/ttyS2");

	if(!arduinoFW.isConnected() || !arduinoBW.isConnected())
	{
		printf("[W]: Arduino isn't attached.\n");
		return NULL;
	}

	while(1)
	{
		snprintf(message, sizeof(message), "MC %d,%d,%d ", angle, direction, speed);
		arduinoFW.sendCommand(message, strlen(message));

		//int spd=arduinoFW.feedback();
		usleep(10000); //10 ms. максимальная задержка реакции робоавтомобиля
	}

	arduinoFW.deinit();
	arduinoBW.deinit();
	return NULL;
}
