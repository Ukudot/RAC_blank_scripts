#ifdef ROBOT
#include <esp_now.h>
#include "esp_wifi.h"
#include <WiFi.h>
#include <motorControl.h>
#include <batteryMonitor.h>
#include <ledUtility.h>
#include "esp_log.h"
#include "mac.h"
#include <stdarg.h>
#include <iostream>
#include <time.h>
static const char *TAG = "MAIN";

#define RANGE 512
#define DEBUG_PRITNS
#define weapPot 7
//---------------------------------------Our functions
void	ft_printf(const char *str, ...)
{
	va_list	args;
	int		i;

	va_start(args, str);
	i = 0;
	while (str[i])
	{
		if (str[i] == 37 && ++i)
		{
			if (str[i] == 'c')
				Serial.print(va_arg(args, int));
			else if (str[i] == 's')
				Serial.print(va_arg(args, char *));
			else if (str[i] == 'd' || str[i] == 'i')
				Serial.print(va_arg(args, int));
			else
				Serial.print(str[i]);
		}
		else
			Serial.print(str[i]);
		i++;
	}
	va_end(args);
}
//---------------------------------------
//------------ turn on generic serial printing
//#define DEBUG_PRINTS


#define MOTOR_A_IN1 8
#define MOTOR_A_IN2 18

#define MOTOR_B_IN1 16
#define MOTOR_B_IN2 17

#define MOTOR_C_IN1 15
#define MOTOR_C_IN2 6

#define MOTOR_D_IN1 4
#define MOTOR_D_IN2 5


//RIGHT
MotorControl motor1 = MotorControl(MOTOR_B_IN1, MOTOR_B_IN2, 0, RANGE); 
//LEFT
MotorControl motor2 = MotorControl(MOTOR_A_IN1, MOTOR_A_IN2, 0, RANGE);
//WPN
MotorControl motor3 = MotorControl(MOTOR_C_IN1, MOTOR_C_IN2, 0, RANGE);
//TAIL
MotorControl motor4 = MotorControl(MOTOR_D_IN1, MOTOR_D_IN2, 0, RANGE);

BatteryMonitor Battery = BatteryMonitor();

LedUtility Led = LedUtility();

typedef struct {
  int16_t speedmotorLeft;
  int16_t speedmotorRight;
  int16_t packetArg1;
  int16_t packetArg2;
  int16_t packetArg3;
  int16_t packetArg4;
}
packet_t;
packet_t recData;


bool failsafe = false;
unsigned long failsafeMaxMillis = 400;
unsigned long lastPacketMillis = 0;
unsigned long	current_time = 0;

int recLpwm = 0;
int recRpwm = 0;
int recArg1 = 0;
int recArg2 = 0;
int recArg3 = 0;
int recArg4 = 0;

int     btntop = 4;
clock_t t;

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&recData, incomingData, sizeof(recData));
  recLpwm = recData.speedmotorLeft;
  recRpwm = recData.speedmotorRight;
  recArg1 = recData.packetArg1;
  recArg2 = recData.packetArg2;
  recArg3 = recData.packetArg3;
  recArg4 = recData.packetArg4;
  lastPacketMillis = millis();
  failsafe = false;
}

int handle_blink(){
  if(Battery.isLow()){
    Led.setBlinks(1,1000);
    return 1;
  }
  if (failsafe){
    Led.setBlinks(2,500);
    return -1;
  }
  Led.setBlinks(0);
  Led.ledOn();
  return 0;
}

void setup()
{
#ifdef DEBUG_PRINTS
  Serial.begin(115200);
  Serial.println("Ready.");
#endif
  analogReadResolution(10);
  Led.init();
  delay(20);
  Led.setBlinks(1,150);
  delay(2000);
  Battery.init();
  delay(20);


  analogSetAttenuation(ADC_11db);
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
  delay(500);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(5, WIFI_SECOND_CHAN_NONE);
  if (esp_wifi_set_mac(WIFI_IF_STA, &robotAddress[0]) != ESP_OK)
  {
    Serial.println("Error changing mac");
    return;
  }
  Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  Led.setBlinks(0);
  Led.ledOn();
}

// motor1 = right
// motor2 = left inverted
void loop()
{
	unsigned long	old_time = current_time;
	current_time = millis();
	int potlevel = analogRead(weapPot);

	if (current_time - lastPacketMillis > failsafeMaxMillis)
		failsafe = true;
	handle_blink();
	if (failsafe)
	{
		std::cout << potlevel << std::endl;
		motor1.setSpeed(0);
		motor2.setSpeed(0);
		motor3.setSpeed(0);
		motor4.setSpeed(0);
	}
	else
	{
  // vvvv ----- YOUR AWESOME CODE HERE ----- vvvv //
	if (recData.packetArg2 && !recData.packetArg1 && !recData.packetArg3)
		motor4.setSpeed(RANGE * 9 / 10);
	else if (recData.packetArg3 && !recData.packetArg1 && !recData.packetArg2)
		motor4.setSpeed(-(RANGE * 9 / 10));
	else
	{
		motor4.setSpeed(0);
		motor4.unlockMotor(); // gpanico
	}

	switch (btntop)
	{
		case 1: // sale
			motor3.setSpeed(RANGE);
    		btntop = (potlevel <= 50) ? 3 : btntop;
			break ;
		// case 2: // fermo in alto
		// 	motor3.setSpeed(0);
		// 	btntop = (recData.packetArg1 && !recData.packetArg2 && !recData.packetArg3) ? 3 : btntop;
		// 	break ;
		case 3: // scende
			if (potlevel < 200)
				motor3.setSpeed(-RANGE);
			else
				motor3.setSpeed((-1) * (330 - potlevel) / (float) (current_time - old_time));
			//std::cout << (-1) * (330 - potlevel) / (float) (current_time - old_time) << std::endl;
			btntop = (potlevel >= 310) ? 4 : btntop;
			break ;
		case 4: // fermo in basso
			motor3.setSpeed(0);
			btntop = (recData.packetArg1 && !recData.packetArg2 && !recData.packetArg3) ? 1 : btntop;
			break ;
	}
	if (recData.packetArg1 && recData.packetArg2 && recData.packetArg3)
	{
		if (potlevel > 50)
			motor3.setSpeed(RANGE);
		else
			motor3.setSpeed(-RANGE);
	}
	if ((recData.packetArg4 > 0 && potlevel > 30) || (recData.packetArg4 < 0 && potlevel < 325))
		motor3.setSpeed(recData.packetArg4 * RANGE / 10);
	motor1.setSpeed(recData.speedmotorLeft);
	motor2.setSpeed(-recData.speedmotorRight);
    // ft_printf("m1: %i\tm2: %i\n", recData.speedmotorLeft, recData.speedmotorRight);
	// -------------------------------------------- //
	}
	delay(2);
}
#endif