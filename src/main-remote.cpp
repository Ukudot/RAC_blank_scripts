#ifdef REMOTE
# include <math.h>
# include <iostream>
# include <esp_now.h>
# include "esp_wifi.h"
# include <WiFi.h>
# include <batteryMonitor.h>
# include <ledUtility.h>
# include "esp_log.h"
# include "mac.h"
# include <stdarg.h>

static const char *TAG = "MAIN";
//------------ turn on generic serial printing

# define RANGE 512
# define DEBUG_PRINTS
//data that will be sent to the receiver

typedef struct {
  int16_t speedmotorLeft;
  int16_t speedmotorRight;
  int16_t packetArg1;
  int16_t packetArg2;
  int16_t packetArg3;
  int16_t packetArg4;
}
packet_t;


packet_t sentData;
packet_t recData;

BatteryMonitor Battery = BatteryMonitor();

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

//---------------------------------------ESP_NOW Variables


String success;
esp_now_peer_info_t peerInfo;
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == 0) {
    success = "Delivery Success :)";
  }
  else {
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&recData, incomingData, sizeof(recData));
}




//---------------------------------------HARDWARE DEPENDANT Variables
// one ifdef case per hardware to speed up modularity of the code

//RAC standard remote
const int steerPot = 7;
const int accPot = 10;
const int leverPot = 8;
//const int trimPot = 39;

const int rightBtn = 2;
const int leftBtn = 4;
const int topBtn = 5;
//const int lowSwitch = 32;
//const int topSwitch = 25;
LedUtility Led(21);

//customisable vars
int analogRes = 10;
int analogReadMax = (1 << analogRes)-1;


//variables for the sketch
int leverValue = 0;

unsigned long current_time = 0;

int handle_blink(){
  if(Battery.isLow()){
    Led.setBlinks(1,1000);
    return 1;
  }
  Led.setBlinks(0);
  Led.ledOn();
  return 0;
}


void setup() {
  //store_values(); // uncomment only to initialize mem
  analogReadResolution(analogRes);
  analogSetAttenuation(ADC_11db);
  pinMode(rightBtn, INPUT_PULLUP);
  pinMode(leftBtn, INPUT_PULLUP);
  pinMode(topBtn, INPUT_PULLUP);
  //pinMode(lowSwitch, INPUT_PULLUP);
  //pinMode(topSwitch, INPUT_PULLUP);
  Led.init();
  Led.setBlinks(1,150);
  delay(2000);
#ifdef DEBUG_PRINTS
  Serial.begin(115200);
  Serial.println("RAC GENERIC BOT");
#endif


  //---------------------------------------ESP NOW setup
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(5, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, robotAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  char macStr[18];
  Serial.print("Packet from: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           peerInfo.peer_addr[0], peerInfo.peer_addr[1], peerInfo.peer_addr[2], peerInfo.peer_addr[3], peerInfo.peer_addr[4], peerInfo.peer_addr[5]);
  Serial.print("sending to: ");
  Serial.println(macStr);
  esp_now_register_recv_cb(OnDataRecv);
  Led.setBlinks(0);
  Led.ledOn();
}



void loop() {
  // battery

  handle_blink();

  //read pots values
  int strValue = analogRead(steerPot);
  delay(3);
  int accValue = analogRead(accPot);
  delay(3);
  int leverValue = analogRead(leverPot);
  delay(3);
  current_time = millis(); 
  bool rightValue = !digitalRead(rightBtn);
  bool leftValue = !digitalRead(leftBtn);
  bool topValue = !digitalRead(topBtn);
  
  // vvvv ----- YOUR AWESOME CODE HERE ----- vvvv //
  // ft_printf("steer: %i\taccel: %i\tlever: %i\trbtn: %i\t\tlbtn: %i\t\ttbtn: %i\n",
  //	  strValue, accValue, leverValue, rightValue, leftValue, topValue);
  if (accValue > 400 && accValue < 600)
    accValue = 512;
  if (strValue > 500 && strValue < 535)
    strValue = 512;
  if (leverValue > 510 && leverValue < 540)
   leverValue = 500;
  strValue = map(strValue, 0, 1023, -RANGE, RANGE);
  accValue = map(accValue, 0, 1023, -RANGE, RANGE);
  sentData.speedmotorLeft = constrain(- strValue - accValue, -RANGE, RANGE);
  sentData.speedmotorRight = constrain(strValue - accValue, -RANGE, RANGE);
  sentData.packetArg1 = topValue;
  sentData.packetArg2 = leftValue;
  sentData.packetArg3 = rightValue;
  sentData.packetArg4 = constrain(map(leverValue, 330, 670, 10, -10), -10, 10);
  //Serial.println(leverValue);
  // -------------------------------------------- //
  esp_err_t result = -1;
  result = esp_now_send(robotAddress, (uint8_t *) &sentData, sizeof(sentData));
  if (result == ESP_OK) {
    //Serial.println("Sent with success");
  } else {
    //Serial.println("Error sending the data");
  }
  delay(10);
}
#endif
