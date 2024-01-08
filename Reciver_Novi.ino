#include <esp_now.h>
#include <WiFi.h>

byte SERVO_PINS[4] = {10, 11, 12, 13};
byte SERVO_CHN[4] = {0, 1, 2, 3};

#define SERVO_FRQ 50 //define the pwm frequency
#define SERVO_BIT 12 //define the pwm precision

void servo_set_pin(int pin, byte chn);
void servo_set_angle(int angle, byte chn);

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal

unsigned long lastRecvTime = 0;

struct PacketData
{
  unsigned int lxAxisValue;
  unsigned int lyAxisValue;
  unsigned int rxAxisValue;
  unsigned int ryAxisValue;
};
PacketData receiverData;
   

void mapAndWriteValues()
{
  Serial.println(receiverData.lxAxisValue);
  Serial.println(receiverData.lyAxisValue);
  Serial.println(receiverData.rxAxisValue);
  Serial.println(receiverData.ryAxisValue);
  ledcWrite(SERVO_CHN[0], receiverData.lxAxisValue);
  ledcWrite(SERVO_CHN[1], receiverData.lyAxisValue);
  ledcWrite(SERVO_CHN[2], receiverData.rxAxisValue);
  ledcWrite(SERVO_CHN[3], receiverData.ryAxisValue);
  /*
  servo_set_angle(receiverData.rxAxisValue, 0);
  servo_set_angle(receiverData.ryAxisValue, 1);
  servo_set_angle(receiverData.rxAxisValue, 2);
  servo_set_angle(receiverData.ryAxisValue, 3); 
  */
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  mapAndWriteValues();  
  lastRecvTime = millis(); 
}



void servo_set_pin(int pin, byte chn) {
 ledcSetup(chn, SERVO_FRQ, SERVO_BIT);
 ledcAttachPin(pin, chn);
}

void setup() 
{
  for(int i = 0; i < 4; i++)
  {
    servo_set_pin(SERVO_PINS[i], SERVO_CHN[i]);
  }
 
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop()
{
  //Check Signal lost.
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    mapAndWriteValues();  
  }
}