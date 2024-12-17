#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050_tockn.h>



// ------------------------------ SENSOR PINS start ------------------------------ //
#define SDA 20 // NARANDZASTA 
#define SCL 21 // ZUTA
// ------------------------------ SENSOR PINS end ------------------------------ //


// ------------------------------ BME280 - ALTITUDE start ------------------------------ //
Adafruit_BME280 bme; // I2C
//Replace with your location's sea-level pressure in hPa
float seaLevelPressure = 1020.2;
float start_altitude;
// ------------------------------ BME280 - ALTITUDE end ------------------------------ //


// ------------------------------ MPU6050 - STABILIZATION start ------------------------------ //
MPU6050 mpu6050(Wire); // Attach the IIC
// ------------------------------ MPU6050 - STABILIZATION end ------------------------------ //



// ------------------------------ SERVO MOTORS start ------------------------------ //

byte ESC_PIN = 14;
byte SERVO_PINS[4] = {10, 11, 12, 13};
#define SERVO_FRQ 50 //define the pwm frequency
#define SERVO_BIT 12 //define the pwm precision





void servo_set_pin(int pin, byte chn);
void servo_set_angle(int angle, byte chn);
// ------------------------------ SERVO MOTORS end ------------------------------ //


// ------------------------------ ESP NOW start ------------------------------ //
#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal

unsigned long lastRecvTime = 0;

struct PacketData
{
  byte rxAxisValue;
  byte ryAxisValue;
  byte potValue;
};
PacketData receiverData;

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info* mac, const uint8_t* incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  mapAndWriteValues();  
  lastRecvTime = millis(); 
}
// ------------------------------ ESP NOW end ------------------------------ //

// ------------------------------ CLAMP and MAP FUNCTION start ----------------------------------- //
int clamp(int val, int min, int max) {
    if (val < min) {
        return min;
    } else if (val > max) {
        return max;
    } else {
        return val;
    }
}

int mapInt(int x, int in_min, int in_max, int out_min, int out_max) {
    const int run = in_max - in_min;
    if(run == 0){
        log_e("map(): Invalid input range, min == max");
        return -1; // AVR returns -1, SAM returns 0
    }
    const int rise = out_max - out_min;
    const int delta = x - in_min;
    return (delta * rise) / run + out_min;
}

// ------------------------------ CLAMP and MAP FUNCTION end ----------------------------------- //


// ------------------------------ CONTROL AND STABILIZATION start ------------------------------ //

void mapAndWriteValues()
{
  ESC_CONTROL();
  mpu6050.update();
  FrontStabilization();
  BackStabilization();
  VerticalStabilization();
  //Wheels();
  /*
  Serial.println(receiverData.lxAxisValue);
  Serial.println(receiverData.lyAxisValue);
  Serial.println(receiverData.rxAxisValue);
  Serial.println(receiverData.ryAxisValue);
  
  */
  //ledcWrite(SERVO_CHN[0], receiverData.lxAxisValue);
  //ledcWrite(SERVO_CHN[1], receiverData.lyAxisValue);
  //ledcWrite(SERVO_CHN[2], receiverData.rxAxisValue);
  //ledcWrite(SERVO_CHN[3], receiverData.ryAxisValue);
  /*
  servo_set_angle(receiverData.rxAxisValue, 0);
  servo_set_angle(receiverData.ryAxisValue, 1);
  servo_set_angle(receiverData.rxAxisValue, 2);
  servo_set_angle(receiverData.ryAxisValue, 3); 
  */
}

void Wheels()
{
  // Wheels
/*
  Serial.print("Altitude = ");
  Serial.print(bme.readAltitude(seaLevelPressure));
  Serial.println(" m");
*/
  if(bme.readAltitude(seaLevelPressure) > start_altitude + 2)
    ledcWrite(SERVO_PINS[3], 512);
  else
    ledcWrite(SERVO_PINS[3], 307);
  
}

void FrontStabilization()
{

      Serial.print("RX = ");
      Serial.print(receiverData.rxAxisValue);
      Serial.println(" degrees");

  //float gyroY = mpu6050.getAngleX() - 35 + receiverData.rxAxisValue;

  float gyroY = mpu6050.getAngleX();

      Serial.print("Y = ");
      Serial.print(gyroY);
      Serial.println(" degrees");

  int servoPosition = mapInt(gyroY, -35, 35, 160, 360);
  servoPosition = clamp(servoPosition, 160, 360);

      Serial.print("Y = ");
      Serial.print(servoPosition);
      Serial.println(" degrees");


/*
      ledcWrite(SERVO_CHN[0], servoPosition);
ledcWrite(SERVO_CHN[1], servoPosition);
change this code so the first line writes the real value od servoPosition and the second one the opposite
If the first one writes 30 the second one should write -30
*/
  //FrontWings.write(90 + servoPosition);
  ledcWrite(SERVO_PINS[0], servoPosition);
  ledcWrite(SERVO_PINS[1], servoPosition);
}

void BackStabilization()
{
/*
      Serial.print("RY = ");
      Serial.print(receiverData.ryAxisValue);
      Serial.println(" degrees");
*/
  float gyroX = mpu6050.getAngleY() - 35 + receiverData.ryAxisValue;
/*
      Serial.print("X = ");
      Serial.print(receiverData.ryAxisValue);
      Serial.println(" degrees");
*/
  int servoPosition = mapInt(gyroX, -35, 35, 80, 460);

  servoPosition = clamp(servoPosition, 80, 460);
/*
      Serial.print("X = ");
      Serial.print(servoPosition);
      Serial.println(" degrees");
*/
  //FrontWings.write(90 + servoPosition);
  ledcWrite(SERVO_PINS[2], servoPosition);
}

void VerticalStabilization()
{
  float gyroZ = - 35 + receiverData.rxAxisValue;
/*
      Serial.print("Z = ");
      Serial.print(gyroZ);
      Serial.println(" ");
*/
  //int servoPosition = mapInt(gyroZ, -35, 35, 500, 100);

  int servoPosition = mapInt(gyroZ, -35, 35, 80, 460);

  servoPosition = clamp(servoPosition, 80, 460);
/*
      Serial.print("Y = ");
      Serial.print(servoPosition);
      Serial.println(" degrees");
*/
  //FrontWings.write(90 + servoPosition);
  ledcWrite(SERVO_PINS[3], servoPosition);
}

void ESC_CONTROL()
{
  int speed = mapInt(receiverData.potValue, 0, 255, 1005, 1308);
/*
      Serial.print("Speed = ");
      Serial.print(speed);
      Serial.println("");
*/
  
  //FrontWings.write(90 + servoPosition);
  //ledcWrite(ESC_CHN, receiverData.potValue);
  ledcWrite(ESC_PIN, speed);
}
// ------------------------------ CONTROL AND STABILIZATION end ------------------------------ //






// ------------------------------ SETUP start ------------------------------ //

void servo_set_pin(int pin) {
 ledcAttach(pin, SERVO_FRQ, SERVO_BIT);
}



void setup() 
{
  //BME280
  Wire.begin(SDA, SCL);
  bme.begin(0x76);

  start_altitude = bme.readAltitude(seaLevelPressure);

  //MPU6050
  Wire.begin(SDA, SCL); // Attach the IIC pin
  mpu6050.begin();      // Initialize the MPU6050
  mpu6050.calcGyroOffsets(true); // Get the offsets value

  for(int i = 0; i < 4; i++)
  {
    servo_set_pin(SERVO_PINS[i]);
  }

  servo_set_pin(ESC_PIN);
 
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
// ------------------------------ SETUP end ------------------------------ //

// ------------------------------ LOOP start ------------------------------ //
void loop()
{
  //Check Signal lost.
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    mapAndWriteValues();  
  }
}
// ------------------------------  LOOP end ------------------------------ //
