#include <esp_now.h>
#include <WiFi.h>

// ------------------------------ ESP NOW start ------------------------------ //
// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t receiverMacAddress[] = {0x34,0x85,0x18,0x42,0x70,0xF4};  //34:85:18:42:70:F4

struct PacketData
{
  byte rxAxisValue;
  byte ryAxisValue;
  byte potValue;
};
PacketData data;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t ");
  Serial.println(status);
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
}
// ------------------------------ ESP NOW end ------------------------------ //


// ------------------------------ JOYSTICK MAPING start ------------------------------ //
//This function is used to map 0-4095 joystick value to 0-254. hence 127 is the center value which we send.
//It also adjust the deadband in joystick.
//Jotstick values range from 0-4095. But its center value is not always 2047. It is little different.
//So we need to add some deadband to center value. in our case 1800-2200. Any value in this deadband range is mapped to center 127.
byte mapAndAdjustJoystickDeadBandValues(int value)
{
  if (value >= 2200)
  {
    value = map(value, 2200, 4095, 35, 70);
  }
  else if (value <= 1900)
  {
    value = map(value, 1900, 0, 35, 0);  
  }
  else
  {
    value = 35;
  }
  Serial.println(value);  
  return value;
}
// ------------------------------ JOYSTICK MAPING end ------------------------------ //


// ------------------------------ JOYSTICK MAPING start ------------------------------ //
//This function is used to map 0-4095 joystick value to 0-254. hence 127 is the center value which we send.
//It also adjust the deadband in joystick.
//Jotstick values range from 0-4095. But its center value is not always 2047. It is little different.
//So we need to add some deadband to center value. in our case 1800-2200. Any value in this deadband range is mapped to center 127.
byte mapAndAdjustPotentiometerValues(int value)
{
  int v = map(value, 0, 4095, 0, 255);
  Serial.println(v);  
  return v;
}
// ------------------------------ JOYSTICK MAPING end ------------------------------ //




// ------------------------------ SETUP start ------------------------------ //
void setup() 
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else
  {
    Serial.println("Succes: Initialized ESP-NOW");
  }

  esp_now_register_send_cb(OnDataSent);

  // Register peer

  esp_now_peer_info_t *peer = (esp_now_peer_info_t*) malloc(sizeof(esp_now_peer_info_t));
  peer->channel = 0; // Same channel as wifi softAP or station
  peer->ifidx = WIFI_IF_STA; // ESP32 soft-AP interface
  peer->encrypt = false; // No LMK set
  memcpy(peer->peer_addr, receiverMacAddress, 6); // MAC address of new peer
  ESP_ERROR_CHECK(esp_now_add_peer(peer)); // add peer to the list

  
  
  // Add peer        
  if (esp_now_add_peer(peer) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  else
  {
    Serial.println("Succes: Added peer");
  } 

     delay(10000);
}
// ------------------------------ SETUP end ------------------------------ //



// ------------------------------ LOOP start ------------------------------ //
void loop() 
{
  data.rxAxisValue    = mapAndAdjustJoystickDeadBandValues(analogRead(34));
  data.ryAxisValue    = mapAndAdjustJoystickDeadBandValues(analogRead(35));
  data.potValue       = mapAndAdjustPotentiometerValues(analogRead(32));
  Serial.println(analogRead(32));

  
  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &data, sizeof(data));
  if (result == ESP_OK) 
  {
    Serial.println("Sent with success");
  }
  else 
  {
    Serial.println("Error sending the data");
  }    
  
  delay(50);
}
// ------------------------------  LOOP end ------------------------------ //





