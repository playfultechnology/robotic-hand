// DEFINES
#define GET_LOW_BYTE(A) (uint8_t)((A))
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
#define FRAME_HEADER            0x55
#define CMD_SERVO_MOVE          0x03
#define CMD_ACTION_GROUP_RUN    0x06
#define CMD_ACTION_GROUP_STOP   0x07
#define CMD_ACTION_GROUP_SPEED  0x0B

// INCLUDES
// ESP32 library for Bluetooth LE
#include "BLEDevice.h"
#include <Wire.h>
#include "lcdgfx.h"

// CONSTANTS
// BLE device, service, and characteristic UID of the robot hand.
// These values were identified using the "BLE scanner" app on an Android phone
// and connecting to the device advertised as "Hiwonder"
static BLEAddress pServerAddress("48:87:2D:62:A8:89");
static BLEUUID serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID("0000ffe1-0000-1000-8000-00805f9b34fb");

// GLOBALS
// The calculated output values sent to the servo controller
uint16_t ServoPwmSet[5] = {3050 - 1950, 1950, 1950, 1950, 1950};

//1100=clenched, 1950=hand open
uint16_t clenched[5] = {3050 - 1100, 1100, 1100, 1100, 1100};
uint16_t open[5] = {3050 - 1950, 1950, 1950, 1950, 1950};
uint16_t middleFinger[5] = {3050 - 1100, 1100, 1950, 1100, 1100};


// BLE characteristic of the connected device
static BLERemoteCharacteristic* pRemoteCharacteristic;

DisplaySSD1306_128x64_I2C display(-1); // or (-1,{busId, addr, scl, sda, frequency}). This line is suitable for most platforms by default

void setup() {
  Serial.begin(115200);
  Serial.println(__FILE__ __DATE__);

  Wire.begin(5, 4);

  Serial.print("Initialising BLE...");
  BLEDevice::init("");
  delay(500);
  BLEClient*  pClient = BLEDevice::createClient();
  Serial.println(F("Done."));

  if (pClient->connect(pServerAddress)) {
    Serial.println("Connected!");
  } else Serial.println("Failed to connect");
 
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.println("Failed to get service");
    return;
  } else Serial.println("Got service");
 
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("Failed to get characteristic");
    return;
  } else Serial.println("Got characteristic");

  display.setFixedFont( ssd1306xled_font6x8 );
  display.begin();
  display.clear();
  display.printFixed(0,  8, "Glove Control", STYLE_NORMAL);

  for(int i=0; i<2; i++) {
    moveServos(open);
    delay(2000);
    moveServos(clenched);
    delay(2000);
    moveServos(middleFinger);
    moveServo(6, 1100, 500);
    delay(500);
    moveServo(6, 1950, 500);
    delay(500);
    moveServo(6, 1500, 500);
    delay(500);
    moveServos(clenched);
  }
}

void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time) {
  uint8_t buf[11];
  if (servoID > 31 || !(Time > 0)) {
    return;
  }
  buf[0] = FRAME_HEADER;             
  buf[1] = FRAME_HEADER;
  buf[2] = 8; // Data Length (excluding frame header)              
  buf[3] = CMD_SERVO_MOVE;              
  buf[4] = 1;                         
  buf[5] = GET_LOW_BYTE(Time);        
  buf[6] = GET_HIGH_BYTE(Time);        
  buf[7] = servoID;             
  buf[8] = GET_LOW_BYTE(Position);        
  buf[9] = GET_HIGH_BYTE(Position);        
  pRemoteCharacteristic->writeValue(buf, 10);
}

void moveServos(uint16_t* servoData ) {
  int Time = 30;
  int numServos = 5;
  uint8_t buf[numServos * 3 + 8];
  memset(buf, 0, sizeof(buf));
  
  buf[0] = FRAME_HEADER;  
  buf[1] = FRAME_HEADER;
  buf[2] = numServos * 3 + 5; // Data Length (excluding frame header)
  buf[3] = CMD_SERVO_MOVE;
  buf[4] = numServos;        
  buf[5] = GET_LOW_BYTE(Time); 
  buf[6] = GET_HIGH_BYTE(Time);

  uint8_t index = 7;
  for(uint8_t i=0; i<numServos; i++) { 
    buf[index++] = i+1;
    buf[index++] = GET_LOW_BYTE(servoData[i]); 
    buf[index++] = GET_HIGH_BYTE(servoData[i]);
  }
  pRemoteCharacteristic->writeValue(buf, buf[2]+ 2);
}

void stopActionGroup() {
  uint8_t buf[4];
  buf[0] = FRAME_HEADER; 
  buf[1] = FRAME_HEADER;
  buf[2] = 2; 
  buf[3] = CMD_ACTION_GROUP_STOP; 
  pRemoteCharacteristic->writeValue(buf, 7);
}

void runActionGroup(uint8_t numOfAction, uint16_t Times) {
  uint8_t buf[7];
  buf[0] = FRAME_HEADER;
  buf[1] = FRAME_HEADER;
  buf[2] = 5;
  buf[3] = CMD_ACTION_GROUP_RUN; 
  buf[4] = numOfAction;
  buf[5] = GET_LOW_BYTE(Times); 
  buf[6] = GET_HIGH_BYTE(Times);
  pRemoteCharacteristic->writeValue(buf, 7);
}

void loop() {
  static int i=1100;
  static int deltas[5] = {10, 10, 10, 10, 10};
  for(int i=0; i<5; i++) {
    ServoPwmSet[i] += deltas[i];
    if(ServoPwmSet[i] > 1950) deltas[i] = -10;
    if(ServoPwmSet[i] < 1100) deltas[i] = 10;

    moveServo(i+1, ServoPwmSet[i], 10);
    delay(10);
  }
}
