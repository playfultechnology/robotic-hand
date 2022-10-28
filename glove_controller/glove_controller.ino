
// INCLUDES
// Software serial connection to Bluetooth module
#include <SoftwareSerial.h>
#include "LobotServoController.h"
// I2C interface
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

// DEFINES
#define BTH_RX 11
#define BTH_TX 12
// Used for car control
#define STOP       0
#define GO_FORWARD 1
#define GO_BACK    2
#define TURN_LEFT  3
#define TURN_RIGHT 4

// CONSTANTS
// Don't use GPIO A4 and A5 as those are used for I2C connection 
const byte fingerPins[] = {A0, A1, A2, A3, A6};
const byte funcButtonPin = 7;
const byte ledPins[] = {2, 3, 4, 5, 6};
const byte maxReadings[] = {230, 252, 317, 309, 305};

// Interval (ms) between sending log info to serial monitor
const int printlogInterval = 2000; 

// GLOBALS
float min_list[5] = {0, 0, 0, 0, 0};
float max_list[5] = {255, 255, 255, 255, 255};
// Raw input values sampled from each finger
float sampling[5] = {0, 0, 0, 0, 0};
// Input values after filtering/rescaling 
float data[5] = {1500, 1500, 1500, 1500, 1500};
// The calculated output values sent to the servo controller
uint16_t ServoPwmSet[5] = {1500, 1500, 1500, 1500, 1500};
bool turn_on = true;
SoftwareSerial Bth(BTH_RX, BTH_TX);
LobotServoController lsc(Bth);
int mode = 0;
bool key_state = false;

// Remap values to constrained range
float float_map(float in, float left_in, float right_in, float left_out, float right_out) {
  return (in - left_in) * (right_out - left_out) / (right_in - left_in) + left_out;
}

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float ax0, ay0, az0;
float gx0, gy0, gz0;
float ax1, ay1, az1;
float gx1, gy1, gz1;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
float radianX;
float radianY;
float radianZ;
float radianX_last;
float radianY_last;

void setup() {
  Serial.begin(115200);
  Serial.println(__FILE__ __DATE__);
  Serial.println("Please ensure fist clenched on startup");
  delay(500);
  Serial.println("Wait until LEDs flash");
  delay(500);
  Serial.println("Then fully extend fingers");
  // Initialise pins
  pinMode(funcButtonPin, INPUT_PULLUP);
  for(int i=0;i<5; i++) {
    pinMode(fingerPins[i], INPUT);
    pinMode(ledPins[i], OUTPUT);
  }
  
  // Start Bluetooth interface
  Bth.begin(9600);
  delay(250);
  // Set role of BT module(1=master/0=slave)
  // Bth.print("AT+ROLE=1");
  // If desired, change BT name
  // Bth.print("AT+NAME=Glove");
  delay(100);

  // MPU6050
  Wire.begin();
  accelgyro.initialize();
  accelgyro.setFullScaleGyroRange(3);
  accelgyro.setFullScaleAccelRange(1);
  delay(200);
  // Read initial calibration readings
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  ax_offset = ax;
  ay_offset = ay;
  az_offset = az - 8192;
  gx_offset = gx;
  gy_offset = gy;
  gz_offset = gz;
}


void finger() {
  static uint32_t timer_sampling;
  static uint32_t timer_init;
  static uint32_t timer_lsc = 0;
  static uint8_t init_step = 0;
  if (timer_lsc == 0)
    timer_lsc = millis();
  if (timer_sampling <= millis()) {
    for (int i = 0; i <5; i++) {
      sampling[i] += analogRead(fingerPins[i]);
	  // Average current and previous reading
      sampling[i] = sampling[i] / 2.0; 

      // Rescale to range 500-2500, where 500=clenched, 2500=hand open
      data[i] = float_map(sampling[i], min_list[i], max_list[i], 2500, 500);

      // Constrain to limits of range
      data[i] = data[i] > 2500 ? 2500 : data[i];
      data[i] = data[i] < 500 ? 500 : data[i];
    }
    //timer_sampling = millis() + 10;
  }

  // Calibration
  if (turn_on && timer_init < millis()) {
    switch (init_step) {
	  // Steps 0-2: Flash LEDs
      case 0:
        for(int i=0; i<5; i++) { digitalWrite(ledPins[i], LOW); }
        timer_init = millis() + 20;
        init_step++;
        break;
      case 1:
        for(int i=0; i<5; i++) { digitalWrite(ledPins[i], HIGH); }
        timer_init = millis() + 200;
        init_step++;
        break;
      case 2:
        for(int i=0; i<5; i++) { digitalWrite(ledPins[i], LOW); }
        timer_init = millis() + 50;
        init_step++;
        break;
      // Step 3: Read "max" values, when hand is clenched
      case 3:
        for(int i=0; i<5; i++) { digitalWrite(ledPins[i], HIGH); }
        timer_init = millis() + 500;
        init_step++;
        Serial.print("max_list:");
        for (int i = 0; i < 5; i++) {
          max_list[i] = sampling[i];
          Serial.print(max_list[i]);
          Serial.print("-");
        }
        Serial.println();
        break;
      case 4:
        init_step++;
        break;
      // Step 5: Wait until index finger has pointed up more than 50 before proceeding
      case 5:
        if ((max_list[1] - sampling[1]) > 50) {
          init_step++;
          for(int i=0; i<5; i++) { digitalWrite(ledPins[i], LOW); }
          timer_init = millis() + 2000;
        }
        break;
	  // Steps 6-7: Flash LEDs
      case 6:
        for(int i=0; i<5; i++) { digitalWrite(ledPins[i], HIGH); }
        timer_init = millis() + 200;
        init_step++;
        break;
      case 7:
        for(int i=0; i<5; i++) { digitalWrite(ledPins[i], LOW); }
        timer_init = millis() + 50;
        init_step++;
        break;
      // Step 8: Now take "min" values, when hand is open outstretched
      case 8:
        for(int i=0; i<5; i++) { digitalWrite(ledPins[i], HIGH); }
        timer_init = millis() + 500;
        init_step++;
        Serial.print("min_list:");
        for (int i = 0; i <5; i++) {
          min_list[i] = sampling[i];
          Serial.print(min_list[i]);
          Serial.print("-");
        }
        Serial.println();
        lsc.runActionGroup(0, 1);
        turn_on = false;
        break;
      default:
        break;
    }
  }
}


void update_mpu6050() {
  static uint32_t timer_u;
  if (timer_u < millis()) {
    timer_u = millis() + 20;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	// Filter the raw values
    ax0 = ((float)(ax)) * 0.3 + ax0 * 0.7;
    ay0 = ((float)(ay)) * 0.3 + ay0 * 0.7;
    az0 = ((float)(az)) * 0.3 + az0 * 0.7;
	// Corrected and converted to multiples of gravitational acceleration
    ax1 = (ax0 - ax_offset) /  8192.0;
    ay1 = (ay0 - ay_offset) /  8192.0;
    az1 = (az0 - az_offset) /  8192.0;
	// Filter the raw angular velocity values
    gx0 = ((float)(gx)) * 0.3 + gx0 * 0.7;
    gy0 = ((float)(gy)) * 0.3 + gy0 * 0.7;
    gz0 = ((float)(gz)) * 0.3 + gz0 * 0.7;
	// Corrected angular velocity
    gx1 = (gx0 - gx_offset);
    gy1 = (gy0 - gy_offset);
    gz1 = (gz0 - gz_offset);

    // Calculate x-axis inclination
    radianX = atan2(ay1, az1);
    radianX = radianX * 180.0 / 3.1415926;
    float radian_temp = (float)(gx1) / 16.4 * 0.02;
    radianX_last = 0.8 * (radianX_last + radian_temp) + (-radianX) * 0.2;

    // Calculate y-axis inclination
    radianY = atan2(ax1, az1);
    radianY = radianY * 180.0 / 3.1415926;
    radian_temp = (float)(gy1) / 16.4 * 0.01;
    radianY_last = 0.8 * (radianY_last + radian_temp) + (-radianY) * 0.2;
  }
}

// Debug
void print_data() {
  static uint32_t timer_p;
  static uint32_t timer_printlog;
  if(timer_p < millis()) {
    Serial.print("ax:"); Serial.print(ax1);
    Serial.print(", ay:"); Serial.print(ay1);
    Serial.print(", az:"); Serial.print(az1);
    Serial.print(", gx:"); Serial.print(gx1);
    Serial.print(", gy:"); Serial.print(gy1);
    Serial.print(", gz:"); Serial.print(gz1);
    Serial.print(", GX:"); Serial.print(radianX_last);
    Serial.print(", GY:"); Serial.println(radianY_last);
    timer_p = millis() + printlogInterval;
  }
  if (timer_printlog <= millis()) {
    for(int i=0; i<5; i++) {
      Serial.print(data[i]);
      Serial.print(" ");
    }
    timer_printlog = millis() + printlogInterval;
    Serial.println();
  }
}

// Hexapod
void run() {
  static uint32_t timer;
  static int act;
  static int last_act;
  static uint8_t count = 0;
  if (timer > millis())
    return;
  timer = millis() + 80;
  // Palm tilted 35-90 degrees right, middle finger straight, ring finger bent  
  if (radianY_last < -35 && radianY_last > -90 && data[3] < 1200  && data[2] > 2000) {
    act = TURN_RIGHT;
  }
  // Palm tilted 35-90 degrees left, middle finger straight, ring finger bent
  if (radianY_last < 90 && radianY_last > 35 && data[3] < 1200 && data[2] > 2000) {
    act = TURN_LEFT;
  }
  // Fist, palm pointing down
  if ((radianY_last < 15 && radianY_last > -15) && data[2] < 600) {
    act = STOP;
  }
  // Flat open hand, palm down
  if ((radianY_last < 15 &&  radianY_last > -15 ) && data[2] > 2100 && data[3] > 2100) {
    act = GO_FORWARD;
  }
  // Palm up, middle finger bent, little finger straight (Spiderman web-slinging pose)
  if ((radianY_last < -130 ||  radianY_last > 130 ) && data[2] < 1200 && data[4] > 2000) {
    act = GO_BACK;
  }
  // Open palm pointing up (i.e. "stop")
  if ((radianY_last < -130 ||  radianY_last > 130 ) && data[2] > 2000) {
    act = STOP;
  }
  if (act != last_act) {
    last_act = act;
    if (act == STOP) {
		if (count != 1) {
		  count = 1;
		  lsc.stopActionGroup();  //停止当前动作组
		  lsc.runActionGroup(0, 1);  //运行指定动作组
		  //   Serial.println("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS");
		  return;
		}
    }
    if (act == GO_FORWARD) {
		if (count != 2) {
		  count = 2;
		  lsc.stopActionGroup();
		  lsc.runActionGroup(1, 0);
		  //   Serial.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
		  return;
		}
    }
    if (act == GO_BACK) {
		if (count != 3) {
		  count = 3;
		  lsc.stopActionGroup();
		  lsc.runActionGroup(2, 0);
		  //   Serial.println("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB");
		  return;
		}
    }
    if (act == TURN_LEFT) {
		if (count != 4) {
		  count = 4;
		  lsc.stopActionGroup();
		  lsc.runActionGroup(3, 0);
		  return;
		}
    }
    if (act == TURN_RIGHT) {
		if (count != 5) {
		  count = 5;
		  lsc.stopActionGroup();
		  lsc.runActionGroup(4, 0);
		  return;
		}
    }
  }
}

// Robot Hand
void run1(int mode) {
  for (int i=0; i<5; i++) {
    ServoPwmSet[i] = data[i]; 
    ServoPwmSet[i] = float_map(ServoPwmSet[i], 500, 2500, 1100, 1950);
  }
  int pos = 0;
  if(mode == 0)
    pos = ServoPwmSet[4];
  else
    pos = 2750 - ServoPwmSet[4];
  lsc.moveServos(5, 30, 1, 3050 - ServoPwmSet[0], 2, ServoPwmSet[1], 3, ServoPwmSet[2], 4, ServoPwmSet[3], 5, pos);
}

// Send car data
void car_control(byte motor1, byte motor2) {
  byte buf[6];
  buf[0] = buf[1] = 0x55;
  buf[2] = 0x04;
  buf[3] = 0x32;
  buf[4] = (byte)motor1;
  buf[5] = (byte)motor2;
  Bth.write(buf, 6);
}

// Car
void run2() {
  static uint32_t timer;
  if (timer > millis())
    return;
  timer = millis() + 100;
  
  if (data[2] < 600 && (radianY_last < -30 && radianY_last > -90)) {
    car_control(100, -100);
  }
  else if (data[2] < 600  && (radianY_last > 30 && radianY_last < 90)) {
   car_control(-100, 100); 
  }
  else if (data[2] < 600 && abs(radianY_last) < 30 ) {
    car_control(100, 100);
  }
  else if (data[2] < 600 && (radianY_last < -130 ||  radianY_last > 130 )) {
   car_control(-100, -100); 
  }
  else
    car_control(0, 0); 
}

// Robot Arm
void run3() {
  static uint32_t timer;
  if (timer > millis())
    return;
  timer = millis() + 50;
  static float RadianY_Las = 0;
  // Make a fist and rotate it to control the rotation of the No. 6 servo
  if (data[1] < 1200 && data[2] < 1000 && data[3] < 1000) {
	if (radianY_last < 90 && radianY_last > -90) {
		lsc.moveServo(6, 1500 + radianY_last*10, 50);
		delay(50);
	}
  }
  // Five fingers open and rotate to control the rotation of No. 5 servo
  else if ( data[0] > 1400 && data[1] > 1400 && data[2] > 1400 && data[3] > 1400) {
	if (radianY_last < 90 && radianY_last > -90) {
		lsc.moveServo(5, 1500 + radianY_last*10, 50);
		delay(50);
	}
  }
  // Straighten the index finger and rotate to control the No. 1 servo
  else if (data[1] > 1400 && data[2] < 1000 && data[3] < 1000 ) {
	if (radianY_last < 90 && radianY_last > -90) {
		lsc.moveServo(1, 1500 + radianY_last*10, 50);
		delay(50);
	}
  }
  // Straighten the index and middle fingers to control the No. 2 servo
  else if (data[1] > 1400 && data[2] > 1400 && data[3] < 1000 ) {
	if (radianY_last < 90 && radianY_last > -90) {
		lsc.moveServo(2, 1500 + radianY_last*10, 50);
		delay(50);
	}
  }
  // The middle finger, the ring finger, and the little finger are straightened to control the No. 3 servo
  else if (data[1] < 1400 && data[2] > 1200 && data[3] > 1000 ) {
	if (radianY_last < 90 && radianY_last > -90) {
		lsc.moveServo(3, 1500 + radianY_last*10, 50);
		delay(50);
	}
  }
  // The index finger, the middle finger, the ring finger, and the little finger are straightened to control the No. 4 servo
  else if (data[0] < 1400 && data[1] > 1400 && data[2] > 1400 && data[3] > 1400) {
	if (radianY_last < 90 && radianY_last > -90) {
		lsc.moveServo(4, 1500 + radianY_last*10, 50);
		delay(50);
	}
  }
}

void loop() {
	
  // Update finger inputs
  finger();
  // Update gyro sensor input
  update_mpu6050();

  // Only execute run function after startup calibration complete
  if (turn_on == false) {
    // Key has been released
    if(key_state == true && digitalRead(funcButtonPin) == true) {
      // Wait 30ms to debounce before resetting state
      delay(30);
      if(digitalRead(funcButtonPin) == true)
        key_state = false;
    }
    // Key has been pressed
    if (digitalRead(funcButtonPin) == false && key_state == false) {
      // Debounce
      delay(30);
      if (digitalRead(funcButtonPin) == false) {
        key_state = true;
        if(mode == 5) { mode = 0; }
        else { mode++; }
        Serial.println(mode);
        // Light up LEDs to indicate the new mode
        for(int i=0; i<5; i++) { 
          digitalWrite(ledPins[i], i<mode ? LOW : HIGH); 
        }
      }
    }

    // Switch operation mode
    // Hand
    if (mode == 0 || mode == 1)  {
      static float RadianY_Las = 0;
      run1(mode);
      if (radianY_last < 90 && radianY_last > -90) {
        if(abs(radianY_last - RadianY_Las) > 1)  {
          lsc.moveServo(6, 1500 + radianY_last*10, 50);
          RadianY_Las = radianY_last;
          delay(50);
        }
      }
    }
    // Car
    if (mode == 2)
      run2();
    // Robot arm
    if (mode == 3)
      run3();
    // Hexapod
    if(mode == 4)
      run(); 
    // Blank
    if(mode == 5)
      ;
      
  }

  // Print debug output
  print_data();
}
