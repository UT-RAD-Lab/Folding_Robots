/*
   Description: This program is used to control the toroid robot with utilizing 2 Adafruit PCA9685 servo drivers and a NRF24L01 radio module

        Wiring: The required components are 3x 20kg servos, 18x MG995 servos, a NRF24L01 radio module, a MPU6050 IMU, and 2x PCA9685 servo drivers
        The components are all connected with the FREDWARD v1 printed circuit board.
        Servos are connected to the drivers according to the pinout[] array.

    Created by: Josh Stanley

    Date Created: 04/06/2022

    Date Modified: 07/14/2022

    Notes: This version upgrades to the ESP8266 control board and adds over the air uploads and the ability to disable the servo drivers
*/

//Libraries
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Ramp.h>
#include <ArduinoOTA.h>

Adafruit_PWMServoDriver driver1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver driver2 = Adafruit_PWMServoDriver(0x41);
RF24 radio(2, 0);
rampInt angle[7][3];

//Variables
bool orientation = 1;
bool gait = 0;
byte cnt = 0;
int yDir = 0;
int xDir = 0;
int duration = 0;
bool transition = 0;
bool enabled = 1;
byte data[2][10] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
byte but[][4] = {{2, 3, 6, 7}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
unsigned long millisPrev = 0;

//Constants
const byte chan[][6] = {"00007", "00008"};
const char* ssid = "*******";          //Update for new wifi networks
const char* password = "*******";

const byte pinout[][3] = {
  {0, 1, 2},          //Servo Driver 1
  {4, 5, 6},
  {8, 9, 10},
  {12, 13, 14},
  {0, 1, 2},          //Servo Driver 2
  {4, 5, 6},
  {8, 9, 10}
};

const int straight[][3] = {    //Array holding the true servo set values to obtain a 180 degree angle
  {198, 194, 180},
  {116, 464, 134},
  {126, 429, 140},
  {153, 460, 148},
  {135, 461, 128},
  {137, 424, 127},
  {110, 435, 130}
};

const int right[][3] = {    //Array holding the true servo set values to obtain a 90 degree angle
  {328, 327, 310},
  {325, 253, 343},
  {330, 219, 342},
  {360, 243, 355},
  {325, 263, 317},
  {326, 241, 327},
  {304, 248, 320},
};

const byte h_shape[][3] = {
  {30, 165, 165},     //0: Pancake
  {60, 150, 150},     //1: High Roll
  {150, 60, 150},     //2: High Roll
  {150, 150, 60},     //3: High Roll
  {90, 180, 90},      //4: Low Roll
  {45, 180, 135},     //5: Low Roll
  {90, 90, 180},      //6: Low Roll
  {135, 45, 180},     //7: Low Roll
  {180, 90, 90},      //8: Low Roll
  {180, 135, 45}      //9: Low Roll
};

const byte e_shape[][3] = {
  {180, 160, 180},    //0: Transition
  {180, 20, 48},      //1: Tilt Left
  {170, 20, 20},      //2: Flat
  {144, 48, 20},      //3: Tilt Right
  {150, 60, 60},      //4: High Roll
  {127, 180, 180},    //5: Extended
  {162, 126, 126},    //6: Midpoint
  {180, 95, 95},      //7: Retracted
  {120, 150, 150},    //8: Back Step
  {56, 180, 180}      //9: Transition
};

const byte seq[2][3][3][6][7] = {    //[orientation][gait][direction][cnt][module]
  { { {{1, 1, 1, 1, 1, 1, 1}, {2, 1, 1, 1, 1, 1, 1}, {3, 1, 1, 1, 1, 1, 1}, {1, 1, 1, 1, 1, 1, 1}, {2, 1, 1, 1, 1, 1, 1}, {3, 1, 1, 1, 1, 1, 1}},
      {{1, 2, 2, 2, 2, 2, 2}, {2, 2, 2, 2, 2, 2, 2}, {3, 2, 2, 2, 2, 2, 2}, {1, 2, 2, 2, 2, 2, 2}, {2, 2, 2, 2, 2, 2, 2}, {3, 2, 2, 2, 2, 2, 2}},    //High Roll
      {{1, 3, 3, 3, 3, 3, 3}, {2, 3, 3, 3, 3, 3, 3}, {3, 3, 3, 3, 3, 3, 3}, {1, 3, 3, 3, 3, 3, 3}, {2, 3, 3, 3, 3, 3, 3}, {3, 3, 3, 3, 3, 3, 3}}
    },
    { {{4, 1, 1, 1, 1, 1, 1}, {5, 1, 1, 1, 1, 1, 1}, {6, 1, 1, 1, 1, 1, 1}, {7, 1, 1, 1, 1, 1, 1}, {8, 1, 1, 1, 1, 1, 1}, {9, 1, 1, 1, 1, 1, 1}},
      {{4, 2, 2, 4, 2, 2, 4}, {5, 4, 2, 2, 4, 2, 2}, {6, 4, 2, 2, 4, 2, 2}, {7, 2, 4, 2, 2, 4, 2}, {8, 2, 4, 2, 2, 4, 2}, {9, 2, 2, 4, 2, 2, 4}},    //Low Roll
      {{4, 3, 3, 3, 3, 3, 3}, {5, 3, 3, 3, 3, 3, 3}, {6, 3, 3, 3, 3, 3, 3}, {7, 3, 3, 3, 3, 3, 3}, {8, 3, 3, 3, 3, 3, 3}, {9, 3, 3, 3, 3, 3, 3}}
    },
    { {{0, 2, 2, 2, 2, 2, 2}, {0, 9, 9, 9, 9, 9, 9}, {0, 5, 5, 5, 5, 5, 5}, {0, 7, 7, 7, 7, 7, 7}}    //Transition to Walking
    }
  },
  { { {{2, 7, 7, 5, 7, 7, 5}, {2, 6, 8, 6, 8, 6, 8}, {2, 5, 5, 7, 5, 5, 7}, {2, 8, 6, 8, 6, 8, 6}},
      {{0, 5, 7, 5, 5, 7, 5}, {0, 6, 8, 6, 8, 6, 8}, {0, 7, 5, 7, 7, 5, 7}, {0, 8, 6, 8, 6, 8, 6}},   //Crab Walk
      {{3, 5, 7, 7, 5, 7, 7}, {3, 6, 8, 6, 8, 6, 8}, {3, 7, 5, 5, 7, 5, 5}, {3, 8, 6, 8, 6, 8, 6}}
    },
    { {{8, 7, 7, 5, 5, 7, 7}, {8, 6, 8, 6, 6, 8, 6}, {8, 5, 7, 7, 7, 7, 5}, {8, 8, 7, 8, 8, 7, 8}},
      {{8, 7, 5, 7, 7, 7, 7}, {8, 8, 6, 8, 8, 6, 8}, {8, 7, 7, 7, 7, 5, 7}, {8, 7, 8, 7, 7, 8, 7}},   //Gorilla Walk
      {{8, 5, 7, 7, 7, 7, 5}, {8, 6, 8, 6, 6, 8, 6}, {8, 7, 7, 5, 5, 7, 7}, {8, 8, 7, 8, 8, 7, 8}}
    },
    { {{0, 2, 2, 2, 2, 2, 2}, {0, 0, 0, 0, 1, 1, 1}, {0, 2, 2, 2, 2, 2, 2}, {1, 2, 2, 2, 2, 2, 2}}    //Transition to Rolling
    }
  }
};

void setup() {
  Serial.begin(9600);
  Serial.println("Serial Communication Initialized");

  radio.begin();
  radio.openReadingPipe(1, chan[0]);
  radio.openWritingPipe(chan[1]);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("Radio Communication Initialized");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  ArduinoOTA.setHostname("FREDWARD");
  ArduinoOTA.setPassword("admin");
  ArduinoOTA.begin();
  Serial.println("Over the Air Updates Enabled");

  driver1.begin();
  driver1.setOscillatorFrequency(27000000);
  driver1.setPWMFreq(50);
  driver2.begin();
  driver2.setOscillatorFrequency(27000000);
  driver2.setPWMFreq(50);
  pinMode(16, OUTPUT);
  digitalWrite(16, 0);
  Serial.println("Servo Drivers Intialized");

  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.println("IMU Initialized");

  for (int j = 0; j < 7; j++) {
    for (int i = 0; i < 3; i++) {
      angle[j][i].go(map(120, 90, 180, right[j][i], straight[j][i]));
      if (j < 4) driver1.setPWM(pinout[j][i], 0, angle[j][i].update());
      else driver2.setPWM(pinout[j][i], 0, angle[j][i].update());
    }
    delay(100);
  }
  Serial.println("Servos Set");

  delay(1000);
  Serial.println("Running");
}

void loop() {
  if (radio.available()) radio.read(&data[0], sizeof(data[0]));
  else {
    ArduinoOTA.handle();
    debounce();
    //measure();

    if (data[0][1] == 0) yDir = 0;
    else yDir = map(data[0][1], 1, 255, -1, 1);

    if (data[0][4] == 0) xDir = 0;
    else xDir = map(data[0][4], 1, 255, -1, 1);

    duration = map(data[0][9], 0, 255, 1000, 250);

    if (but[2][0]) {    //Transition
      transition = 1;
      gait = 0;
      cnt = 0;
    }
    else if (but[2][1] || but[2][3]) {    //Change Gait
      gait = !gait;
      cnt = 0;
    }
    if (but[2][2]) {    //Enable or Disable
      enabled = !enabled;
      cnt = 0;
      if (enabled) digitalWrite(16, 0);
    }

    if ((millis() - millisPrev) > angle[0][0].getDuration() && ((yDir != 0) || transition || (enabled == 0))) {
      if (enabled) {
        if (transition) {    //Transitioning
          for (int j = 0; j < 7; j++) {
            for (int i = 0; i < 3; i++) {
              if (j == 0) angle[j][i].go(map(h_shape[seq[orientation][2][0][cnt][j]][i], 90, 180, right[j][i], straight[j][i]), 1000);
              else angle[j][i].go(map(e_shape[seq[orientation][2][0][cnt][j]][i], 90, 180, right[j][i], straight[j][i]), 1000);
            }
          }
          if (cnt == 3) {
            orientation = !orientation;
            transition = 0;
          }
          else cnt++;
        }
        else {
          if (orientation) {    //Walking
            if (yDir < 0) {
              xDir = -xDir;
              if (cnt == 0) cnt = 3;
              else cnt--;
            }
            else {
              if (cnt == 3) cnt = 0;
              else cnt++;
            }
          }
          else {    //Rolling
            if (yDir < 0) {
              if (cnt == 0) cnt = 5;
              else cnt--;
            }
            else {
              if (cnt == 5) cnt = 0;
              else cnt++;
            }
          }
          for (int j = 0; j < 7; j++) {
            for (int i = 0; i < 3; i++) {
              if (j == 0) angle[j][i].go(map(h_shape[seq[orientation][gait][1 + xDir][cnt][j]][i], 90, 180, right[j][i], straight[j][i]), duration);
              else angle[j][i].go(map(e_shape[seq[orientation][gait][1 + xDir][cnt][j]][i], 90, 180, right[j][i], straight[j][i]), duration);
            }
          }
        }
      }
      else {    //Disabling
        if (cnt == 0) {
          for (int j = 0; j < 7; j++) {
            for (int i = 0; i < 3; i++) {
              if (j == 0) angle[j][i].go(map(h_shape[0][i], 90, 180, right[j][i], straight[j][i]), 1000);
              else angle[j][i].go(map(e_shape[2][i], 90, 180, right[j][i], straight[j][i]), 1000);
            }
          }
          cnt++;
        }
        else digitalWrite(16, 1);
      }
      millisPrev = millis();
    }
    for (int j = 0; j < 7; j++) {
      for (int i = 0; i < 3; i++) {
        if (j < 4) driver1.setPWM(pinout[j][i], 0, angle[j][i].update());
        else driver2.setPWM(pinout[j][i], 0, angle[j][i].update());
      }
    }
  }
}

void measure() {    //Function obtaining, normalizing, and sending IMU data    REMOVE FUNCTION?
  int16_t input[7];

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  for (int i = 0; i < 7; i++) {
    input[i] = Wire.read() << 8 | Wire.read();
  }
  
//  radio.stopListening();
//  radio.write(&data[1], sizeof(data[1]));
//  delay(5);
//  radio.startListening();
}

void debounce() {    //Function to return momentary signal when a remote button is pressed
  for (int i = 0; i < 4; i++) {
    if (data[0][but[0][i]]) {
      if (!but[1][i]) but[2][i] = 1;
      else but[2][i] = 0;
      but[1][i] = 1;
    }
    else {
      but[1][i] = 0;
      but[2][i] = 0;
    }
  }
}
