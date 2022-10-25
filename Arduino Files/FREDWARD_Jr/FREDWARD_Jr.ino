/*
   Description: This program is used to control a remote control toroid folding robot

        Wiring: The required components are a 20kg servo, 8x MG995 servos, a NRF24L01 radio module, a MPU6050 IMU, and a PCA9685 servo driver
        The components are all connected with the FREDWARD v1 printed circuit board.
        Servos are connected to the drivers according to the pinout[] array.

    Created by: Josh Stanley

    Date Created: 04/15/2022

    Date Modified: 07/14/2022

    Notes: This version upgrades to the ESP8266 control board and adds over the air uploads
*/

//Libraries
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Ramp.h>
#include <ArduinoOTA.h>

Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver(0x40);
RF24 radio(2, 0);
rampInt angle[5][2];

//Variables
bool orientation = 1;
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
const char* ssid = "*******";        //Update for new wifi networks
const char* password = "*******";

const int pinout[][2] = {
  {0, 1}, {4, 5}, {6, 7}, {8, 9}, {10, 11}
};
const int straight[][2] = {
  {190, 193}, {122, 448}, {120, 427}, {111, 424}, {116, 420}
};
const int right[][2] = {
  {320, 323}, {317, 244}, {314, 226}, {302, 224}, {312, 226}
};
const byte h_shape[] =  {
  90,         //0: Square
  30,         //1: Acute
  150         //2: Obtuse
};
const byte e_shape[][3] =  {
  {17, 180},    //0: Transition to Roll
  {180, 124},   //1: Tilt Left
  {150, 150},   //2: Flat
  {124, 180},   //3: Tilt Right
  {100, 100},   //4: Pointed
  {40, 60},     //5: Extended
  {33, 61},     //6: Midpoint
  {17, 62},     //7: Retracted
  {17, 84},     //8: Back Step
  {30, 90},     //9: High Step
  {180, 90}     //10: Transition to Walk
};

const byte seq[2][6][4][5] = {   //[orientation][direction][cnt][module]
  { {{1, 1, 1, 1, 1}, {1, 1, 1, 1, 1}, {1, 1, 1, 1, 1}, {1, 1, 1, 1, 1}},
    {{1, 1, 1, 1, 1}, {0, 1, 4, 1, 4}, {2, 1, 1, 1, 1}, {0, 4, 1, 4, 1}},
    {{1, 2, 2, 2, 2}, {0, 2, 4, 2, 4}, {2, 2, 2, 2, 2}, {0, 4, 2, 4, 2}},
    {{1, 3, 3, 3, 3}, {0, 3, 4, 3, 4}, {2, 3, 3, 3, 3}, {0, 4, 3, 4, 3}},
    {{1, 3, 3, 3, 3}, {1, 3, 3, 3, 3}, {1, 3, 3, 3, 3}, {1, 3, 3, 3, 3}},
    {{0, 2, 2, 2, 2}, {0, 1, 1, 1, 1}, {0, 10, 10, 10, 10}, {0, 8, 8, 8, 8}}
  },
  { {{2, 5, 5, 7, 7}, {2, 6, 8, 6, 9}, {2, 7, 7, 5, 5}, {2, 9, 6, 8, 6}},
    {{1, 5, 5, 5, 5}, {1, 8, 6, 8, 6}, {1, 7, 7, 7, 7}, {1, 6, 8, 6, 8}},
    {{1, 5, 7, 7, 5}, {1, 6, 9, 6, 8}, {1, 7, 5, 5, 7}, {1, 9, 6, 8, 6}},
    {{1, 5, 5, 5, 5}, {1, 6, 8, 6, 8}, {1, 7, 7, 7, 7}, {1, 8, 6, 8, 6}},
    {{2, 5, 5, 7, 7}, {2, 8, 6, 9, 6}, {2, 7, 7, 5, 5}, {2, 6, 9, 6, 8}},
    {{2, 1, 0, 0, 1}, {2, 0, 0, 0, 0}, {1, 0, 0, 0, 0}, {1, 2, 2, 2, 2}}
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
  ArduinoOTA.setHostname("FREDWARD Jr");
  ArduinoOTA.setPassword("admin");
  ArduinoOTA.begin();
  Serial.println("Over the Air Updates Enabled");

  driver.begin();
  driver.setOscillatorFrequency(27000000);
  driver.setPWMFreq(50);
  Serial.println("Servo Driver Initialized");

  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.println("IMU Initialized");

  for (int j = 0; j < 5; j++) {
    for (int i = 0; i < 2; i++) {
      if (j == 0) angle[j][i].go(map(h_shape[1], 90, 180, right[j][i], straight[j][i]));
      else angle[j][i].go(map(e_shape[4][i], 90, 180, right[j][i], straight[j][i]));
      driver.setPWM(pinout[j][i], 0, angle[j][i].update());
    }
    delay(100);
  }
  Serial.println("Servos Set");
  pinMode(16, OUTPUT);
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

    duration = map(data[0][9], 0, 255, 2000 - 1500 * orientation, 500 - 350 * orientation);

    if (but[2][0]) {
      transition = 1;
      cnt = 0;
    }
    if (but[2][2]){
      enabled = !enabled;
      digitalWrite(16, !enabled);
    }
    
    if ((millis() - millisPrev) > angle[0][0].getDuration() && enabled && ((yDir != 0 || xDir != 0) || transition)) {
      if (transition) {
        for (int j = 0; j < 5; j++) {
          for (int i = 0; i < 2; i++) {
            if (j == 0) angle[j][i].go(map(h_shape[seq[orientation][5][cnt][j]], 90, 180, right[j][i], straight[j][i]), 1000);
            else angle[j][i].go(map(e_shape[seq[orientation][5][cnt][j]][i], 90, 180, right[j][i], straight[j][i]), 1000);
          }
        }
        if (cnt == 3){
          orientation = !orientation;
          transition = 0;
        }
        else cnt++;
      }
      else {
        if (yDir < 0) {
          if (cnt == 0) cnt = 3;
          else cnt--;
        }
        else {
          if (cnt == 3) cnt = 0;
          else cnt++;
        }
        for (int j = 0; j < 5; j++) {
          for (int i = 0; i < 2; i++) {
            if (j == 0) angle[j][i].go(map(h_shape[seq[orientation][2 + (2 - abs(yDir))*xDir][cnt][j]], 90, 180, right[j][i], straight[j][i]), duration);
            else angle[j][i].go(map(e_shape[seq[orientation][2 + (2 - abs(yDir))*xDir][cnt][j]][i], 90, 180, right[j][i], straight[j][i]), duration);
          }
        }
      }
      millisPrev = millis();
    }
    for (int j = 0; j < 5; j++) {
      for (int i = 0; i < 2; i++) {
        driver.setPWM(pinout[j][i], 0, angle[j][i].update());
      }
    }
  }
}

void measure() {
  int16_t input[7];    //x acc, y acc, z acc, temp, x gyro, y gyro, z gyro

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  for (int i = 0; i < 7; i++) {
    input[i] = Wire.read() << 8 | Wire.read();
  }
  //radio.stopListening();
  //radio.write(&data[1], sizeof(data[1]));
  //delay(5);
  //radio.startListening();
}

void debounce() {
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
