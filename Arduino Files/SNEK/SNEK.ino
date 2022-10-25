/*
   Description: This program is used to control a remote control snake robot with five articulated joints

        Wiring: The required components are 5x MG995 servos and a NRF24L01 radio module
        The servos are connected with brown wire to GND, red wire to Vin, and orange wire to D2, D3, D4, D5, and D6
        The NRF24L01 is connected 3.3V to 3v3, GND to GND, CSN to D10, MOSI to D11, CE to D9, SCK to D13, MISO to D12

    Created by: Josh Stanley

    Date Created: 07/14/2022

    Date Modified: 07/20/2022

    Notes: This version updated to the five module version of the robot
*/

//Libraries
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Ramp.h>

RF24 radio(6, 7);
Servo servo[4];
ramp angle[4];

//Variables
byte gait = 0;
byte cnt = 5;
int yDir = 0;
int duration = 0;
int mod = 0;
bool transition = 0;
byte data[2][10] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
byte but[][4] = {{2, 3, 6, 7}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
unsigned long millisPrev = 0;

//Constants
const byte chan[][6] = {"00007", "00008"};
const int right[2][4] = {{28, 24, 26, 26}, {152, 153, 152, 153}};
const int seq[4][6][4] = {
  {{90, 90, 180, 90}, {150, 60, 150, 150}, {180, 90, 90, 180}, {150, 150, 60, 150}, {90, 180, 90, 90}, {60, 150, 150, 60}},
  {{180, 270, 270, 180}, {180, 280, 260, 180}, {180, 300, 180, 180}, {180, 240, 240, 180}, {180, 180, 300, 180}, {180, 260, 280, 180}},
  {{150, 180, 180, 180}, {240, 150, 180, 180}, {150, 240, 150, 180}, {180, 150, 240, 150}, {180, 180, 150, 240}, {180, 180, 180, 150}},
  {{ -5, -5, 10, -5}, {10, -5, -5, 10}, { -5, 10, -5, -5}, { -5, -5, 10, -5}, {10, -5, -5, 10}, { -5, 10, -5, -5}}
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

  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.println("IMU Initialized");

  for (int i = 0; i < 4; i++) {
    servo[i].attach(i);
    angle[i].go((right[0][i] + right[1][i]) / 2);
    servo[i].write(angle[i].update());
  }
  Serial.println("Servos Attached");

  delay(250);
  Serial.println("Running");
}

void loop() {
  if (radio.available()) radio.read(&data[0], sizeof(data[0]));
  else {
    debounce();
    measure();

    if (data[0][1] == 0) yDir = 0;
    else yDir = map(data[0][1], 1, 255, -1, 1);

    if (gait == 1) duration = 400;
    else duration = map(data[0][9], 0, 255, 500, 100);

    mod = map(data[0][8], 0, 255, 1, 10);

    if (but[2][1]) {
      if (gait == 3) gait = 0;
      else gait++;
      cnt = 5;
      for (int i = 0; i < 4; i++) {
        angle[i].go((right[0][i] + right[1][i]) / 2, 1000);
      }
      millisPrev = millis();
    }
    else if (but[2][3]) {
      if (gait == 0) gait = 3;
      else gait--;
      cnt = 5;
      for (int i = 0; i < 4; i++) {
        angle[i].go((right[0][i] + right[1][i]) / 2, 1000);
      }
      millisPrev = millis();
    }
    else if (but[2][0]) {
      for (int i = 0; i < 4; i++) {
        angle[i].go(map(map(data[0][8], 0, 255, 160, 110), 90, 270, right[0][i], right[1][i]), 1000);
      }
      millisPrev = millis();
      transition = 1;
    }

    if ((millis() - millisPrev) > angle[0].getDuration()) {
      if (transition) {
        for (int i = 0; i < 4; i++) {
          angle[i].go(map(180, 90, 270, right[0][i], right[1][i]));
        }
        transition = 0;
        cnt = 5;
      }
      else if (yDir != 0) {
        if (yDir > 0) {
          if (cnt == 5) cnt = 0;
          else cnt++;
        }
        else {
          if (cnt == 0) cnt = 5;
          else cnt--;
        }
        if (gait < 3) {
          for (int i = 0; i < 4; i++) {
            angle[i].go(map(seq[gait][cnt][i], 90, 270, right[0][i], right[1][i]), duration);
          }
        }
        else {
          for (int i = 0; i < 4; i++) {
            angle[i].go(map(mod*seq[gait][cnt][i] + 180, 90, 270, right[0][i], right[1][i]), duration);
          }
        }
        millisPrev = millis();
      }
    }
    for (int i = 0; i < 4; i++) {
      servo[i].write(angle[i].update());
    }
  }
}

void measure() {
  int16_t input[7];

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  for (int i = 0; i < 7; i++) {
    input[i] = Wire.read() << 8 | Wire.read();
  }

  data[1][0] = map(input[0], -32768, 32768, 0, 200);
  data[1][1] = map(input[1], -32768, 32768, 0, 200);
  data[1][2] = map(input[2], -32768, 32768, 0, 200);

  //radio.stopListening();
  //radio.write(&data[1], sizeof(data[1]));
  //delay(5);
  //radio.startListening();
}

void debounce() {
  for (int i = 0; i < 4; i++) {
    if (data[0][but[0][i]]) {
      if (!but[1][i]) {
        but[2][i] = 1;
      }
      else but[2][i] = 0;
      but[1][i] = 1;
    }
    else {
      but[1][i] = 0;
      but[2][i] = 0;
    }
  }
}
