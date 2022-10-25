/*
   Description: This program is used to control a remote control hexagonal folding robot with IMU feedback

        Wiring: The required components are 3x MG995 servos, a NRF24L01 radio module, and a MPU6050 IMU
        The servos are connected with brown wire to GND, red wire to Vin, and orange wire to D3, D5, and D6
        The NRF24L01 is connected 3.3V to 3v3, GND to GND, CSN to D10, MOSI to D11, CE to D9, SCK to D13, MISO to D12
        The MPU6050 is wired VCC to 5V, GND to GND, SCL to A5, and SDA to A4

    Created by: Josh Stanley

    Date Created: 04/07/2022

    Date Modified: 07/14/2022
*/


//Libraries
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Ramp.h>

RF24 radio(9, 10);
Servo servo[3];
ramp angle[3];

int currentSens = A7;

//Variables
byte gait = 0;
byte cnt = 0;
int yDir = 0;
int xDir = 0;
int duration = 0;
float orientation = 0;
byte data[2][10] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
byte but[][4] = {{2, 3, 6, 7}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
unsigned long millisPrev = 0;

//Constants
const byte pinout[3] = {3, 5, 6};
const byte chan[][6] = {"00007", "00008"};

const byte straight[3] = {10, 8, 16}; const byte right[3] = {75, 76, 78};

const byte seq[4][7][3] = {
  {{150, 150, 60}, {60, 150, 150}, {150, 60, 150}, {150, 150, 60}, {60, 150, 150}, {150, 60, 150}, {150, 150, 60}},
  {{163, 163, 94}, {60, 120, 120}, {163, 94, 163}, {120, 120, 60}, {94, 163, 163}, {120, 60, 120}, {120, 120, 120}},
  {{180, 90, 90}, {180, 135, 45}, {90, 180, 90}, {45, 180, 135}, {90, 90, 180}, {135, 45, 180}},
  {{180, 90, 90}, {150, 150, 60}, {90, 180, 90}, {60, 150, 150}, {90, 90, 180}, {150, 60, 150}}
};

int offset[][3] {{4, 4, -8}, { -8, 4, 4}, {4, -8, 4}, {4, 4, -8}, { -8, 4, 4}, {4, -8, 4}};

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


  for (int i = 0; i < 3; i++) {
    servo[i].attach(pinout[i]);
    angle[i].go(right[i] - (right[i] - straight[i]) / 3);
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
    if (data[0][4] == 0) xDir = 0;
    else xDir = map(data[0][4], 1, 255, -1, 1);

    duration = map(data[0][9], 0, 255, 400, 100);

    if (but[2][1]) {
      if (gait == 3) gait = 0;
      else gait++;
      cnt = 0;
      for (int i = 0; i < 3; i++) {
        angle[i].go(map(seq[gait][cnt][i], 90, 180, right[i], straight[i]), duration);
      }
    }
    else if (but[2][3]) {
      if (gait == 0) gait = 3;
      else gait--;
      cnt = 0;
      for (int i = 0; i < 3; i++) {
        angle[i].go(map(seq[gait][cnt][i], 90, 180, right[i], straight[i]), duration);
      }
    }

    if ((millis() - millisPrev) > angle[0].getDuration()) {
      if (yDir != 0) {
        if (yDir > 0) {
          if (cnt == 5) cnt = 0;
          else cnt++;
        }
        else {
          if (cnt == 0) cnt = 5;
          else cnt--;
        }
        for (int i = 0; i < 3; i++) {
          angle[i].go(map(seq[gait][cnt][i], 90, 180, right[i], straight[i]), duration);
        }
        millisPrev = millis();
      }
      else if (xDir != 0 && gait == 0) {
        if(xDir > 0){
          if (cnt == 1 || cnt == 3) cnt++;
          if (cnt == 5) cnt = 0;
        }
        else{
          if (cnt == 0 || cnt == 2 || cnt == 4) cnt++;
        }
        
        for (int i = 0; i < 3; i++) {
          angle[i].go(map((seq[0][cnt][i] + offset[cnt][i]), 90, 180, right[i], straight[i]), 11);
          offset[cnt][i] = -offset[cnt][i];
        }
        millisPrev = millis();
      }
    }
    for (int i = 0; i < 3; i++) {
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

  double current = double(analogRead(currentSens) - 505) / 36;
  orientation = atan2(input[0] - 100, input[2] - 100) * 100;
  orientation = map(orientation, -314, 314, 0, 359);

  data[1][0] = current * 100;
  data[1][1] = map(input[0], -32768, 32768, 0, 200);
  data[1][2] = map(input[2], -32768, 32768, 0, 200);
  data[1][3] = map(-input[5], -32768, 32768, 0, 200);

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
