/*
   Description: This program is used to control a remote control cubic folding robot with IMU feedback

        Wiring: The required components are 4x MG995 servos, a NRF24L01 radio module, and a MPU6050 IMU
        The servo is connected with brown wire to GND, red wire to Vin, and orange wire to D3
        The NRF24L01 is connected 3.3V to 3v3, GND to GND, CSN to D10, MOSI to D11, CE to D9, SCK to D13, MISO to D12
        The MPU6050 is wired VCC to 5V, GND to GND, SCL to A5, and SDA to A4

    Created by: Josh Stanley

    Date Created: 03/10/2022

    Date Modified: 07/03/2022
*/

//Libraries
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10);
Servo servo[4];

//Variables
int xDir = 0;
int yDir = 0;
int xAngle = 90;
int yAngle = 90;
int duration = 0;
int face = 0;
float orientation[] = {0, 0};
int drive = 0;
byte data[2][10] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

//Constants
const byte chan[][6] = {"00007", "00008"};
const int acute[] = {43, 144, 143, 42};
const int obtuse[] = {135, 54, 46, 136};
const int seq[] = {45, 90, 135};

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
    servo[i].attach(i + 5);
    servo[i].write(map(90, 45, 135, acute[i], obtuse[i]));
  }
  Serial.println("Servo Attached");

  delay(1000);
  Serial.println("Running");
}

void loop() {
  if (radio.available()) radio.read(&data[0], sizeof(data[0]));
  else {
    if (data[0][6]) {                  //Press and hold left joystick for manual angle control
      if (data[0][0] == 0) xAngle = 90;
      else xAngle = map(data[0][0], 1, 255, 45, 135);
      if (data[0][1] == 0) yAngle = 90;
      else yAngle = map(data[0][1], 1, 255, 135, 45);

      servo[0].write(map(xAngle, 45, 135, acute[0], obtuse[0]));
      servo[1].write(map(xAngle, 45, 135, acute[1], obtuse[1]));
      servo[2].write(map(yAngle, 45, 135, acute[2], obtuse[2]));
      servo[3].write(map(yAngle, 45, 135, acute[3], obtuse[3]));
      delay(100);
    }
    else {
      if (data[0][0] == 0) xDir = 0;
      else xDir = map(data[0][0], 1, 255, -1, 1);
      if (data[0][1] == 0) yDir = 0;
      else yDir = map(data[0][1], 1, 255, -1, 1);

      //duration = map(data[0][9], 0, 255, 500, 250);   //Variable speed control
      duration = 325;     //Constant speed control

      if (yDir != 0 || xDir != 0) {
        if (abs(drive)) {
          drive = -drive;
          for (int i = 0; i < 2; i++) servo[i].write(map(seq[(drive * xDir) + 1], 45, 135, acute[i], obtuse[i]));
          for (int i = 2; i < 4; i++) servo[i].write(map(seq[(drive * yDir) + 1], 45, 135, acute[i], obtuse[i]));
        }
        else {
          for (int i = 0; i < 2; i++) servo[i].write(map(seq[xDir + 1], 45, 135, acute[i], obtuse[i]));
          for (int i = 2; i < 4; i++) servo[i].write(map(seq[yDir + 1], 45, 135, acute[i], obtuse[i]));
          drive = 1;
        }
        delay(duration);
      }
      else {
        if (abs(drive)) {
          for (int i = 0; i < 4; i++) {
            servo[i].write(map(90, 45, 135, acute[i], obtuse[i]));
          }
          delay(500);
          drive = 0;
        }
        else {
          measure();
          if (face == 1) {
            for (int i = 2; i < 4; i++) servo[i].write(acute[i]);
            delay(250);
            for (int i = 2; i < 4; i++) servo[i].write(map(90, 45, 135, acute[i], obtuse[i]));
            delay(750);
          }
          else if (face == 2) {
            for (int i = 0; i < 2; i++) servo[i].write(acute[i]);
            delay(250);
            for (int i = 0; i < 2; i++) servo[i].write(map(90, 45, 135, acute[i], obtuse[i]));
            delay(750);
          }
        }
      }
    }
  }

void measure() {
  int16_t input[7];    //x acc, y acc, z acc, temp, x gyro, y gyro, z gyro
  double xComp, yComp, zComp;

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  for (int i = 0; i < 7; i++) {
    input[i] = Wire.read() << 8 | Wire.read();
  }

  xComp = map(input[1], -32768, 32768, 200, -200);
  yComp = map(input[0], -32768, 32768, -200, 200);
  zComp = map(input[2], -32768, 32768, -200, 200);

  if (!yDir && !xDir) {
    if (abs(zComp) > 75) face = 0;
    else if (abs(yComp) > 75) face = 1;
    else if (abs(xComp) > 75) face = 2;
  }

  orientation[0] = atan2(yComp, -zComp) * 100;
  orientation[0] = map(orientation[0], -314, 314, 0, 359);
  orientation[1] = atan2(xComp, -zComp) * 100;
  orientation[1] = map(orientation[1], -314, 314, 0, 359);

  data[1][0] = orientation[0] / 2;
  data[1][1] = orientation[1] / 2;
  data[1][2] = xComp + 100;
  data[1][3] = yComp + 100;
  data[1][4] = zComp + 100;
  data[1][5] = map(-input[5], -32768, 32768, 0, 200);
  data[1][6] = map(-input[4], -32768, 32768, 0, 200);
  data[1][7] = map(-input[6], -32768, 32768, 0, 200);

  radio.stopListening();
  radio.write(&data[1], sizeof(data[1]));
  delay(5);
  radio.startListening();
}
