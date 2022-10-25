/*
   Description: This program is used to control a remote control hexagonal folding robot with IMU feedback

        Wiring: The required components are 3x MG995 servos, a NRF24L01 radio module, and a MPU6050 IMU
        The servos are connected with brown wire to GND, red wire to Vin, and orange wire to D3, D5, and D6
        The NRF24L01 is connected 3.3V to 3v3, GND to GND, CSN to D10, MOSI to D11, CE to D9, SCK to D13, MISO to D12
        The MPU6050 is wired VCC to 5V, GND to GND, SCL to A5, and SDA to A4

    Created by: Josh Stanley

    Date Created: 01/21/2022

    Date Modified: 01/25/2022
*/


//Libraries
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10);
Servo servo[3];

int currentSens = A7;

//Variables
byte mode = 1;
float cnt = 0;
int yDir = 0;
int duration = 0;
int amplitude = 60;
bool sending = 1;
float orientation = 0;
byte data[2][11] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
byte but[][6] = {{2, 3, 6, 7, 8, 9}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}};
unsigned long millisPrev = 0;

//Constants
const byte pinout[3] = {3, 5, 6};
const byte chan[][6] = {"00001", "00002"};
const byte straight[3] = {10, 11, 16};
const byte right[3] = {73, 77, 78};
const double phase[2][3] = {{5 * PI / 3, PI / 3, PI}, {PI, 7 * PI / 3, 5 * PI / 3}};

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
  }
  Serial.println("Servos Attached");

  for (int i = 0; i < 3; i++) {
    servo[i].write(map(fourier(i), 90, 180, right[i], straight[i]));
  }
  delay(250);
  Serial.println("Running");
}

void loop() {
  if (radio.available()) {
    radio.read(&data[0], sizeof(data[0]));
  }
  else {
    debounce();
    measure();

    //if (data[0][1] == 0) yDir = 0;
    //else yDir = map(data[0][1], 1, 255, -1, 1);
    duration = map(data[0][10], 0, 255, 8*2000, 8*600);

    if (but[2][1]) {
      mode = !mode;
      cnt = 0;
    }
    else if (but[2][0]){
      if (amplitude == 60) amplitude = 45;
      else amplitude = 60;
    }
    else if (but[2][2]) sending = !sending;
    else if (but[2][4]) {
      if (yDir != 0) yDir = 0;
      else yDir = 1;
    }
    else if (but[2][5]) {
      if (yDir != 0) yDir = 0;
      else yDir = -1;
    }

    if (yDir > 0) cnt += double(millis() - millisPrev) / duration;
    else if (yDir < 0) cnt -= double(millis() - millisPrev) / duration;
    millisPrev = millis();

    for (int i = 0; i < 3; i++) {
      servo[i].write(map(fourier(i), 90, 180, right[i], straight[i]));
    }
    Serial.println("");
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

  double current = double(analogRead(currentSens) - 505) / 36;
  //orientation = atan2(input[0] - 100, input[2] - 100) * 100;
  //orientation = map(orientation, -314, 314, 0, 359);

  data[1][0] = yDir;
  if (current < 0) data[1][1] = 0;
  else data[1][1] = current * 100;
  data[1][2] = map(input[0], -32768, 32768, 0, 200);
  data[1][3] = map(input[2], -32768, 32768, 0, 200);
  data[1][4] = map(-input[5], -32768, 32768, 0, 200);
  data[1][5] = duration / 10;

  if (sending) {
    radio.stopListening();
    radio.write(&data[1], sizeof(data[1]));
    delay(5);
    radio.startListening();
  }
  else delay(2);
}

void debounce() {
  for (int i = 0; i < 6; i++) {
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

double fourier(int i) {
  float output;
  if (mode) output = 120 - amplitude * cos(4 * PI * cnt + phase[mode][i]);
  else output = 120 + 3.2 * cos(2 * PI * cnt + phase[mode][i]) - 43.2 * cos(2 * (2 * PI * cnt + phase[mode][i])) - 20 * cos(3 * (2 * PI * cnt + phase[mode][i]));
  Serial.print(String(output) + '\t');
  return output;
}
