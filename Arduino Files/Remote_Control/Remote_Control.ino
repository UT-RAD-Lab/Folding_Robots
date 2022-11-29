/*
   Description: This program is for the Universal Remote Control

        Wiring: The required components are 2x joysticks, 2x momentary switches, and 2x potentiometer

    Created by: Josh Stanley

    Date Created: 06/03/2022

    Date Modified: 06/03/2022
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(9, 10);                         //CE, CSN
const byte chan[6] = "00007";              //Byte of array representing the address. This is the address where we will send the data. This should be same on the receiving side.

int r_xPot = A2;
int r_yPot = A3;
int r_joy = A1;
int l_xPot = A5;
int l_yPot = A6;
int l_joy = A4;
int r_bumper = 2;
int l_bumper = 3;
int r_pot = A0;
int l_pot = A7;

byte data[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void setup() {
  Serial.begin(9600);
  Serial.println("Serial Communication Initialized");

  pinMode(r_xPot, INPUT);
  pinMode(r_yPot, INPUT);
  pinMode(l_xPot, INPUT);
  pinMode(l_yPot, INPUT);
  pinMode(r_joy, INPUT_PULLUP);
  pinMode(l_joy, INPUT_PULLUP);
  pinMode(r_bumper, INPUT_PULLUP);
  pinMode(l_bumper, INPUT_PULLUP);
  pinMode(r_pot, INPUT);
  pinMode(l_pot, INPUT);

  radio.begin();                           //Starting the radio communication
  radio.openWritingPipe(chan);             //Setting the address at which we will send the data
  radio.setPALevel(RF24_PA_MIN);           //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.stopListening();
}

void loop() {
  if (analogRead(r_xPot) > 515 && analogRead(r_xPot) < 545) data[0] = 0;
  else data[0] = map(analogRead(r_xPot), 0, 1023, 1, 255);
  
  if (analogRead(r_yPot) > 486 && analogRead(r_yPot) < 516) data[1] = 0;
  else data[1] = map(analogRead(r_yPot), 0, 1023, 1, 255);
  
  data[2] = !digitalRead(r_joy);
  data[3] = !digitalRead(r_bumper);

  if (analogRead(l_xPot) > 490 && analogRead(l_xPot) < 520) data[4] = 0;
  else data[4] = map(analogRead(l_xPot), 0, 1023, 1, 255);
  
  if (analogRead(l_yPot) > 505 && analogRead(l_yPot) < 535) data[5] = 0;
  else data[5] = map(analogRead(l_yPot), 0, 1023, 1, 255);
  
  data[6] = !digitalRead(l_joy);
  data[7] = !digitalRead(l_bumper);
  data[8] = map(analogRead(r_pot), 0, 1023, 0, 255);
  data[9] = map(analogRead(l_pot), 0, 1023, 0, 255);


  for (int i = 0; i < 10; i++) {
    Serial.print(data[i]);
    Serial.print('\t');
  }
  Serial.println("");
  radio.write(&data, sizeof(data));
  delay(10);
}
