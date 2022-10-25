/*
   Description: This program is used to control the body of the FRED Car

        Wiring: The required components are a MG995 servos, and a NRF24L01 radio module
        The servos are connected with brown wire to GND, red wire to Vin, and orange wire to D3
        The NRF24L01 is connected 3.3V to 3v3, GND to GND, CSN to D10, MOSI to D11, CE to D9, SCK to D13, MISO to D12

    Created by: Josh Stanley

    Date Created: 06/08/2022

    Date Modified: 06/08/2022
*/



//Libraries
#include <Servo.h>
#include <nRF24L01.h>3
#include <RF24.h>

RF24 radio(9, 10);
Servo servo;

//Variables
int xDir = 0;
int heading = 0;
byte data[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//Constants
const byte chan[][6] = {"00007", "00008"};

void setup() {
  Serial.begin(9600);
  Serial.println("Serial Communication Initialized");

  radio.begin();
  radio.openReadingPipe(1, chan[0]);
  radio.openWritingPipe(chan[1]);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("Radio Communication Initialized");

  servo.attach(3);
  servo.write(98);
  Serial.println("Servos Attached");
  
  delay(250);
  Serial.println("Running");
}

void loop() {
  if (radio.available()) radio.read(&data, sizeof(data));
  else {
    if(data[4] == 0) xDir = 0;
    else xDir = map(data[4], 1, 255, -45, 45);

    if(xDir > heading) heading++;
    else if(xDir < heading) heading--;

    servo.write(map(heading, -45, 45, 45, 152));
  }
  delay(10);
}
