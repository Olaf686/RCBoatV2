
//             RCBOAT
//
//                 __/___
//           _____/______|
//   _______/_____\_______\_____
//   \              < < <       |
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//

#include <nRF24L01.h>  //library for transceiver (nrf24l01+ PA+LNA) https://github.com/nRF24/RF24
#include <RF24.h>
#include <SPI.h>       //library for SPI protocol

#include <Adafruit_SSD1306.h>  //library for display (SSD1306 128x64) https://github.com/adafruit/Adafruit_SSD1306
#include <Wire.h>              //library for I2C protocol

const byte CE = 7;                   //rf24 CE pin
const byte CSN = 8;                  //rf24 CSN pin
const byte address[5] = {0,0,0,0,1};  //rf24 address

bool connection = 0;
float batteryCurrent = 0.0;
float batteryVoltage = 0.0;
float boatSpeed = 0.0;
unsigned long timer;         //millis timer

struct receiving{   //structure for telemetry data
  int8_t autoMode;
  int8_t waypoint;
  int16_t batteryVoltage;
  int16_t batteryCurrent;
  int16_t batteryEnergy;
  int16_t boatHeading;
  int16_t boatSpeed;
  int16_t targetDistance;
  int16_t targetHeading;
}__attribute__((packed));

struct receiving answer;                       //telemetry object
Adafruit_SSD1306 display(128, 64, &Wire, -1);  //display object
RF24 radio(CE, CSN);                           //wireless object

void setup() {
  
  radio.begin();                      //start radio service as transmitter
  radio.setPALevel(RF24_PA_MIN);      //transmission power
  radio.setDataRate(RF24_250KBPS);    //data rate
  radio.openReadingPipe(1, address);  //open reading pipe
  radio.startListening();             //start listening for incoming data
 
  Serial.begin(115200);  //set serial baudrate
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {   //boot OLED display
    Serial.println(F("SSD1306 allocation failed"));  //boot failed
    for(;;);
  }
  
  delay(2000);                 //allow startup of display
  display.clearDisplay();      //clear the display
  display.setTextSize(1);      //standard 1:1 pixel scale
  display.setTextColor(WHITE); //set white text
  display.cp437(true);         //use full 256 char 'Code Page 437' font

}

void loop() {
  if(radio.available()){
    radio.read(&answer, sizeof(answer));
    timer = 0;
    answerToDisplay();                    //put all info on the display
  }
  else{
    if(timer == 0)
      timer = millis(); //new timestamp
    if(millis()-timer>10000){
      display.clearDisplay();
      display.setCursor(0, 1);
      display.print("connection lost...");
      display.display();
    }
  }
}


void answerToDisplay(){  //function for putting answer from boat on display
  display.clearDisplay();
  
  display.setCursor(0, 1);
  display.print("Telemetry Data");
  
  display.setCursor(0, 14);
  switch(answer.autoMode){
    case 0:
      display.print("Underway to WP");
      display.print(answer.waypoint);
      break;
    case 1:
      display.print("Arrived at WP");
      display.print(answer.waypoint);
      break;
    case 2:
      display.print("No GPS Data!");
      break;
    case 3:
      display.print("Manual Mode");
      break;
    case 4:
      display.print("Override to WP");
      display.print(answer.waypoint);
      break;
    default:
      display.print("No valid status code");
      break;
  }
  
  display.setCursor(0, 27);
  batteryVoltage = answer.batteryVoltage / 100.0;
  display.print(batteryVoltage, 2);
  display.print("V ");
  batteryCurrent = answer.batteryCurrent / 100.0;
  display.print(batteryCurrent, 2);
  display.print("A ");
  display.print(answer.batteryEnergy);
  display.print("mAh");
  
  display.setCursor(0, 40);
  display.print("Course: ");
  display.print(answer.boatHeading);
  display.print((char)248);
  display.print(" ");
  boatSpeed = answer.boatSpeed/10.0;
  display.print(answer.boatSpeed, 1);
  display.print("km/h");
  
  display.setCursor(0, 53);
  display.print("Target: ");
  if(answer.targetHeading >= 0)
    display.print("+");
  display.print(answer.targetHeading);
  display.print((char)248);
  display.print(" ");
  display.print(answer.targetDistance);
  display.print("m");

  display.display();
  
}
