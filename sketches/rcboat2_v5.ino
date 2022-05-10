
//             RCBOAT
//
//                 __/___
//           _____/______|
//   _______/_____\_______\_____
//   \              < < <       |
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//flight controller sketch for the Arduino Mega
//additional components:
//gps (GY-NEO6MV2)
//compass (QMC5883L)
//telemetry radio (nRF24l01+ PA+LNA)
//current sensor (ACS712)
//25V sensor (1:4 voltage divider)
//ppm receiver (Turnigy iA8)
//esc (ZS X11F)
//servo (SG90 9g)

#include <Servo.h>            //library for rudder servo (SG90)
#include <SPI.h>              //library for SPI communication
#include <PPMReader.h>        //library for PPM receiver input https://github.com/dimag0g/PPM-reader

#include <TinyGPS++.h>        //library for gps (GY-NEO6MV2) https://github.com/mikalhart/TinyGPSPlus
#include <QMC5883LCompass.h>  //library for compass (QMC5883L) https://github.com/mprograms/QMC5883LCompass

#include <RF24.h>             //library for transceiver (nRF24l01+ PA+LNA) https://github.com/nRF24/RF24
#include <nRF24L01.h>
#include <RF24_config.h>

const int interruptPin = 18;          //ppm pin
const int channelAmount = 8;          //amount of channels
const int pwmPin = 2;                 //motor pwm pin
const int dirPin = 3;                 //motor direction pin
const int servoPin = 44;              //rudder servo pin
const int batteryVoltagePin = A14;    //battery voltage pin
const int batteryCurrentPin = A15;    //battery voltage pin
const int CE = 48;                    //rf24 CE pin
const int CSN = 49;                   //rf24 CSN pin
const byte address[5] = {0,0,0,0,1};  //rf24 address

int pwmMax;                    //max throttle (pwm duty cycle) while in autoMode (range: 0 <-> 255)
unsigned long timeOut = 0;     //timer for transmitter connection failsafe
unsigned long timer = 0;       //timer for reading input and setting output for motor and rudder
unsigned long radioTimer = 0;  //timer for telemetry transmit freqency

long distanceToTarget;  //distance in meters to target location (from GPS)
int courseToTarget;     //course in degrees to target location (from GPS, range: 0 <-> 359*)
int azi;                //azimuth heading of the boat (from compass, range: 0 <-> 359*)
int targetHeading;      //course to target with respect to current heading (range: -180* <-> 180*)

const int currentMeasurements = 150;  //number of current measurements
int current[currentMeasurements];     //measurement array
int currentCounter = 0;               //array index
long currentTotal = 0L;               //total sum of measurements
float batteryCurrent = 0.0;
float batteryEnergy = 0.0;
unsigned long currentMillis = 0;      //timer for starting a new measurement

const int voltageMeasurements = 100;  //number of voltage measurements
int voltage[voltageMeasurements];     //measurement array
int voltageCounter = 0;               //array index
long voltageTotal = 0L;               //total sum of measurements
float batteryVoltage = 0.0;
unsigned long voltageMillis = 0;      //timer for starting a new measurement

struct receiving{  //structure for telemetry data
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

struct receiving answer;                     //telemetry object
Servo rudderServo;                           //servo object
int rudderAngle = 90;                        //rudder value
TinyGPSPlus gps;                             //gps object
QMC5883LCompass compass;                     //compass object
RF24 radio(CE, CSN);                         //wireless object
PPMReader ppm(interruptPin, channelAmount);  //ppm object
int PWM[channelAmount];                      //ppm values array

const int waypointSize = 4;                //number of waypoints
const float waypoint[waypointSize][2] = {  //waypoints for autoMode
  {51.966497, 6.274391},
  {51.966700, 6.274511},
  {51.966750, 6.274232},
  {51.966608, 6.274146},
};
byte waypointCounter = 0;

void setup() {
  pinMode(pwmPin, OUTPUT);  //set pwm pin as output
  pinMode(dirPin, OUTPUT);  //set direction pin as output

  pinMode(22, OUTPUT);     //Vcc for low power i/o module (ACS712)
  digitalWrite(22, HIGH);
  pinMode(23, OUTPUT);     //Vcc for low power i/o module (compass)
  digitalWrite(23, HIGH);
  pinMode(24, OUTPUT);     //GND for low power i/o module (ACS712)
  digitalWrite(24, LOW);
  pinMode(25, OUTPUT);     //GND for low power i/o module (compass)
  digitalWrite(25, LOW);
  pinMode(26, OUTPUT);     //GND for low power i/o module (25V voltage divider)
  digitalWrite(26, LOW);
  
  rudderServo.attach(servoPin);  //attaches the servo to the servo object
  
  radio.begin();                    //start radio service as receiver
  radio.setPALevel(RF24_PA_MIN);    //transmission power
  radio.setDataRate(RF24_250KBPS);  //data rate
  radio.setRetries(5,5);            //delay, count
  radio.openWritingPipe(address);   //set pipe address
  radio.stopListening();            //stop listening
  
  compass.init();                                               //start compass
  compass.setCalibration(-1490, 328, -612, 1190, -1690, 1500);  //compass calibration
  
  Serial.begin(115200);  //Arduino <-> PC baudrate
  Serial2.begin(9600);   //Arduino <-> GPS baudrate
  
}

void loop() {
  
  while (Serial2.available()){   //acquire stream of GPS data
    gps.encode(Serial2.read());  //process GPS data
  }
  
  if(millis() - currentMillis >= 3){                          //measure current every 3ms
    currentMillis = millis();                                 //reset timer
    current[currentCounter] = analogRead(batteryCurrentPin);  //measure current
    currentCounter++;                                         //increase counter
    if(currentCounter == currentMeasurements)                 //reset counter if end of array
      currentCounter = 0;
  }

  if(millis() - voltageMillis >= 5){                          //measure voltage every 5ms
    voltageMillis = millis();                                 //reset timer
    voltage[voltageCounter] = analogRead(batteryVoltagePin);  //measure voltage
    voltageCounter++;                                         //increase counter
    if(voltageCounter == voltageMeasurements)                 //reset counter if end of array
      voltageCounter = 0;
  }

  if(millis() - timer >= 20){  //process new transmitter data every 20ms
    timer = millis();          //reset timer
    
    for (byte channel = 1; channel <= channelAmount; ++channel) {  //loop over channels
      PWM[channel-1] = ppm.latestValidChannelValue(channel, 0);    //obtain channels
    }
    /*
    if(PWM[0] == 0){                  //nothing received from transmitter
      if(timeOut == 0)                //if last loop iteration received something, start the timeout counter
        timeOut = millis();           //timestamp for last transmission received
      if(millis() - timeOut > 5000){  //after 5 seconds of continuous lost signal -> something is wrong!
        analogWrite(pwmPin, 0);       //stop the motor
        rudderServo.write(90);        //neutral rudder position
        answer.txStatus = 0;          //report disconnected transmitter
      }
    }
    else{
      answer.txStatus = 1;  //transmitter message received
    }
    */
    if(PWM[0])
      PWM[0] = map(PWM[0], 1080, 1920, 135, 43);  //map ppm value to servo angle
    else
      PWM[0] = 90;                                //neutral rudder position in case of disconnected Tx
    PWM[0] = constrain(PWM[0], 45, 135);          //constrain servo angle
    
    if(PWM[1])
      PWM[1] = map(PWM[1], 1080, 1920, -100, 100);  //map speed to 8 bit value
    else
      PWM[1] = 0;
    PWM[1] = constrain(PWM[1], -100, 100);           //constrain speed value
    
    PWM[2] = map(PWM[2], 1080, 1920, 0, 255);   //map speed to 8 bit value
    PWM[2] = constrain(PWM[2], 0, 255);         //constrain speed value
    
    PWM[4] = map(PWM[4], 1080, 1920, 0, 100);     //map potmeter to boolean value
    PWM[4] = constrain(PWM[4], 0, 100);           //constrain switch value
    
    PWM[5] = map(PWM[5], 1080, 1920, 0, 100);     //map potmeter to boolean value for waypoint override
    PWM[5] = constrain(PWM[5], 0, 100);           //constrain switch value
    
    PWM[6] = map(PWM[6], 1080, 1920, 0, (waypointSize-1)+10);  //map potmeter to waypoint selector
    PWM[6] = constrain(PWM[6], 0, waypointSize-1);             //constrain waypoint value
    
    PWM[7] = map(PWM[7], 1080, 1920, 0, 10);  //map gear switch to boolean value for autoMode
    PWM[7] = constrain(PWM[7], 0, 1);         //constrain switch value
  

  
    if(PWM[7]){                                                                                //determine pwm and rudder with GPS location, compass heading and target location as input
      answer.autoMode = autoMode(waypoint[waypointCounter][0], waypoint[waypointCounter][1]);  //status code of autoMode() for info and debugging
      if(answer.autoMode == 1)                                                                 //set next waypoint if target waypoint reached
        waypointCounter++;
      
      if(waypointCounter = waypointSize)  //end of waypoint array?
        waypointCounter = 0;              //reset back to first waypoint
      
      if(PWM[5]>50){               //waypoint override switch
        waypointCounter = PWM[6];  //set new waypoint
        answer.autoMode = 4;       //override mode
      }
    }
    else{
      manualMode();         //determine pwm and rudder with controller values as input
      answer.autoMode = 3;  //autoMode status code 3: manual mode
    }
  }
  
  if(millis() - radioTimer >= 100){     //send telemetry data every 100ms
    radioTimer = millis();              //reset timer
    answer.waypoint = waypointCounter;  //current waypoint
    
    currentTotal = 0L;                                         //reset total
    for(int i = 0; i < currentMeasurements; i++)               //loop over all measurements
      currentTotal = currentTotal + current[i];                //add samples together to get total
    currentTotal = currentTotal / currentMeasurements;         //calculate average current measurement
    batteryCurrent = (2.5-(currentTotal*(5.0/1024.0)))/0.185;  //calculate average current
    batteryEnergy += batteryCurrent / 36.0;                    //calculate total mAh used

    answer.batteryCurrent = 100*batteryCurrent;
    answer.batteryEnergy = batteryEnergy;
    
    voltageTotal = 0L;                                  //reset total
    for(int i = 0; i < voltageMeasurements; i++)        //loop over all measurements
      voltageTotal = voltageTotal + voltage[i];         //add samples together to get total
    voltageTotal = voltageTotal / voltageMeasurements;  //calculate average voltage measurement
    batteryVoltage = voltageTotal*25.0/1024.0;          //calculate average voltage

    answer.batteryVoltage = 100*batteryVoltage;
    
    compass.read();                             //read compass device for sending answer to controller
    answer.boatHeading = compass.getAzimuth();  //heading of the boat
    
    if(gps.speed.isValid())
      answer.boatSpeed = 10*gps.speed.kmph();  //speed of the boat
    else
      answer.boatSpeed = 0;                    //default speed value
    
    answer.targetDistance = distanceToTarget;  //distance to target
    answer.targetHeading = targetHeading;      //course to target

    for (byte channel = 1; channel <= channelAmount; ++channel) {  //print values
      Serial.print(PWM[channel-1]);
      if(channel <= channelAmount) Serial.print('\t');
    }
    Serial.println("");

    
    if(radio.write(&answer, sizeof(answer)))  //send telemetry data
      Serial.print("radio succes   ");
    else
      Serial.print("radio fail   ");
    
    Serial.print(answer.boatHeading);
    Serial.print("   ");
    
    Serial.print(gps.location.lat(),6);
    Serial.print(" ");
    Serial.println(gps.location.lng(),6);
  }
}

void manualMode(){  //determine pwm and rudder with controller values as input
  
  if(PWM[0] != rudderAngle){         //only updated values are processed (prevents servo jitter)
    rudderAngle = PWM[0];            //import new value
    rudderServo.write(rudderAngle);  //sets rudder servo to correct angle
  }
  
  if(PWM[1] > 50)
    digitalWrite(dirPin, HIGH);  //forwards state
  if(PWM[1] < -50)
    digitalWrite(dirPin, LOW);   //backwards state
  
  analogWrite(pwmPin, PWM[2]);   //generate pwm value
  
}

byte autoMode(float targetLat, float targetLon){  //determine pwm and rudder with GPS location, compass heading and target location as input
                                                  //returns 0 for underway, 1 for destination reached, 2 for no gps data
  if(gps.location.isValid()){                                                                                             //wait for gps fix to start calculating
    distanceToTarget = (long)TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), targetLat, targetLon);  //distance to target (m)
    courseToTarget = int((TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), targetLat, targetLon)+0.5));      //absolute course to target from current position (deg)
  }
  else{                      //autoMode will not work if not receiving a valid location from gps
    analogWrite(pwmPin, 0);  //stop the motor
    rudderServo.write(90);   //neutral servo position
    return 2;                //exit autoMode with error code 2: no gps
  }
  
  if(distanceToTarget < 4){  //very close to target -> destination reached!
    analogWrite(pwmPin, 0);  //stop the motor
    rudderServo.write(90);   //neutral servo position
    return 1;                //exit autoMode with status code 1: destination reached
  }
  
  compass.read();                        //read compass device
  azi = compass.getAzimuth();            //acquire azimuth (= current heading)
  targetHeading = courseToTarget - azi;  //contains relative heading of the target with respect to current heading
  if(targetHeading < 0)                  //correction 1: compass values are now all positive
    targetHeading += 360;
  if(targetHeading > 180)                //correction 2: 'left' values now run from 0 to -180, 'right' 0 to 180
    targetHeading -= 360;                //-180: behind, -90: left, 0: straight ahead, 90: right, 180: behind
  
  pwmMax = PWM[2];                                       //set the maximum duty cycle of the motor
  long pwm = pwmMax - 1L*abs(targetHeading)*pwmMax/180;  //calculate motor pwm target value
  constrain(pwm, 0.5*pwmMax, pwmMax);                    //min and max pwm values
  analogWrite(pwmPin, pwm);                              //apply pwm value
  
  int rudder = 90 - targetHeading;  //calculate rudder servo value
  constrain(rudder, 45, 135);       //min and max rudder position
    
  if(rudder != rudderAngle){         //only updated values are processed (prevents servo jitter)
    rudderAngle = rudder;            //import new value
    rudderServo.write(rudderAngle);  //sets rudder servo to correct angle
  }
  
  return 0;  //autoMode function status code 0: succesfully executed but destination has not been reached yet
}
