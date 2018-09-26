/*=========================================================================================*/
 /*ENGR697GW: Senior Design Project V3(Spring 2018)
  * Project: MIDI Drum Glove
  * Members: Wesley Jonson, Colton DeMarr, Tengrithy Khling
  * 
  *Build: Hand Module Version 3
  *-This module will be installed on the feather M0 microcontroller
  *-Functions include:
  *   -Sensor Reading
  *   -MIDI note generation
  *   -Bluetooth conenction and commuincation
  *   -IMU Feature
  *   -Multiple input combinations (Needs reworking to better sense)
  *   -ON/OFF switch
  *Devices: 
  *   -Adafruit Feather M0 Microcontroller
  *   -Conductive adhesive Copper Foil Tape
  *   -Adafruit Velostat (Pressure sensitive fabric)
  *   -MPU6050 IMU sensor
  *   -Flex Sensor (Hand made with Velostat)
  *   -SPST switch
  *   
  *General MIDI Protocol 2 (GM2) will be followed
  *Testing with KORG IDS-10 App for iPhone w/ Midimittr app for virtual BLE MIDI connection 
 =========================================================================================*/   
#include <Arduino.h>
#include <SPI.h>
/*=======================Libraies used for BLE and MIDI===================================*/
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEMIDI.h"
#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.7.0"

/*=======================Libraies used for IMU and I2C===================================*/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

/*=============================Bluetooth/MPU Configuration===================================*/
/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

Adafruit_BLEMIDI midi(ble);

MPU6050 mpu;
/*==========================================================================================*/

const int anaPin0=A0;                         //Analog Pin 0
const int anaPin1=A1;                         //Analog Pin 1
const int anaPin2=A2;                         //Analog Pin 2
const int anaPin3=A3;                         //Analog Pin 3
const int anaPin4=A4;                         //Analog Pin 4
const int anaPin5=A5;                         //Analog Pin 5

const int digiPin11=11;                       //Digital Pin 11
const int digiPin12=12;                       //Digital Pin 5
const int digiPin6=6;                         //Digital Pin 6

//MPU Control/Status Variables
uint8_t fifoBuffer[64];                       //FIFO storage buffer
uint8_t mpuIntStatus;                         //holds actual interrupt status byte from MPU
uint8_t devStatus;                            //return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                          //expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                           //count of all bytes currently in FIFO

// orientation/motion vars
Quaternion q;                                 // [w, x, y, z]         quaternion container
VectorInt16 aa;                               // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;                           // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;                          // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;                          // [x, y, z]            gravity vector
float euler[3];                               // [psi, theta, phi]    Euler angle container

int note_on=0x90;                             //MIDI message for note on, MIDI Channel x
int note_off=0x80;                            //MIDI message for note off, MIDI Channel x
int pitch_bend = 0xE0;
int MIDI_chan = 0x09;
int status_byte = 0x00;
int bend1 = 0x00;                             //Byte 1 for bend MIDI message
int bend2 = 0x00;                             //Byte 2 for bend MIDI message

int wait_time = 0;                            //Sets wait time for whip (IMU channel changer fun)
bool isConnected = false;                     //Confirms sucessful bluetooth connection
bool sense_trigg = true;                      //Turns sensors on and off
bool flex_middle = false;                     //Senses flexsensor input for middle finger
bool flex_index = false;                      //Senses flexsensor input for index finger

volatile bool mpuInterrupt = false;           // indicates whether MPU interrupt pin has gone high
bool blinkState = false;                      //Used for MPU to show what happening
bool dmpReady = false;                        //Set true if DMP init was successful
bool whip = false;                            //Detects positive channel change (IMU Channel changer fun)

//bool tap0 = false;                          //Confirms sucessful sensor 0 tap
bool tap1 = false;                            //Confirms sucessful sensor 1 tap
bool tap2 = false;                            //Confirms sucessful sensor 2 tap
bool tap3 = false;                            //Confirms sucessful sensor 3 tap
bool tap4 = false;                            //Confirms sucessful sensor 4 tap
bool tap5 = false;                            //Confirms sucessful sensor 4 tap

/*=============================Error Function===============================================*/
/*Function called in "steup" and will indicate bluetooth connection error may not need anymore*/
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}
/*==========================================================================================*/

/*===================Bluetooth Connection Confirmation Functions============================*/
/*Following functions will display to user sucesful connection or not*/
void connected(void)                          //Used in callbacks in setup
{
  isConnected = true;

  delay(1000);
}

void disconnected(void)                       //Used in callbacks in setup
{
  isConnected = false;
}

//Setup for MIDI library
void BleMidiRX(uint16_t timestamp, uint8_t status, uint8_t byte1, uint8_t byte2)
{
  Serial.print("[MIDI ");
  Serial.print(timestamp);
  Serial.print(" ] ");

  Serial.print(status, HEX); Serial.print(" ");
  Serial.print(byte1 , HEX); Serial.print(" ");
  Serial.print(byte2 , HEX); Serial.print(" ");

  Serial.println();
}
/*==========================================================================================*/

/*================================Setup All Functions=======================================*/
void setup(void)
{
/*================================Bluetooth Setup===========================================*/
  Serial.begin(115200);
  
  ble.begin(VERBOSE_MODE);                                          //Initialize BLE Module

  ble.factoryReset();                                               //Factory Reset

  ble.echo(false);
  
  ble.info();                                                       //Prints Bluetooth info but may not be needed

  ble.setConnectCallback(connected);                                //Set BLE Callbacks
  ble.setDisconnectCallback(disconnected);

  midi.setRxCallback(BleMidiRX);                                    //Set MIDIRx Callback

  midi.begin(true);                                                 //Enable MIDI
  
  ble.verbose(false);                                               //Disable command echo from Bluefruit
  
  pinMode(digiPin6, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(digiPin6), sensorMode, FALLING);   //Interrupt for ON/OFF

  pinMode(digiPin12, INPUT);
  attachInterrupt(digitalPinToInterrupt(digiPin12), pitchInterrupt, RISING);     //Interrupt for IMU

  pinMode(digiPin11, INPUT);
  attachInterrupt(digitalPinToInterrupt(digiPin11), chanInterrupt, RISING);     //Interrupt for IMU
/*================================IMU Setup Functions=======================================*/
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    mpu.initialize();

    // verify connection
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(148);
    mpu.setYGyroOffset(-87);
    mpu.setZGyroOffset(-11);
    mpu.setXAccelOffset(-159);
    mpu.setYAccelOffset(268);
    mpu.setZAccelOffset(546);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}
/*==========================================================================================*/

/*===================================Main Function==========================================*/
void loop(void)
{
  ble.update(5);                                                     //Interval for each scanning ~ 5ms (non blocking). Polls for new data
  if (! isConnected){                                                //Terminates loop if bluetooth is not conencted
    Serial.print("BLE OFF");
    Serial.print("\n");
    return;
  }
    
  if (!dmpReady){                                                    //Terminates if IMU is not setup properly
    Serial.print("DMP OFF");
    Serial.print("/n");
    return;
  }

  if (! sense_trigg){                                                //Terminates loop if sensors are turned off
    Serial.print("SWITCH OFF");
    Serial.print("\n");
    return;
  }
  
//  int tap_th0 =5;                                                  //Sets threshold for finger sensors velocity
  int tap_th1 =10;                                                   //Sets threshold for palm sensor velocity  
  int tap_th2 =7;                                                    //Sets threshold for finger sensors velocity
  int tap_th3 =7;                                                    //Sets threshold for palm sensor velocity 
  int tap_th4 =7;                                                    //Sets threshold for palm sensor velocity 
  int tap_th5 =7;                                                    //Sets threshold for palm sensor velocity 
  
//  int note0 = 37;                                                  //note 0 based on General MIDI 2 protocol (37=C#2=Snare on KORG)
  int note1 = 42;                                                    //note 1 based on General MIDI 2 protocol (38=D2=High-Hat on KORG)
  int note2 = 46;                                                    //note 2 based on General MIDI 2 protocol (39=D#2=High-Hat2 on KORG))
  int note3 = 41;                                                    //note 3 based on General MIDI 2 protocol (40=E2=Tom on KORG)
  int note4 = 36;                                                    //note 4 based on General MIDI 2 protocol (36=C2=Bass on KORG)
  int note5 = 38;                                                    //note 4 based on General MIDI 2 protocol (36=C2=Bass on KORG)

  //ANALOG PIN 0 IS NOT BEING USED BUT STILL NEEDS TO BE READ TO PREVENT ERRORS!!! Conected to GND
  int note0_velocity = map(analogRead(anaPin0),0, 300, 0, 127);    //Reads analog voltage from pin 0 and maps in range 0-127
  delay(1);
  int note1_velocity = map(analogRead(anaPin1),0, 300, 0, 127);    //Reads analog voltage from pin 1 and maps in range 0-127
  delay(1);
  int note2_velocity = map(analogRead(anaPin2),0, 300, 0, 127);    //Reads analog voltage from pin 2 and maps in range 0-127
  delay(1);
  int note3_velocity = map(analogRead(anaPin3),0, 300, 0, 127);    //Reads analog voltage from pin 3 and maps in range 0-127
  delay(1);
  int note4_velocity = map(analogRead(anaPin4),0, 300, 0, 127);    //Reads analog voltage from pin 4 and maps in range 0-127
  delay(1);
  int note5_velocity = map(analogRead(anaPin5),0, 300, 0, 127);    //Reads analog voltage from pin 4 and maps in range 0-127
  delay(1);
  
//  Serial.print(note0_velocity);
//  Serial.print("\t");
  Serial.print(note1_velocity);
  Serial.print("\t");
  Serial.print(note2_velocity);
  Serial.print("\t");
  Serial.print(note3_velocity);
  Serial.print("\t");
  Serial.print(note4_velocity);
  Serial.print("\t");
  Serial.print(note5_velocity);
  Serial.print("\t");
  Serial.print("\n");

/*
 * Sensor 1 = Palm, Sensor 2 = Pinky, Sensor 3 = Ring, Sensor 4 = Index, Sensor 5 = Middle (I messed up when wiring -_-)
 * 
 * Goal of this version of code is to only use sensor combinations to activate different notes. To do this the threshold for the finger sensors are lowered to make them all equally sensitive.
 * No sensor on its own will activate a note. The reason for this because the since the threshold is lower so low, the sensor will activate continously even when not actuated with a hit
 * The palm sensor has its own threshold since it does not require input combinations
*/
  
/*Snare (Index + Middle)*/  
  //"If" statements control if the sensors are pressed and released
  if (note5_velocity > tap_th5 && note4_velocity > tap_th4 && note3_velocity < tap_th3 && note2_velocity < tap_th2 &&  tap5 == false &&  tap4 == false &&  tap3 == false &&  tap2 == false){                   
    status_byte = note_on + MIDI_chan;
    midi.send(status_byte, note5, 0x64); 
    tap4 = true;                                                 //Stops multiple notes being sensed while note is held
    delay(3);
    Serial.println("SNARE DRUM");
    Serial.println("\n");
  }
  else if (note5_velocity < tap_th5 && note4_velocity < tap_th4 && note3_velocity < tap_th3 && note2_velocity < tap_th2 && tap4 ==true ){  
    status_byte = note_off + MIDI_chan;                 
    midi.send(status_byte, note5, 0x64);
    tap4= false;                                                //Update tap to allow another tap to be sensed
  }

/*High-Hat 1 (Middle + Ring)*/
  if (note5_velocity > tap_th5 && note4_velocity < tap_th4 && note3_velocity > tap_th3 && note2_velocity < tap_th2 && tap5 == false &&  tap4 == false &&  tap3 == false &&  tap2 == false){  
    status_byte = note_on + MIDI_chan;                         
    midi.send(status_byte, note1, 0x64);
    tap5 = true;
    delay(3);
    Serial.println("HIGH-HAT 1");
    Serial.println("\n");
  }
  else if (note5_velocity < tap_th5 && note4_velocity < tap_th4 && note3_velocity < tap_th3 && note2_velocity < tap_th2 && tap5 ==true){ 
    status_byte = note_off + MIDI_chan;                  
    midi.send(status_byte, note1, 0x64);
    tap5 = false;                                               
  }

/*High-Hat 2 (Middle + Ring)*/
  if (note5_velocity < tap_th5 && note4_velocity < tap_th4 && note3_velocity > tap_th3 && note2_velocity > tap_th2 && tap5 == false &&  tap4 == false &&  tap3 == false &&  tap2 == false){                        
    status_byte = note_on + MIDI_chan;                                               
    midi.send(status_byte, note2, 0x64); 
    tap3 = true;
    delay(3);
    Serial.println("HIGH-HAT 2");
    Serial.println("\n");
  }
   else if (note5_velocity < tap_th5 && note4_velocity < tap_th4 && note3_velocity < tap_th3 && note2_velocity < tap_th2 && tap3 ==true){
    status_byte = note_off + MIDI_chan;                  
    midi.send(status_byte, note2, 0x64); 
    tap3 = false;                                                
  }

/*Tom (Index + Pinky)*/
  if (note5_velocity < tap_th5 && note4_velocity > tap_th4 && note3_velocity < tap_th3 && note2_velocity > tap_th2 && tap5 == false &&  tap4 == false &&  tap3 == false &&  tap2 == false){                        
    status_byte = note_on + MIDI_chan; 
    midi.send(status_byte, note3, 0x64);
    tap2 = true;
    delay(3); 
    Serial.println("TOM");
    Serial.println("\n");
  }
   else if (note5_velocity < tap_th5 && note4_velocity < tap_th4 && note3_velocity < tap_th3 && note2_velocity < tap_th2 && tap2 ==true){               
    status_byte = note_off + MIDI_chan; 
    midi.send(status_byte, note3, 0x64); 
    tap2 = false;                                                  
  }

/*Bass Drum (Palm)*/ 
  if (note1_velocity > tap_th1 && tap1 == false){
    status_byte = note_on + MIDI_chan;                                                                       
    midi.send(status_byte, note4, 0x64); 
    tap1 = true; 
    delay(3);
    Serial.print("BASS DRUM");
    Serial.print("\n");
  }
   else if (note1_velocity < tap_th1 && tap1 ==true){   
    status_byte = note_off + MIDI_chan;           
    midi.send(status_byte, note4, 0x64); 
    tap1 = false;                                                 
  }

/*IMU Functions*/
  if (flex_index)
    Pitch_FUN();
  if (flex_middle)
    Chan_FUN();
  
}
/*==========================================================================================*/
void Pitch_FUN(void){
    Serial.print("PITCH CHANGE");
    mpu.resetFIFO();
    while(flex_index == true){
    fifoCount = mpu.getFIFOCount();
    
    if (fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
    }
    else{
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) 
        fifoCount = mpu.getFIFOCount();
  
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      mpu.resetFIFO();
          
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
  
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      
      status_byte = pitch_bend + MIDI_chan;

      if (euler[2] < 0.087 && euler[2] > -0.087){
        bend1 = 0x00;
        bend2 = 0x40;
      }
      else if (euler[2] > 0.087 && euler[2] <= 0.785){
        bend1 = 0x00;
        bend2 = 80.2141*euler[2] + 64;
      }
      else if (euler[2] > 0.785){
        bend1 = 161.7014*euler[2] - 127;
        bend2 = 0x7F;
      }
      else if (euler[2] < -0.087){
        bend1 = 0x00;
        bend2 = 40.107*euler[2] + 63;
      }
      Serial.print(bend1);
      Serial.print("\t");
      Serial.print(bend2);
      Serial.print("\n");
      midi.send(status_byte, bend1, bend2); 
    }
   }
   Serial.print("LOOP EXITED");
   Serial.print("\n");
}
/*===================================Interrupt Function=====================================*/
void Chan_FUN(void){
    Serial.print("CHANNEL CHANGE");
    mpu.resetFIFO();
    while(flex_middle == true){
      fifoCount = mpu.getFIFOCount();
      
      if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
      }
      else{
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) 
          fifoCount = mpu.getFIFOCount();
    
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.resetFIFO();
            
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
    
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  
        if (aaWorld.y > 10000 && whip == false){
          whip = true;
          if (MIDI_chan > 0x00)
            MIDI_chan--;
          Serial.print(MIDI_chan);
          Serial.print("\n");
        }
        else if (aaWorld.y < -10000 && whip == false){
          whip = true;
          if (MIDI_chan < 0x0F)
            MIDI_chan++;
          Serial.print(MIDI_chan);
          Serial.print("\n");
        }
        
        if (whip==true)
          wait_time++;
        if (wait_time == 50){
          whip = false;
          wait_time =0;
        }
      }
   }
   Serial.print("LOOP EXITED");
   Serial.print("\n");
}
/*===================================Interrupt Function=====================================*/
/*Interrupt will trigger sensors on or off*/
void sensorMode(void)
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200){
    if (sense_trigg){
      sense_trigg = false;
    }
    else{
      sense_trigg = true; 
    }
  }
  last_interrupt_time = interrupt_time;
}
/*==========================================================================================*/
//May need to change for our purpose maybe have flex on pin 5 to activate interrupt
//maybe add wakeup function to interrupt to enable imu
/*===================================Interrupt for IMU Function=============================*/
void pitchInterrupt() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200){
    if (!flex_index){
      flex_index = true;
    }
    else{
      flex_index = false;
    }
  }
  last_interrupt_time = interrupt_time;
}
/*==========================================================================================*/
void chanInterrupt() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200){
    if (!flex_middle){
      flex_middle = true;
    }
    else{
      flex_middle = false;
    }
  }
  last_interrupt_time = interrupt_time;
}

