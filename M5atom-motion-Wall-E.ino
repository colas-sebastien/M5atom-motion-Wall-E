/*

    M5 Atom WALL-E Project

    Create your own WALL-E thanks to M5 Atom / M5 Atom Echo

    Author: Sébastien Colas

 */

#include <M5Atom.h>
#include <SPIFFS.h>
#include <Grove_Motor_Driver_TB6612FNG.h>
#include <Ultrasonic.h>
#include <Wire.h>
#include <Bluepad32.h>
#include <Adafruit_NeoPixel.h>
#include "AtomMotion.h"

#define PIN_RGB                 27  
#define NUMPIXELS                1   
#define GAMEPAD_DEAD_ZONE_X     50
#define GAMEPAD_DEAD_ZONE_Y     50
#define GAMEPAD_MAX_X          510
#define GAMEPAD_MAX_Y          510
#define GAMEPAD_MAX_BRAKE     1020
#define GAMEPAD_MAX_THROTTLE  1020
#define GAMEPAD_BUTTON_X         0
#define GAMEPAD_BUTTON_Y         1
#define GAMEPAD_BUTTON_A         2
#define GAMEPAD_BUTTON_B         3
#define GAMEPAD_BUTTON_L1        4
#define GAMEPAD_BUTTON_R1        5
#define GAMEPAD_BUTTON_PREV      6
#define GAMEPAD_BUTTON_NEXT      7
#define GAMEPAD_BUTTON_LOGO      8

#define MOTOR_MIN_SPEED         50
#define MOTOR_MAX_SPEED        255
#define MOTOR_A_DIR              1
#define MOTOR_B_DIR             -1

#define SMOOTH_NUMBER           20

/*
 * PORT A => PORT_SDA 26
 *           PORT_SCL 32
 *
 * For Atom Motion
 * PORT B => PORT_SDA 33
 *           PORT_SDA 23
 * PORT C => PORT_SDA 19
 *           PORT_SDA 22
 * I2C    => PORT_SDA 25
 *        => PORT_SCL 21 
 */
// PORT B for Motor Cotroller
#define PORT_SDA                  33
#define PORT_SCL                  23

// PORT C for UltraSonicSensor
#define ULTRASONIC_PORT           22

Adafruit_NeoPixel pixels = Adafruit_NeoPixel( NUMPIXELS, PIN_RGB,NEO_GRB + NEO_KHZ800);

ControllerPtr myControllers[BP32_MAX_CONTROLLERS];
MotorDriver motor;

AtomMotion Atom;
bool direction = true;
Ultrasonic ultrasonic(ULTRASONIC_PORT);
long RangeInCentimeters;

bool    buttons[9] = {false, false, false, false, false, false, false, false, false};
//      Buttons         X      Y      A      B     L1     R1     PREV   NEXT   LOGO

int32_t hands_values[2][SMOOTH_NUMBER];
int8_t  hands_index=0;

int32_t smooth_value(int32_t *values)
{
  int32_t value =    0;
  int32_t min   = 1000;
  int32_t max   =    0;
  int32_t sum   =    0;

  for (int32_t i=0;i<SMOOTH_NUMBER;i++)
  {
    if (values[i]<min)
    {
      min=values[i];
    }
    if (values[i]>max)
    {
      max=values[i];
    }
    sum+=values[i]; 
  }
  return round((sum-min-max)/(SMOOTH_NUMBER-2));
}

bool button_pressed(bool state,uint8_t id)
{
  bool pressed=false;
  if (state)                      // Button is currently pressed
  {                               
    if (buttons[id])              // Button is still pressed 
    {
                                  // Nothing to to
    }
    else                          // Button was previously  not pressed
    {
      buttons[id]=true;
      pressed=true;   
    }    
  }
  else
  {
    buttons[id]=false;
  }
  return pressed;
}

void GetServoStatus() 
{
    for (int ch = 1; ch < 4; ch++) 
    {
        Serial.printf("Servo Channel %d: %d \n", ch, Atom.ReadServoAngle(ch));
    }
}

void setup() {

  for (int i=0;i<SMOOTH_NUMBER;i++)
  {
    hands_values[0][i]=0;
    hands_values[1][i]=0;
  }
  M5.begin(true,false);
  Atom.Init();  
  pixels.begin();
  pixels.setPixelColor(0,0,0,200);
  pixels.show();
  Wire.begin(PORT_SDA, PORT_SCL);
  Serial.begin(115200);
  while (!Serial) {
    ;
  }  

  Serial.println();
  Serial.println("M5Atom Wall-E");

  String fv = BP32.firmwareVersion();
  Serial.print("Firmware version installed: ");
  Serial.println(fv);

  // To get the BD Address (MAC address) call:
  const uint8_t* addr = BP32.localBdAddress();
  Serial.print("BD Address: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(addr[i], HEX);
    if (i < 5)
      Serial.print(":");
    else
      Serial.println();
  }

  motor.init();  

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();       
}

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;

  pixels.setPixelColor(0,0,50,0);
  pixels.show();

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.print("CALLBACK: Controller is connected, index=");
      Serial.println(i);
      myControllers[i] = ctl;
      foundEmptySlot = true;

      // Optional, once the gamepad is connected, request further info about the
      // gamepad.
      ControllerProperties properties = ctl->getProperties();
      char buf[80];
      sprintf(buf,
              "BTAddr: %02x:%02x:%02x:%02x:%02x:%02x, VID/PID: %04x:%04x, "
              "flags: 0x%02x",
              properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
              properties.btaddr[3], properties.btaddr[4], properties.btaddr[5],
              properties.vendor_id, properties.product_id, properties.flags);
      Serial.println(buf);
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println(
        "CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundGamepad = false;
  
  pixels.setPixelColor(0,100,0,0);
  pixels.show();

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.print("CALLBACK: Controller is disconnected from index=");
      Serial.println(i);
      myControllers[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    Serial.println(
        "CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

bool    head_pos      = true;
bool    hand_manual   = false;
bool    hand_left_up  = false;
bool    hand_right_up = false;


void processGamepad(ControllerPtr gamepad) {
  
  int16_t speed_x;
  int16_t speed_y;

  if (hand_manual)
  { 
    hands_values[0][hands_index]=gamepad->brake();
    hands_values[1][hands_index]=gamepad->throttle();

    hands_index=(hands_index+1)%SMOOTH_NUMBER;    
    
    Atom.SetServoAngle(1, 180-smooth_value(hands_values[0])*120/GAMEPAD_MAX_BRAKE);        // Left Hand
    Atom.SetServoAngle(3, 0  +smooth_value(hands_values[1])*120/GAMEPAD_MAX_THROTTLE);     // Right Hand
  }

  if (button_pressed(gamepad->a(),GAMEPAD_BUTTON_A))                // A pressed
  {                                                                 // Hands down + Head front
    Atom.SetServoAngle(1, 180);                                     // Left Hand
    Atom.SetServoAngle(3, 0  );                                     // Right Hand
    Atom.SetServoAngle(2, 90);                                      // Head Front
  }

  if (button_pressed(gamepad->b(),GAMEPAD_BUTTON_B))                // B pressed
  {                         
    hand_manual=!hand_manual;                                       // Activating hand movement control by triggers
  }

  if (button_pressed(gamepad->x(),GAMEPAD_BUTTON_X))                // X pressed
  {
    Atom.SetServoAngle(1,   0);                                     // Left Hand Rear
    Atom.SetServoAngle(3, 180);                                     // Right Hand Rear
    Atom.SetServoAngle(2, 90);                                      // Head Front           
  }

  if (button_pressed(gamepad->y(),GAMEPAD_BUTTON_Y))               // Y pressed
  {
    head_pos=!head_pos;
    if (head_pos)
    {
      Atom.SetServoAngle(2, 110);
    }
    else
    {
      Atom.SetServoAngle(2, 70);
    }
  }

  if (button_pressed(gamepad->l1(),GAMEPAD_BUTTON_L1))              // Left Button pressed
  {                               
      hand_left_up=!hand_left_up;                                   // Move up or down the left hand
      hand_manual=false;                                            // Take over manual control
      if (hand_left_up)
      {
        Atom.SetServoAngle(1, 180-120);
      }
      else
      {
        Atom.SetServoAngle(1, 180);
      }
  }

  if (button_pressed(gamepad->r1(),GAMEPAD_BUTTON_R1))              // Rigth Button pressed
  {                               
      hand_right_up=!hand_right_up;                                 // Move up or down the left hand
      hand_manual=false;                                            // Take over manual control
      if (hand_right_up)
      {
        Atom.SetServoAngle(3, 0+120);
      }
      else
      {
        Atom.SetServoAngle(3, 0);
      }
  }

  if (button_pressed(gamepad->miscBack(),GAMEPAD_BUTTON_PREV))     // Left Button pressed
  {                               
      hand_left_up=true;                                            // Move up the left hand up
      hand_manual=false;                                            // Take over manual control
      Atom.SetServoAngle(1, 180-120);                               // Left hand up
      hand_right_up=false;                                          // Move the right hand down
      Atom.SetServoAngle(3, 0);                                     // Right hand down
      head_pos=false;                                               // Move Head to the right
      Atom.SetServoAngle(2, 70);                                    // Head to the right

  }

  if (button_pressed(gamepad->miscHome(),GAMEPAD_BUTTON_NEXT))      // Rigth Button pressed
  {                               
      hand_left_up=false;                                           // Move up the left hand down
      hand_manual=false;                                            // Take over manual control
      Atom.SetServoAngle(1, 180);                                   // Left hand down
      hand_right_up=true;                                           // Move the right hand down
      Atom.SetServoAngle(3, 0+120);                                 // Right hand up
      head_pos=true;                                                // Move Head to the right
      Atom.SetServoAngle(2, 110);                                   // Head to the right
  }  


  speed_x=MOTOR_MIN_SPEED+(abs(gamepad->axisX())-GAMEPAD_DEAD_ZONE_X)*MOTOR_MAX_SPEED/GAMEPAD_MAX_X;
  speed_y=MOTOR_MIN_SPEED+(abs(gamepad->axisY())-GAMEPAD_DEAD_ZONE_Y)*MOTOR_MAX_SPEED/GAMEPAD_MAX_Y;
  
  if (gamepad->axisY()<-GAMEPAD_DEAD_ZONE_Y) {          // FORWARD
    if (gamepad->axisX()<-200) {                        //  + LEFT
      motor.dcMotorRun(MOTOR_CHA, -speed_y*MOTOR_A_DIR);
      motor.dcMotorRun(MOTOR_CHB, -(speed_y-speed_x+MOTOR_MIN_SPEED)*MOTOR_B_DIR);
    }
    else if (gamepad->axisX()>200) {                    //  + RIGHT
      motor.dcMotorRun(MOTOR_CHA, -(speed_y-speed_x+MOTOR_MIN_SPEED)*MOTOR_A_DIR);
      motor.dcMotorRun(MOTOR_CHB, -speed_y*MOTOR_B_DIR);
    }
    else
    {
      motor.dcMotorRun(MOTOR_CHA, -speed_y*MOTOR_A_DIR);
      motor.dcMotorRun(MOTOR_CHB, -speed_y*MOTOR_B_DIR);    
    }
  }
  else if (gamepad->axisY()>GAMEPAD_DEAD_ZONE_Y) {      // BACKWARD    
    motor.dcMotorRun(MOTOR_CHA,  speed_y*MOTOR_A_DIR);
    motor.dcMotorRun(MOTOR_CHB,  speed_y*MOTOR_B_DIR);
  }
  else  if (gamepad->axisX()<-GAMEPAD_DEAD_ZONE_X) {    // LEFT
    motor.dcMotorRun(MOTOR_CHA, -150*MOTOR_A_DIR);
    motor.dcMotorRun(MOTOR_CHB,  150*MOTOR_B_DIR);
  }
  else   if (gamepad->axisX()>GAMEPAD_DEAD_ZONE_Y) {    // RIGHT
    motor.dcMotorRun(MOTOR_CHA,  150*MOTOR_A_DIR);
    motor.dcMotorRun(MOTOR_CHB, -150*MOTOR_B_DIR);
  } 
  else {                                // STOP
    motor.dcMotorStop(MOTOR_CHA);
    motor.dcMotorStop(MOTOR_CHB);
  }
}



void loop()
{
  BP32.update();

  // It is safe to always do this before using the controller API.
  // This guarantees that the controller is valid and connected.
  for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
    ControllerPtr myController = myControllers[i];

    if (myController && myController->isConnected()) {
      if (myController->isGamepad())
        processGamepad(myController);
    }
  }

  // RangeInCentimeters = ultrasonic.MeasureInCentimeters(); // two measurements should keep an interval
  // Serial.print(RangeInCentimeters);//0~400cm
  // Serial.println(" cm");  

  delay(10);
}