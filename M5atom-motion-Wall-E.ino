/*
*******************************************************************************
*
*
*
*******************************************************************************
 */

#include <M5Atom.h>
#include "AtomMotion.h"
#include "Grove_Motor_Driver_TB6612FNG.h"
#include "Ultrasonic.h"
#include <Wire.h>
#include <Bluepad32.h>
#include <Adafruit_NeoPixel.h>

#define PIN_RGB                 27  
#define NUMPIXELS                1   
#define GAMEPAD_DEAD_ZONE_X     50
#define GAMEPAD_DEAD_ZONE_Y     50
#define GAMEPAD_MAX_X          510
#define GAMEPAD_MAX_Y          510
#define GAMEPAD_MAX_BRAKE     1020
#define GAMEPAD_MAX_THROTTLE  1020
#define MOTOR_MIN_SPEED         50
#define MOTOR_MAX_SPEED        255
#define MOTOR_A_DIR              1
#define MOTOR_B_DIR             -1

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

void GetStatus() {
    for (int ch = 1; ch < 5; ch++) {
        Serial.printf("Servo Channel %d: %d \n", ch, Atom.ReadServoAngle(ch));
    }
    Serial.printf("Motor Channel %d: %d \n", 1, Atom.ReadMotorSpeed(1));
    Serial.printf("Motor Channel %d: %d \n", 2, Atom.ReadMotorSpeed(2));
}

void setup() {

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

bool    head_pos = true;
bool    hand_manual = false;

void processGamepad(ControllerPtr gamepad) {
  
  int16_t speed_x;
  int16_t speed_y;
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...

  if (hand_manual)
  {
    Atom.SetServoAngle(1, 180-gamepad->brake()*120/GAMEPAD_MAX_BRAKE);           // Left Hand
    Atom.SetServoAngle(3, 0  +gamepad->throttle()*120/GAMEPAD_MAX_THROTTLE);     // Right Hand
  }

  if (gamepad->a())                 // A pressed
  {                                 // Hands down
    Atom.SetServoAngle(1, 180);     // Left Hand
    Atom.SetServoAngle(3, 0  );     // Right Hand
  }

  if (gamepad->b())               // B pressed
  {                               // Hands up
    hand_manual=!hand_manual;
    delay(100);
  }

  if (gamepad->x())  
  {
    // X pressed
    Atom.SetServoAngle(2, 90);    
  }

  if (gamepad->y())               // Y pressed
  {
    if (head_pos)
    {
      Atom.SetServoAngle(2, 110);
    }
    else
    {
      Atom.SetServoAngle(2, 70);
    }
    head_pos=!head_pos;
    delay(100);
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

  // Left trigger Serial.println(gamepad->brake());
  //Serial.println(gamepad->throttle());
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
  delay(50);
}