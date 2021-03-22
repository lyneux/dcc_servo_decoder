#include <DCC_Decoder.h>
#include <VarSpeedServo.h>
#include <ezButton.h>
#include <PushButton.h>
#include <EEPROM.h>
#include <BlinkControl.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Defines and structures
//
#define kDCC_INTERRUPT            0

const int MIN_SERVO_SPEED = 1;
const int MAX_SERVO_SPEED = 255;
const int MIN_SERVO_ANGLE = 0;
const int MAX_SERVO_ANGLE = 180;

const int PROGRAMMING_BUTTON_DELAY = 50; // Required in order to slow down the up/down buttons

const int MASTER_BUTTON_PIN = 3;
const int UP_BUTTON_PIN = 4;
const int DOWN_BUTTON_PIN = 5;
const int MASTER_LED_PIN = 8;

int single_blink_pattern[] = {100,1000};
int double_blink_pattern[] = {100,200,100,1000};
int triple_blink_pattern[] = {100,200,100,200,100,1000};
int quad_blink_pattern[] = {100,200,100,200,100,200,100,1000};

BlinkControl masterLed(MASTER_LED_PIN);

int current_servo = 0;  // A reference to the current servo being programmed
int target_angle = 0; // Used when programming speed (modes 3, 6, 9, 12) to hold the angle that the servo is moving to

typedef struct {
  int               address;                // Address to respond to
  byte              output;                 // State of output 1=on, 0=off
  int               outputPin;              // Arduino output pin to drive

  VarSpeedServo     servo;
  int               angle1;                  // Endpoint 1
  int               angle2;                  // Endpoint 2
  int               servoSpeed;
  bool              servoDetached;
  int               targetAngle;

} DCCAccessoryAddress;

DCCAccessoryAddress gAddresses[4];

PushButton master_button(MASTER_BUTTON_PIN);
PushButton up_button(UP_BUTTON_PIN);
PushButton down_button(DOWN_BUTTON_PIN);

boolean programming_mode = false;
int programming_step = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Factory Reset
//
void FactoryReset() {

  Serial.println("Factory Reset Starting");
  digitalWrite(MASTER_LED_PIN, HIGH);
  delay(50);
  
  EEPROM.write(0,20);
  EEPROM.write(1,90);
  EEPROM.write(2,50);
  EEPROM.write(3,20);
  
  EEPROM.write(4,20);
  EEPROM.write(5,90);
  EEPROM.write(6,50);
  EEPROM.write(7,20);
  
  EEPROM.write(8,20);
  EEPROM.write(9,90);
  EEPROM.write(10,50);
  EEPROM.write(11,20);
  
  EEPROM.write(12,20);
  EEPROM.write(13,90);
  EEPROM.write(14,50);
  EEPROM.write(15,20);

  ConfigureDecoder();
  digitalWrite(MASTER_LED_PIN, LOW);
  delay(50);
  digitalWrite(MASTER_LED_PIN, HIGH);
  delay(50);
  digitalWrite(MASTER_LED_PIN, LOW);
  delay(500);
  Serial.println("Factory Reset Complete");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Decoder Init
//
void ConfigureDecoder()
{
  gAddresses[0].address = 714;
  gAddresses[0].output = 0;
  gAddresses[0].outputPin = 9;
  gAddresses[0].angle1 = EEPROM.read(0);
  gAddresses[0].angle2 = EEPROM.read(1);
  gAddresses[0].servoSpeed = EEPROM.read(2);
  gAddresses[0].servoDetached = true;
  gAddresses[0].targetAngle = EEPROM.read(3);

  gAddresses[1].address = 715;
  gAddresses[1].output = 0;
  gAddresses[1].outputPin = 10;
  gAddresses[1].angle1 = EEPROM.read(4);
  gAddresses[1].angle2 = EEPROM.read(5);
  gAddresses[1].servoSpeed = EEPROM.read(6);
  gAddresses[1].servoDetached = true;
  gAddresses[1].targetAngle = EEPROM.read(7);

  gAddresses[2].address = 716;
  gAddresses[2].output = 0;
  gAddresses[2].outputPin = 11;
  gAddresses[2].angle1 = EEPROM.read(8);
  gAddresses[2].angle2 = EEPROM.read(9);
  gAddresses[2].servoSpeed = EEPROM.read(10);
  gAddresses[2].servoDetached = true;
  gAddresses[2].targetAngle = EEPROM.read(11);

  gAddresses[3].address = 717;
  gAddresses[3].output = 0;
  gAddresses[3].outputPin = 12;
  gAddresses[3].angle1 = EEPROM.read(12);
  gAddresses[3].angle2 = EEPROM.read(13);
  gAddresses[3].servoSpeed = EEPROM.read(14);
  gAddresses[3].servoDetached = true;
  gAddresses[3].targetAngle = EEPROM.read(15);

  // Setup output pins
  /*
  for (int i = 0; i < (int)(sizeof(gAddresses) / sizeof(gAddresses[0])); i++) {
    if ( gAddresses[i].outputPin ) {
      //pinMode( gAddresses[i].outputPin, OUTPUT );
      gAddresses[i].servo.attach(gAddresses[i].outputPin);
      gAddresses[i].servoDetached = false;
    }
  }
  */
  delay(100); // Wait 100ms for the power supply to stabilise before trying to move servos

  
  for (int i = 0; i < (int)(sizeof(gAddresses) / sizeof(gAddresses[0])); i++) {
    Serial.println("Configuring Servo " + String(i+1));
    Serial.println("Angle 1 " + String(gAddresses[0].angle1));
    Serial.println("Angle 2 " + String(gAddresses[0].angle2));
    Serial.println("Speed " + String(gAddresses[0].servoSpeed));
    Serial.println("Target Angle " + String(gAddresses[0].targetAngle));
  }

  Serial.println("Configuration complete");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Basic accessory packet handler
//
void BasicAccDecoderPacket_Handler(int address, boolean activate, byte data)
{
  // Convert NMRA packet address format to human address
  address -= 1;
  address *= 4;
  address += 1;
  address += (data & 0x06) >> 1;

  boolean enable = (data & 0x01) ? 1 : 0;

  for (int i = 0; i < (int)(sizeof(gAddresses) / sizeof(gAddresses[0])); i++) {
    if ( address == gAddresses[i].address ) {
      Serial.print("Basic addr: ");
      Serial.print(address, DEC);
      Serial.print("   activate: ");
      Serial.println(enable, DEC);

      if ( enable ) {
        gAddresses[i].targetAngle = gAddresses[i].angle1;
      } else {
        gAddresses[i].targetAngle = gAddresses[i].angle2;
      }
      EEPROM.write((i*4)+3, gAddresses[i].targetAngle); // Save the current state
      
      gAddresses[i].servo.attach(gAddresses[i].outputPin);
      gAddresses[i].servoDetached = false;
      Serial.println("Servo " + String(i+1) + " attached");
      gAddresses[i].servo.slowmove (gAddresses[i].targetAngle, gAddresses[i].servoSpeed);
      delay(50); // ensure that servo doesn't detach before moving commences
      
    }
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Setup
//
void setup() {
  // Serial setup tasks
  Serial.begin(9600);
  
  // Decoder library setup tasks
  DCC.SetBasicAccessoryDecoderPacketHandler(BasicAccDecoderPacket_Handler, true);
  ConfigureDecoder();
  DCC.SetupDecoder( 0x00, 0x00, kDCC_INTERRUPT );

  // Position all servos at saved angle when powered up
  for (int i = 0; i < (int)(sizeof(gAddresses) / sizeof(gAddresses[0])); i++) {
    Serial.println("Positioning Servo " + String(i+1) + " at angle " + String(gAddresses[i].targetAngle));
    gAddresses[i].servo.attach(gAddresses[i].outputPin);
    gAddresses[i].servoDetached = false;
    gAddresses[i].servo.slowmove (gAddresses[i].targetAngle, gAddresses[i].servoSpeed);
    
    delay(1000); // ensure that servo doesn't detach before moving commences and that servos move one by one so as not to put a large strain on the power supply
  }

  // Hardware setup tasks
  pinMode(MASTER_BUTTON_PIN, INPUT);
  pinMode(UP_BUTTON_PIN, INPUT);
  pinMode(DOWN_BUTTON_PIN, INPUT);
  pinMode(MASTER_LED_PIN, OUTPUT);
  up_button.setHoldTime(5);
  down_button.setHoldTime(5);
  up_button.setDebounceTime(0);
  down_button.setDebounceTime(0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Main loop
//
void loop() {

  // If we have a servo attached and are not in programming mode, then stop the servo when it reaches angle 1 or angle 2
  static int addr = 0;
  if( ++addr >= (int)(sizeof(gAddresses)/sizeof(gAddresses[0])) ) {
    addr = 0;
  }
  
  if (!programming_mode && !gAddresses[addr].servoDetached && ((gAddresses[addr].servo.read() == gAddresses[addr].angle1) || (gAddresses[addr].servo.read() == gAddresses[addr].angle2))) {
    // Servo can stop moving
    gAddresses[addr].servo.detach();
    gAddresses[addr].servoDetached = true;
    Serial.println("Servo " + String(addr+1) + " detached");
  }

  // Delegate to various objects that need to loop or update:
  DCC.loop();
  masterLed.loop();
  master_button.update();
  up_button.update();
  down_button.update();


  // Perform a factory reset with the right button combination
  if (up_button.isActive() && down_button.isActive()) {
    FactoryReset();
  }

  /**************** Enter programming mode ****************/
  if (!programming_mode && master_button.isHeld()) {
    programming_mode = true;
    
    masterLed.blink(single_blink_pattern, sizeof(single_blink_pattern)/sizeof(int));
    //masterLed.blink1();
    
    programming_step = 1;
    current_servo = 0;
    Serial.println("Programming Mode Activated, Programming Step 1, Attaching all servos");
    gAddresses[0].servo.attach(gAddresses[0].outputPin);
    gAddresses[1].servo.attach(gAddresses[1].outputPin);
    gAddresses[2].servo.attach(gAddresses[2].outputPin);
    gAddresses[3].servo.attach(gAddresses[3].outputPin);
    gAddresses[0].servoDetached = false;
    gAddresses[1].servoDetached = false;
    gAddresses[2].servoDetached = false;
    gAddresses[3].servoDetached = false;
  }

  /**************** Step through to the next programming step ****************/
  if (programming_mode && master_button.isClicked()) {
    programming_step++;
    Serial.println("Programming Step Increased to " + String(programming_step));
    
    if (programming_step == 3 || programming_step == 6 || programming_step == 9 || programming_step == 12) {
      target_angle = gAddresses[current_servo].angle1;
      gAddresses[current_servo].servo.slowmove (target_angle, gAddresses[current_servo].servoSpeed);
      Serial.println("Slow moving to = " + String(target_angle));
    }

    if (programming_step == 4) {
      current_servo++;
      masterLed.blink(double_blink_pattern, sizeof(double_blink_pattern)/sizeof(int));
      //masterLed.blink2();
    }

    if (programming_step == 7) {
      current_servo++;
      masterLed.blink(triple_blink_pattern, sizeof(triple_blink_pattern)/sizeof(int));
      //masterLed.blink3();
    }

    if (programming_step == 10) {
      current_servo++;
      masterLed.blink(quad_blink_pattern, sizeof(quad_blink_pattern)/sizeof(int));
      //masterLed.blink4();
    }
    
  }

  /**************** Set First Angle ****************/
  if (programming_mode && (programming_step == 1 || programming_step == 4 || programming_step == 7 || programming_step == 10)) {
    
    gAddresses[current_servo].servo.slowmove (gAddresses[current_servo].angle1, gAddresses[current_servo].servoSpeed);

    if (up_button.isActive() && gAddresses[current_servo].angle1 < MAX_SERVO_ANGLE) {
      gAddresses[current_servo].angle1++;
      SaveValue(current_servo * 4,gAddresses[current_servo].angle1);
    }
    if (down_button.isActive() && gAddresses[current_servo].angle1 > MIN_SERVO_ANGLE) {
      gAddresses[current_servo].angle1--;  
      SaveValue(current_servo * 4,gAddresses[current_servo].angle1);
    }
    
  }

  /**************** Set Second Angle ****************/
  if (programming_mode && (programming_step == 2 || programming_step == 5 || programming_step == 8 || programming_step == 11)) {

    gAddresses[current_servo].servo.slowmove (gAddresses[current_servo].angle2, gAddresses[current_servo].servoSpeed);
    
    if (up_button.isActive()  && gAddresses[current_servo].angle2 < MAX_SERVO_ANGLE) {
      gAddresses[current_servo].angle2++;
      SaveValue((current_servo * 4) + 1,gAddresses[current_servo].angle2);
    }
    if (down_button.isActive() && gAddresses[current_servo].angle2 > MIN_SERVO_ANGLE) {
      gAddresses[current_servo].angle2--;
      SaveValue((current_servo * 4) + 1,gAddresses[current_servo].angle2);
    }
    
  }

  /**************** Set Speed ****************/
  if (programming_mode && (programming_step == 3 || programming_step == 6 || programming_step == 9 || programming_step == 12)) {
    
    if (gAddresses[current_servo].servo.read() == target_angle && target_angle == gAddresses[current_servo].angle1) {
      target_angle = gAddresses[current_servo].angle2;
      gAddresses[current_servo].servo.slowmove (target_angle, gAddresses[current_servo].servoSpeed);
      Serial.println("Slow moving to = " + String(target_angle));
    }
    if (gAddresses[current_servo].servo.read() == target_angle && target_angle == gAddresses[current_servo].angle2) {
      target_angle = gAddresses[current_servo].angle1;
      gAddresses[current_servo].servo.slowmove (target_angle, gAddresses[current_servo].servoSpeed);
      Serial.println("Slow moving to = " + String(target_angle));
    }

    if (up_button.isActive() && gAddresses[current_servo].servoSpeed < MAX_SERVO_SPEED) {
      gAddresses[current_servo].servoSpeed++;
      SaveValue((current_servo * 4) + 2,gAddresses[current_servo].servoSpeed);
      gAddresses[current_servo].servo.slowmove (target_angle, gAddresses[current_servo].servoSpeed);
      Serial.println("Speed increased to  = " + String(gAddresses[current_servo].servoSpeed));
    }
    if (down_button.isActive() && gAddresses[current_servo].servoSpeed > MIN_SERVO_SPEED) {
      gAddresses[current_servo].servoSpeed--;
      SaveValue((current_servo * 4) + 2,gAddresses[current_servo].servoSpeed);
      gAddresses[current_servo].servo.slowmove (target_angle, gAddresses[current_servo].servoSpeed);
      Serial.println("Speed decreased to  = " + String(gAddresses[current_servo].servoSpeed));
    }
  }

  if (programming_step == 13) {
    Serial.println("Programming Step 13");
    programming_step = 0;
    programming_mode = false;
    masterLed.pause();
    Serial.println("Completed Programming Step 13");
  }
  
}

void SaveValue(int address, int value) {
  EEPROM.write(address,value);
  delay(PROGRAMMING_BUTTON_DELAY);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
