#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
#define SW_PIN 2
#define VRX_PIN A0
#define VRY_PIN A1

#include <AccelStepper.h>
// https://www.makerguides.com/a4988-stepper-motor-driver-arduino-tutorial/
// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define stepPin 6
#define dirPin 5
#define motorInterfaceType 1
// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
//AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

const unsigned long Motor_Pulse = 6400;
const unsigned long Motor_Pulse_UpDown = 1600; //3200
int Motor_Speed;
const unsigned long Motor_Max_Speed = 6400; //128000
int Motor_rev = 16;

//HX711 12 13
#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 4;
HX711 scale;
const int AH3505Pin = A3;

enum {
  RUN = 0,      //Being run
  PAUSE,    //Get new settings
  SPEEDCHOOSE
} ControlState;
enum {
  SHOWUP,
  SHOWDOWN,
  CLEARSHOWUPDOWN ,   //Get new settings
  ISRUNNING,
  COMPRESSIONSPEED
} DisplayState;
enum {
  MoveUp,      //Being run
  MoveDown,    //Get new settings
  MoveLeft,
  MoveRight,
  Nothing
} MyJoystick;
const int EN = 13; //Mới thêm
#include <avr/io.h>
#include <avr/interrupt.h>
#include <EEPROM.h>
#include <millisDelay.h>
const unsigned long DISPLAYING_TIME = 500; // in mS
millisDelay DisplayingDelay; // the delay object
const unsigned long READINGSPEED_TIME = 1000; // in mS
millisDelay ReadingSpeedDelay; // the delay object

//HẢI đọc xem để hiệu chỉnh tham số cho cân 1kG - 2023-05-06
long Zero_scale = 119163, One100g_scale; //value at 0 kg and 100 gr
float the_scale = 880.5f;                 //kg
//Scale value: 880.5 Offset value: 119163


float the_force, force_scale;
const boolean CALIB = false;
/*
  Calibration process!
  Put nothing on scale.
  Put 100g weight on scale.
  Calibration process is DONE!
  Scale value: 1304.7 Offset value: 68729 */

#include<ButtonV2.h>
ButtonV2 button;

//Các hằng số cho HX711 và cảm biến Hall
float Voffset = 2.5;
float Sensitivity = 2.5;
float calibration_factor = -100525;

void setup() {
  lcd.init(); // initialize the lcd
  lcd.backlight(); //Back light is on
  //https://www.engineersgarage.com/articles-arduino-16x2-character-lcd-generating-custom-characters-icons/
  byte c1[8] = {B00100, B01110, B11111, B00100, B00100, B00100, B00100, B00100,}; //Up-Arrow
  byte c2[8] = {B00100, B00100, B00100, B00100, B00100, B11111, B01110, B00100,}; //Down-Arrow
  lcd.createChar(0 , c1);   //Creating custom characters in CG-RAM
  lcd.createChar(1 , c2);


  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(VRX_PIN, INPUT);
  pinMode(VRY_PIN, INPUT);
  pinMode(7, OUTPUT); //Sleep
  pinMode(8, OUTPUT); //Reset
  pinMode(10, OUTPUT); //M0 DRV8825
  pinMode(11, OUTPUT); //M1
  pinMode(12, OUTPUT); //M2
  pinMode(13, OUTPUT); //EN enable
  digitalWrite(10, HIGH);// Chia 8 - Nếu động cơ ì ì ko quay,
  digitalWrite(11, HIGH);//  chỉnh chiết áp công suất để nó quay đều
  digitalWrite(12, LOW);

  digitalWrite(13, LOW); //Enable LOW - Cannot work; HIGH - Can work
  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
  /*
    //Đọc vào các hằng số cho HX711 và cảm biến Hall
    int addr;
    addr = 0;
    EEPROM.get(addr, Voffset);
    addr += sizeof(Voffset);
    EEPROM.get(addr, Sensitivity);
    addr += sizeof(Sensitivity);
    EEPROM.get(addr, calibration_factor);
  */
  pinMode(AH3505Pin, INPUT_PULLUP); //Hall sensor AH3505
  Serial.begin(57600);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  lcd.setCursor(0, 0); lcd.print("Magnetic  Mesurement");
  lcd.setCursor(0, 1); lcd.print("   Reduction test   ");
  lcd.setCursor(0, 2); lcd.print("by   Green Team     ");
  lcd.setCursor(0, 3); lcd.print(".....          .....");
  delay(2000);
  // Set the maximum speed in steps per second:
  stepper.setMaxSpeed(Motor_Max_Speed);
  ControlState = PAUSE;
  SetUpInterrupts(500); //500 usecs
  //Show display setting
  DisplayingDelay.start(DISPLAYING_TIME);
  ReadingSpeedDelay.start(READINGSPEED_TIME);
  Serial.println("Loadcell is checking.");
  while (!scale.is_ready()) {
    delay(1000);
  }
  Serial.println("Ok....done");

  button.SetStateAndTime(LOW, 500);
  //byte type = button.CheckButton(SW_PIN);

  if (CALIB) {
    Serial.println("Calibration process!");
    Serial.println("Put nothing on scale.");
    delay(5000);
    Zero_scale = scale.read_average(5);
    Serial.println("Put 100g weight on scale.");
    delay(10000);
    One100g_scale = scale.read_average(5);
    Serial.println("Calibration process is DONE!");
    Serial.print("Scale value: "); Serial.print((float) (One100g_scale - Zero_scale) / 100.0, 1);
    Serial.print(" Offset value: "); Serial.println(Zero_scale);
    the_scale = (float) (One100g_scale - Zero_scale) / 100.0;
    delay(10000);
  }

  scale.set_scale(the_scale);
  scale.set_offset(Zero_scale);
  delay(3000);
  the_force = 0.f;

}
long MyJoystickRead, times;
boolean flag_Move = false;
byte Compression_speed = 5;

float Hall_reading() {
  //https://www.ahest.net/linear-hall-sensor/942.html
  int sensorValue = analogRead(AH3505Pin);
  float Vin = sensorValue * (5.0 / 1024);
  float B = (Vin - Voffset) / (Sensitivity * 0.001);
  return B;
}

void loop() {

  IsPressed(SW_PIN);
  MyJoystickUpDown();
  switch (ControlState) {
    case SPEEDCHOOSE:
      //Serial.println("Compression speed choose.");
      switch (MyJoystick) {
        case MoveUp:
          //case MoveRight:
          if (ReadingSpeedDelay.justFinished()) {
            // toggle led and repeat
            ReadingSpeedDelay.repeat(); // repeat
            Compression_speed++;
            if (Compression_speed > 10) Compression_speed = 10;
          }
          break;
        case MoveDown:
          //case MoveLeft:
          if (ReadingSpeedDelay.justFinished()) {
            // toggle led and repeat
            ReadingSpeedDelay.repeat(); // repeat
            Compression_speed--;
            if (Compression_speed < 1) Compression_speed = 1;
          }
          break;
        case Nothing:
          Motor_Speed = 0;
          DisplayState = COMPRESSIONSPEED;
          stepper.setSpeed(Motor_Speed);
          break;
      }
      break;
    case PAUSE:
      //Move up down by hand
      //Up
      switch (MyJoystick) {
        case MoveUp:
          if (DisplayState != SHOWUP) DisplayState = SHOWUP;
          Motor_Speed = Motor_Pulse_UpDown;
          stepper.setSpeed(-Motor_Speed);
          //lcd.setCursor(6, 3); lcd.print(the_force, 3); lcd.print(" kgf");
          break;
        case MoveDown:
          if (DisplayState != SHOWDOWN) DisplayState = SHOWDOWN;
          Motor_Speed = Motor_Pulse_UpDown;
          stepper.setSpeed(Motor_Speed);
          break;
        case MoveLeft:
        case MoveRight:
        case Nothing:
          if (DisplayState != CLEARSHOWUPDOWN) DisplayState = CLEARSHOWUPDOWN;
          Motor_Speed = 0;
          stepper.setSpeed(Motor_Speed);
          break;
      }
      break;
    case RUN:
      DisplayState = ISRUNNING;
      //1 mm / minute = Motor_Speed = 6400 * 1 mm / 8 mm / 60 = 13 pulse
      //2 mm / minute = Motor_Speed = 6400 * 2 mm / 8 mm / 60 = 26 pulse
      //3 mm / minute = Motor_Speed = 6400 * 3 mm / 8 mm / 60 = 40 pulse
      //4 mm / minute = Motor_Speed = 6400 * 4 mm / 8 mm / 60 = 53 pulse
      //5 mm / minute = Motor_Speed = 6400 * 5 mm / 8 mm / 60 = 66 pulse
      Motor_Speed = 320; //(int) Compression_speed * Motor_Pulse / 8.f / 60.f;
      stepper.setSpeed(-Motor_Speed); //Move UP
      force_scale = scale.get_units(1) / 1000.f;
      if (the_force < force_scale) the_force = force_scale;
      //
      //if (((the_force > 0.5f) && ((the_force - force_scale) >= (0.2f * the_force))) || (force_scale > 9.79f)) {
      //  ControlState = PAUSE;
      //  Serial.println("Compression is stopped");
      //;
      //}
      //lcd.setCursor(6, 3); lcd.print(the_force, 3); lcd.print(" kgf");
      //Serial.print((millis() - times) / 1000.f, 1); Serial.print("\t"); Serial.print(force_scale, 3); Serial.println("\tkgf.\t");
      break;
  }
  if (DisplayingDelay.justFinished()) {
    // toggle led and repeat
    DisplayingDelay.repeat(); // repeat
    Display_Info();
    Serial.print("Magnetic strength: ");
    Serial.print(Hall_reading()); Serial.println(" Gass");
  }
}

long lastDebounceSWTime = 0;
const int debounceSW_time = 2000;
void IsPressed(int SWpin) {
  byte type = button.CheckButton(SW_PIN);
  switch (type)
  {
    case WAITING:
      break;
    case PRESSED:
      //Serial.println("pressed 1 time");
      if (ControlState == PAUSE) {
        ControlState = RUN;
        the_force = 0.f; times = 0;
        lcd.setCursor(0, 3); lcd.print(".....          .....");
        Serial.println("ControlState....RUN");
        Serial.println("Time(sec.)\tForce\tUnit");
      } else {
        ControlState = PAUSE;
        lcd.setCursor(0, 3); lcd.print(".....          .....");
        Serial.println("ControlState....PAUSE");
      }
      break;
    case DOUBLE_PRESSED:
      //Serial.println("pressed 2 times");
      break;
    case MULTI_PRESSED:
      //Serial.println("pressed 3 times");
      break;
    case HELD:
      //Serial.println("Button HELD");
      ControlState = SPEEDCHOOSE;
      break;
  }
}

char displayString[40] = "";

void Display_Info() {
  char myChar;
  String gf;
  byte num_dec;
  char buffer[20];
  float magnetic_B;
  String unit_G;
  force_scale = scale.get_units(1);
  if (force_scale >= 100)  {
    num_dec = 1;
    gf = " gf ";
  }
  else {
    num_dec = 2;
    gf = "  gf ";
  }
  magnetic_B = Hall_reading();
  if (magnetic_B > 100) {
    unit_G = " Gauss";
  } else if (magnetic_B > 10) {
    unit_G = "  Gauss";
  } else {
    unit_G = "  Gauss";
  }

  switch (ControlState) {
    case PAUSE:
      switch (MyJoystick) {
        case MoveUp:
          break;
        case MoveDown:
          break;
        case MoveLeft:
        case MoveRight:
        case Nothing:
          break;
      }
      break;
    case RUN:
      break;
  }
  switch (DisplayState) {
    case SHOWUP:
      //sprintf(displayString, "Moving up......     ");
      //sprintf(displayString,   "         Moving up  ");

      lcd.setCursor(0, 2); lcd.print(char(0)); lcd.print(" "); lcd.print(force_scale, num_dec); lcd.print(gf);  lcd.print(magnetic_B, 0); lcd.print(unit_G);
      lcd.setCursor(0, 1); lcd.print("   Reduction test   ");

      break;
    case SHOWDOWN:
      //sprintf(displayString, "Moving up......     ");
      //sprintf(displayString,   " Moving down        ");
      //lcd.setCursor(0, 2); lcd.print(char(1));lcd.print("                   ");
      lcd.setCursor(0, 2); lcd.print(char(1)); lcd.print(" "); lcd.print(force_scale, num_dec); lcd.print(gf);  lcd.print(magnetic_B, 0); lcd.print(unit_G);
      lcd.setCursor(0, 1); lcd.print("   Reduction test   ");
      break;
    case CLEARSHOWUPDOWN:
      //sprintf(displayString, "Moving up......     ");
      //sprintf(displayString,   "    Press to run    ");
      lcd.setCursor(0, 2); lcd.print("  "); lcd.print(force_scale, num_dec); lcd.print(gf);  lcd.print(magnetic_B, 0); lcd.print(unit_G);
      lcd.setCursor(0, 1); lcd.print("   Reduction test   ");
      break;
    case ISRUNNING:
      sprintf(displayString,   "        Moving up "); //"Force is applying...
      lcd.setCursor(0, 1); lcd.print(char(0)); lcd.print(displayString); lcd.print(char(0));
      lcd.setCursor(0, 2); lcd.print("  "); lcd.print(force_scale, num_dec); lcd.print(gf);  lcd.print(magnetic_B, 0); lcd.print(unit_G);
      break;
    case COMPRESSIONSPEED:
      sprintf(displayString,   " Compression speed  ");
      lcd.setCursor(0, 2); lcd.print(displayString);
      lcd.setCursor(6, 3); lcd.print(Compression_speed); lcd.print(" mm/min.");
      break;
  }
}

// when you did not use a button set the value to zero
#define CONTROL_analog_up_min 600 // Button Up
#define CONTROL_analog_up_max 1024
#define CONTROL_analog_down_min  0// Button Down
#define CONTROL_analog_down_max 400
#define CONTROL_analog_left_min 0 // Button Left
#define CONTROL_analog_left_max 1600
#define CONTROL_analog_right_min 2500 // Button Right
#define CONTROL_analog_right_max 4095
void MyJoystickUpDown() {
  //Run for First time //2023-05-05
  int valuex = analogRead(VRX_PIN); // doc gia tri cua truc X
  int valuey = analogRead(VRY_PIN); // doc gia tri cua truc Y
  MyJoystick = Nothing;
  if (valuey >= CONTROL_analog_up_min && valuey <= CONTROL_analog_up_max) MyJoystick = MoveUp;
  if (valuey >= CONTROL_analog_down_min && valuey <= CONTROL_analog_down_max) MyJoystick = MoveDown;  // down
  //if (valuey >= CONTROL_analog_left_min && valuey <= CONTROL_analog_left_max) MyJoystick = MoveLeft;  // left
  //if (valuey >= CONTROL_analog_right_min && valuey <= CONTROL_analog_right_max) MyJoystick = MoveRight;  // right

  //if ((x > 3000) && ((y > 1850) && (y < 2400))) MyJoystick = MoveUp;
  //else if ((x < 1000) && ((y > 1850) && (y < 2400))) MyJoystick = MoveDown;
  //else MyJoystick = Nothing;
}
/*****************************************************************************
** SetUpInterrupts
** ===============
  Set up interrupt routine to service stepper motor run() function.
  from: https://groups.google.com/forum/#!topic/accelstepper/FIct-aiBk3o
*/
bool SetUpInterrupts(const int usecs)
{
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B

  // set compare match register to desired timer count (1ms):
  // ATmega328 with a 16MHz clock, clk/8
  // (# timer counts + 1) = (target time) / (timer resolution)
  //                      =     .0001s      /   6.25e-8 s  * 8
  //                      =   200
  const float targetSecs = ((float) usecs) / 1e6;
  const float timerSteps = 6.25e-8;                //    1/16MHz
  int count = 0;
  int prescale = 1;  // valid values: 1, 8, 64, 256, 1024
  do  {
    count = targetSecs / (timerSteps * prescale);
    if (count < 65535) // Timer 1 is 16-bits wide
      break;
    prescale *= 8;
  } while (prescale <= 1024);
  if (prescale > 1024)                // time too long
    return false;
  if (prescale == 1 && count < 100)   // time too short
    return false;

  OCR1A = count;         // Eg, 200 = 0.1ms - I found 1ms gives rough acceleration
  // turn on CTC mode (Clear Timer on Compare Match):
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  // CS12   CS11   CS10
  //   0      0      0   no clock source, Timer/counter stopped
  //   0      0      1   clk/1  no prescaling
  //   0      1      0   clk/8
  //   0      1      1   clk/64
  //   1      0      0   clk/256
  //   1      0      1   clk/1024
  //   1      1      0   external clock on T1 pin, falling edge
  //   1      1      1   external clock on T1 pin, rising edge
  switch (prescale)  {
    case 1:
      TCCR1B |= (1 << CS10);                   // 0 0 1
      break;
    case 8:
      TCCR1B |= (1 << CS11);                   // 0 1 0
      break;
    case 64:
      TCCR1B |= (1 << CS11) & (1 << CS10);     // 0 1 1
      break;
    case 256:
      TCCR1B |= (1 << CS12);                   // 1 0 0
      break;
    case 1024:
      TCCR1B |= (1 << CS12) & (1 << CS10);     // 1 0 1
      break;
  }
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  // enable global interrupts:
  sei();

  return true;
}

/*****************************************************************************
** ISR
** ===
  Interrupt service routine for when Timer 1 matches compare value
*/
ISR(TIMER1_COMPA_vect)
{
  stepper.runSpeed();
}
/****************************************************************************/

