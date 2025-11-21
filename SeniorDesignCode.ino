/*
11/04/2025
Titus Smith
Team Gerbil F25 S5110
Colorado School of Mines

Description:
This code is responsible for running a hydro-
electric turbine for Axxess energy.

The code is responsible for starting up the turbine
and disconnecting it if infavorable circumstances are
encountered (over-voltage, failure to reach voltage).
*/

// OLED display libraries
#include <U8x8lib.h>
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8( U8X8_PIN_NONE, U8X8_PIN_NONE, U8X8_PIN_NONE);

//Button definitions
const int estop = 3;
const int g_button = 2;
const int o_button = 4;

//voltage sensor definition
const int voltage_sense = A0;
float measured_voltage = 0.0f;

//Current sensor definitions
const int dump_relay_current_pin = A2;
float dump_relay_current = 0.0f;
const int inverter_current_pin = A3;
float inverter_current = 0.0f;

//Linear actuator controlls
const int motor_dir1 = 5;
const int motor_dir2 = 6;

//dump relay controlls
const int dump_relay = 7;
const int cap_relay = 8;

//Threashholds 
const float voltage_threashold = 350.0f; //shutoff threashhold
const float conversion_factor = 0.4177f;
//voltage to shut off dump load (not currently implemented)
//dump load was implemented as a toggle button currently
//const float stable_voltage = 320.0f; // voltage of stable running

//bool flags to handle state machine
bool engaged = false;
bool motor_running = false;
bool dump_enabled = true;

//motor movement timer
unsigned long setpoint_time = 0;
unsigned long timeout = 4000;//variable used by update_motor DONT CHANGE
const unsigned int timeout_val = 4000;//update this variable instead

//startup voltage timeout
unsigned long startup_timer = 0;
const unsigned long startup_timeout = 1*60*1000;//wait one minutes to spin up

//Restart definitions (currently disabled to prevent auto-restart)
unsigned long restart_timer = 0;
const unsigned long restart_timeout = 500*2*60*60*1000;//wait 2 hours to try restart
const bool try_restart = false;

void setup() {
  Serial.begin(9600);

  //initialize display
  u8x8.begin();
  u8x8.setPowerSave(0); // Turn off power saving mode
  u8x8.setFont(u8x8_font_chroma48medium8_r); // A common default font
  u8x8.clearDisplay();
  update_display(0.0,0.0,0.0);

  //pin setups
  pinMode(estop,INPUT_PULLUP);
  pinMode(g_button,INPUT_PULLUP);
  pinMode(o_button,INPUT_PULLUP);
  pinMode(voltage_sense,INPUT);

  pinMode(dump_relay_current_pin,INPUT);
  pinMode(inverter_current_pin,INPUT);

  pinMode(motor_dir1,OUTPUT);
  pinMode(motor_dir2,OUTPUT);

  pinMode(dump_relay,OUTPUT);
  pinMode(cap_relay,OUTPUT);

  //turbine initialization
  Serial.println("Welcome to Gerbil Turbine\nInitializing Turbine...");
  delay(3000);
  //dump relay on and caps off and turn on brake
  digitalWrite(dump_relay,HIGH);
  digitalWrite(cap_relay,LOW);
  motor_disengage();
  //wait for brake to fully engage
  while(motor_update());
}

void loop() {
  //measure voltage and current
  measured_voltage = conversion_factor*analogRead(voltage_sense);
  dump_relay_current = abs((analogRead(dump_relay_current_pin)*5.0/1023)-2.5)/0.0667;
  inverter_current = abs((analogRead(inverter_current_pin)*5.0/1023)-2.5)/0.0667;

  //Check Estop state
  if(digitalRead(estop)==LOW && engaged==true){
    //the Estop has been pressed
    //disengage
    engaged=false;
    restart_timer = millis();
    //dump load on caps off brake on
    digitalWrite(dump_relay,HIGH);
    digitalWrite(cap_relay,LOW);
    motor_disengage();
    //set timeout for motor movement
    timeout = timeout_val+750;
    Serial.println("Estop pressed");
    return;
  }

  //over volt protection
  if (measured_voltage > voltage_threashold){
    //we have sensed over voltage
    delay(1000);
    //wait 1 second and read again to make sure it wasnt a fluke
    measured_voltage = conversion_factor*analogRead(voltage_sense);
    if (measured_voltage > voltage_threashold){
      //Trigger disconnect
      digitalWrite(dump_relay,HIGH);
      digitalWrite(cap_relay,LOW);
      engaged=false;
      motor_disengage();
      timeout = timeout_val+750;
      Serial.print("Voltage Limit Exceeded: ");
      Serial.println(measured_voltage);
    }
  }

  //engage code
  if(digitalRead(g_button)==LOW && engaged==false){
    //if the green button is pressed, try to engage
    try_startup_sequence();
  }

  //dump load toggle button
  if(digitalRead(o_button)==LOW){
    //toggle the dump load state
    dump_enabled = !dump_enabled;
    digitalWrite(dump_relay,dump_enabled);
    delay(300);
  }

  //Restart code (unused)
  /*
  if((try_restart && !engaged) && (unsigned long)(millis()-restart_timer) > restart_timeout){
    try_startup_sequence();
  }
  */

  //update the motor to check if it needs to turn off
  motor_update();
  //print current state to display
  update_display(measured_voltage,inverter_current,dump_relay_current);
}

//function to try starting up the generator
void try_startup_sequence(){
  //measure state of generator current and voltage
  measured_voltage = conversion_factor*analogRead(voltage_sense);
  dump_relay_current = abs((analogRead(dump_relay_current_pin)*5.0/1023)-2.5)/0.0667;
  inverter_current = abs((analogRead(inverter_current_pin)*5.0/1023)-2.5)/0.0667;

  //cancel start if estop pressed
  if(digitalRead(estop)==LOW){
    Serial.println("Estop pressed");
    //delay(200);
    engaged = false;
    restart_timer = millis();
    return;
  }
  //else try to turn on the generator
  engaged=true;
  digitalWrite(dump_relay,HIGH);
  digitalWrite(cap_relay,HIGH);
  motor_engage();//turn off the brake
  timeout = timeout_val;
  Serial.println("Starting Motor...");
  startup_timer = millis();
  /*
  //wait till voltages stabilize (unused stabilization code)
  while(measured_voltage < stable_voltage && debug != true){
    measured_voltage = conversion_factor*analogRead(voltage_sense);
    dump_relay_current = abs((analogRead(dump_relay_current_pin)*5.0/1023)-2.5)/0.0667;
    inverter_current = abs((analogRead(inverter_current_pin)*5.0/1023)-2.5)/0.0667;

    //Exit if we have estop engaged
    if(digitalRead(estop)==LOW){
      digitalWrite(dump_relay,HIGH);
      digitalWrite(cap_relay,LOW);
      motor_disengage();
      timeout=timeout_val+750;
      engaged=false;
      return;
    }
    if((unsigned long) (millis()-startup_timer) > startup_timeout){
      //we have gone 90 seconds without finding a stable voltage so shutdown.
      engaged = false;
      digitalWrite(dump_relay,HIGH);
      digitalWrite(cap_relay,LOW);
      restart_timer = millis();
      motor_disengage();
      timeout=timeout_val+750;
      Serial.println("Voltage not reached in time limit, shutting down");
      //while(motor_update());
      return;
    }
    motor_update();
    update_display(measured_voltage,inverter_current,dump_relay_current);
  }
  Serial.println("Voltage reached... engaging load");
  */

  //wait for inverter to take current (not currently enabled)
  /*
  while(inverter_current < inverter_current_threashold){
    measured_voltage = conversion_factor*analogRead(voltage_sense);
    dump_relay_current = abs((analogRead(dump_relay_current_pin)*5.0/1023)-2.5)/0.0667;
    inverter_current = abs((analogRead(inverter_current_pin)*5.0/1023)-2.5)/0.0667;

    //Exit if we have estop engaged
    if(digitalRead(estop)==LOW){
      digitalWrite(dump_relay,HIGH);
      digitalWrite(cap_relay,LOW);
      motor_disengage();
      engaged=false;
      timeout=timeout_val+750;
      return;
    }

    //over volt protection
    if (measured_voltage > voltage_threashold){
      //Trigger disconnect
      digitalWrite(dump_relay,HIGH);
      digitalWrite(cap_relay,LOW);
      engaged=false;
      motor_disengage();
      timeout = timeout_val+750;
      Serial.print("Voltage Limit Exceeded: ");
      Serial.println(measured_voltage);
      //while(motor_update());
      return;
    }
    motor_update();
    update_display(measured_voltage,inverter_current,dump_relay_current);

  }
  */
  //presumably the inverter is taking power now, so...
  //digitalWrite(dump_relay,LOW);

  Serial.println("Load started");
}

void motor_disengage(){
  motor_retract();
  setpoint_time = millis();
}

void motor_engage(){
  motor_forward();
  setpoint_time = millis();
}

//functino to check if the motor should stop moving
bool motor_update(){
  if(motor_running && (unsigned long) (millis()-setpoint_time) > timeout){
    motor_halt();
  }
  return motor_running;
}

void motor_retract(){
  digitalWrite(motor_dir1,HIGH);
  digitalWrite(motor_dir2,LOW);
  motor_running = true;
}
void motor_forward(){
  digitalWrite(motor_dir1,LOW);
  digitalWrite(motor_dir2,HIGH);
  motor_running = true;
}
void motor_halt(){
  digitalWrite(motor_dir1,LOW);
  digitalWrite(motor_dir2,LOW);
  motor_running = false;
}

//function to print cucrent statistics to display
void update_display(float voltage, float inverter_current, float dump_relay_current) {
  
  // Clear the display
  //disabled bc of bad refresh rate
  //u8x8.clearDisplay();
  
  // 1. Display the current voltage (Row 0)
  u8x8.setCursor(0, 0); // Start at column 0, row 0
  u8x8.print("VOLTAGE:"); 
  
  // Voltage is too large to fit in 1 line, let's use the next line
  u8x8.setCursor(0, 1);
  u8x8.print(voltage, 1); // Print with 1 decimal place
  u8x8.print(" V   ");

  // 2. Display the current state/status (Row 3, 4, 5)
  u8x8.setCursor(0, 3);
  
  if (digitalRead(estop) == LOW) {
    u8x8.setInverseFont(1); // Invert text for E-STOP
    u8x8.print("!!! E-STOP !!!     ");
    u8x8.setInverseFont(0); // Turn inverse off
    u8x8.setCursor(0, 4);
    u8x8.print("SYSTEM OFFLINE     ");
  } else if (voltage > voltage_threashold) {
    u8x8.print("CRITICAL:     ");
    u8x8.setCursor(0, 4);
    u8x8.print("OVER VOLTAGE     ");
  } else if (!engaged) {
    u8x8.print("STATUS: OFFLINE   ");
    u8x8.setCursor(0, 4);
    
    if ((unsigned long)(millis()-restart_timer) < restart_timeout && restart_timer != 0 && try_restart) {
        // Calculate remaining time for display
        unsigned long time_remaining = (restart_timeout - (millis() - restart_timer)) / 1000;
        u8x8.print("RESTART IN: ");
        u8x8.print(time_remaining / 60);
        u8x8.print("m    ");
    } else {
        u8x8.print("Ready to Engage    ");
    }
  } else {
    // Engaged Status
    u8x8.print("STATUS: ENGAGED    ");
    u8x8.setCursor(0, 4);
    u8x8.print("LOAD: ");
    u8x8.print(digitalRead(dump_relay) == LOW ? "ACTIVE   " : "OFF   ");
  }
  
  // 3. Display motor status (Row 7, bottom of screen)
  u8x8.setCursor(0, 7); 
  u8x8.print("Motor: ");
  if (motor_running) {
    u8x8.print(digitalRead(motor_dir1) == HIGH ? "RETRACTING    " : "ENGAGING    ");
  } else {
    u8x8.print("HALTED    ");
  }

  // 4. Display Currents
  u8x8.setCursor(0, 5); // Start at column 0, row 5
  u8x8.print("INV CUR: ");
  u8x8.print(inverter_current, 1); // Print with 1 decimal place
  u8x8.print(" A    ");

  u8x8.setCursor(0, 6); // Start at column 0, row 6
  u8x8.print("DUMP CUR: ");
  u8x8.print(dump_relay_current, 1); // Print with 1 decimal place
  u8x8.print(" A    "); 
  // NOTE: U8x8 mode displays immediately, there is no separate display.display() call.
}