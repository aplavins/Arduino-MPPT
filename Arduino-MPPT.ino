//----------------------------------------------------------------------------------------------------
//
//  Author: Adam Plavinskis
//          aplavins@gmail.com
//
//  This code is for an arduino UNO based Solar MPPT charge controller.
//  It is based of work done by Julian Ilett (256.co.uk), Debasish Dutta/deba168,
//  and Tim Nolan (www.timnolan.com).
//
//  Updated to use intergers instead of floats. The math is faster and more accurate.
//  All units are in deci Volts
//
//  This is an open source project and you are free to use any code you like.
//  Please give credit.
//
//  Improvements in this version:
//  - using 2104 MOSFET driver for high side drive
//  - synchronous operation with high and low side MOSFETS for better buck converter efficiency
//  - dual high side MOSFETS arranged back to back to prevent current leakage in low light
//  - arduino protection with zener diodes
//  - analog input filtering for more accurate measurments
//  - eliminated absorb state for simpler operation
//  - eliminated ammeter to reduce cost
//
//  Specs:
//  - 250W input capacity (with heatsinks and fan)
//  - 50V input max
//  - 20V output max
//  - 30A output max  (with heatsinks and fan)
//  - 30A load capacity (with heatsinks and fan)
//
//  Warning!:
//  -Disconnecting the battery while in a charging state will cause and overshoot of voltage on the battery side.
//  This could damage any loads that are running from the battery, including the arduino, charge controller, 
//  and computer (if it's connected at the time)
//  -Setting the pulseWidth to values less than 30% for even a few milliseconds will cause the low side MOSFET
//  to short out and fail (sometimes violently).
//
//----------------------------------------------------------------------------------------------------


#include <PWM.h>

// Wiring:
// A0 - Voltage divider (solar)
// A1 - Voltage divider (battery)
// D4 - Load Mosfet
// D3 - 2104 MOSFET driver IN    (needs to be 3 or 11 with safe timer)
// D5 - 2104 MOSFET driver SD
// D6 - fan

#define panelMeter A0         // Analog input pin for PV voltmeter
#define batteryMeter A1       // Analog input pin for battery voltmeter
#define driver 3              // PWM output pin to mosfet driver (needs to be 3 or 11 with safe timer)
#define shutDown 5            // connected to shutdown pin of mosfet driver (needs to be a PWM pin)
#define fan 6                 // digital pin controlling the fan (needs to be a PWM pin)
#define load 4                // digital pin controlling the load MOSFET (use 4 because PWM is not needed)
#define Vbulk 130             // Bulk voltage set-point (deci Volts)
#define lowBatt 110           // Low battery voltage set-point (deci Volts)
#define Vmax 150              // Maximum voltage the batteries should hit before throwing an error (deci Volts)
#define check 15000           // 15 seconds in milliseconds
#define Vfloat 136            // Float voltage set-point (deci Volts)

long panelVolts = 0;          // Solar panel Voltage (deci Volts)
long batteryVolts = 120;      // Battery Voltage (deci Volts)
long Voc = 0;                 // Panel open-circuit voltage (deci Volts)
long Vcvm = 0;                // Estimted voltage for MPP with CVM (deci Volts)
int pulseWidth = 100;         // Digital value of pwm (should never be 0 or 255)
int lastpulseWidth = 100;     // remember the value of pulseWidth for random resets
int pwm = 0;                  // Percentage of PWM
int b = 1;                    // do once
int stepAmount = 1;           // Scaling factor of pwm for large differences in voltage
int inByte = 0;               // incoming serial byte
int panelADC = 0;             // for sending through serial
int batteryADC = 0;           // for sending through serial
int state = 0;                // for sending through serial
int LEDstate = LOW;           // to record the state of the LED
//int errorCount = 0;           // record the # of times an error has occured
int32_t frequency = 40000;    // Frequency (in HZ)
unsigned long time = 0;       // Timer variable for timed charging cycles
unsigned long time2 = 0;      // Second timer variable for blink cycles

String enable = "starting";   // string of text to show enable pin of MOSFET driver
String SOC = "Initializing";  // string of text to show the charger state
String Load = "Off";          // string of text to show the load state

enum charger_mode {no_battery, sleep, bulk, Float, error} charger_state;  // enumerated variable that holds state for charger state machine

void setup() {
  
  Serial.begin(115200);                                       // faster communication means faster tracking and less voltage overshoot / undershoot
  pinMode(13, OUTPUT);                                        // set the LED pin as an output
  pinMode(load, OUTPUT);                                      // set the load pin as an output
  pinMode(shutDown, OUTPUT);                                  // set the shutDown pin as an output
  pinMode(fan, OUTPUT);                                       // set the fan pin as an output
  disable_charger();                                          // make sure the MOSFET driver is off
  InitTimersSafe();                                           // This is part of the PWM library. It allows you to set almost any PWM frequency you want
  charger_state = sleep;                                      // start with charger state as sleep
  
  bool success = SetPinFrequencySafe(driver, frequency);      // if setting the frequency to the desired pin was successful
  if(success){
    digitalWrite(13, HIGH);                                   // turn on the LED
    Serial.println("Timer frequency set success!");           // and print it to the serial port
  }
  else{                                                       // if not,
    digitalWrite(13, LOW);                                    // keep the LED off
    Serial.println("Timer frequency set failed!");            // and print it to the serial port
  }
  update_Vcvm();
  //establishContact();  // send a byte to establish contact until receiver responds (use with sendtogui)
}

void loop() {          // Main loop
  
  
  read_data();         // read the analog inputs
  mode_select();       // use that info to decide what charging state we should be in
  set_charger();       // run the selected charger sequence
  run_load();          // turn the load on or off, depending on the battery voltage
  run_fan();           // turn the fan on or off, depending on the charging cycle
    
  //Use only one of these 2:
  //sendtogui();         // for use with processing sketch
  print_data();        // print data to the serial port so that humans know what you're doing

}

void read_data() {                                   // function for reading analog inputs
  
  panelVolts = 0;
  for(int i=0;i<100;i++){
    panelVolts += analogRead(panelMeter);            // read the panel voltage 100 times and add the values together
  }
  batteryVolts = 0;
  for(int i=0;i<100;i++){
    batteryVolts += analogRead(batteryMeter);        // read the battery voltage 100 times and add the values together
  }
  
  panelVolts = panelVolts/100;                       // Divide by 100 to get the average value
  batteryVolts = batteryVolts/100;                   // Divide by 100 to get the average value

  //panelADC = panelVolts;                             // for sending through serial (use with sendtogui)
  //batteryADC = batteryVolts;                         // for sending through serial (use with sendtogui)
  
  //panelADC = panelADC/4;                             // for sending through serial (use with sendtogui)
  //batteryADC = batteryADC/4;                         // for sending through serial (use with sendtogui)
  
  panelVolts = (panelVolts*488)/1197;                // multiply the averaged ADC value by the scaling factor to get a number in deci volts
  batteryVolts = (batteryVolts*488)/2441;            // multiply the averaged ADC value by the scaling factor to get a number in deci volts
  
}

void mode_select(){
  if (batteryVolts < 100) charger_state = no_battery ;                                        // If battery voltage is below 10, there is no battery connected or dead / wrong battery
  else if (batteryVolts > Vmax) charger_state = error;                                        // If battery voltage is over 15, there's a problem
  else if ((batteryVolts > 100) && (batteryVolts < Vmax) && (panelVolts > Vmax)){             // If battery voltage is in the normal range and there is light on the panel
    if (batteryVolts >= (Vfloat-1)) charger_state = Float;                                    // If battery voltage is above 13.5, go into float charging
    else charger_state = bulk;                                                                // If battery voltage is less than 13.5, go into bulk charging
  }
  else if (panelVolts < Vmax){                                                                // If there's no light on the panel, go to sleep
    charger_state = sleep;
  }
}

void set_charger(){                                                                           // function for selecting the charging state
  
  switch (charger_state){                                                                     // skip to the state that is currently set
    
    case no_battery:                                                                          // if none of the other cases are satisfied,
      disable_charger();                                                                      // turn off the MOSFET driver
      error_blink();                                                                          // blink LED to indicate an error
      SOC = "No Battery!";
      state = 0; 
      break;
    
    case sleep:                                                                               // the charger is in the sleep state
      disable_charger();                                                                      // disable the charger so that current doesn't leak back into the solar panel
      SOC = "Sleep";
      sleep_blink();
      state = 1;
      pulseWidth = 100;
      break;
      
    case bulk:                                                                                // the charger is in the bulk state
      SOC = "Bulk";
      CVM();                                                                                  // begin the MPPT algorithm
      run_charger();                                                                          // enable the MOSFET driver
      digitalWrite(13, HIGH);                                                                 // turn the LED on to indicate bulk
      state = 2;
      break;
      
    case Float:                                                                               // the charger is in the float state, it uses PWM instead of MPPT
      SOC = "Float";
      pulseWidth = 245;                                                                       // set the pulseWidth to max (MPP doesn't matter when the battery is full)
      if (batteryVolts < Vfloat) run_charger();                                               // If battery voltage is below 13.6 enable the MOSFET driver
      else disable_charger();                                                                 // If above, disable the MOSFET driver
      digitalWrite(13, LOW);                                                                  // Turn the LED off to indicate float
      state = 3;
      break;
      
    case error:                                                                               // if there's something wrong
      disable_charger();                                                                      // turn off the MOSFET driver
      error_blink();                                                                          // blink LED to indicate an error
      SOC = "Error";
      state = 4;
      //errorCount++;
      break;                                                                                  // this state needs a reset to escape from
      
    default:                                                                                  // if none of the other cases are satisfied,
      disable_charger();                                                                      // turn off the MOSFET driver
      error_blink();                                                                          // blink LED to indicate an error
      SOC = "Off";
      break;
  }
}

void run_charger(){
  pulseWidth = constrain (pulseWidth, 75, 245);      // prevent overflow of pulse width and not fully on or off for the charge pump
  pwm = map(pulseWidth, 0, 255, 0, 100);             // use pulseWidth to get a % value and store it in pwm
  pwmWrite(driver, pulseWidth);                      // send the new pulseWidth to the MOSFET driver
  digitalWrite(shutDown, HIGH);                      // enable the MOSFET driver (enabling should always be done after sending pulseWidth)
  enable = "On";
}

void disable_charger(){
  digitalWrite(shutDown, LOW);                       // disable the MOSFET driver (disabling should always be done before sending pulseWidth)
  enable = "Off";
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                                               //
//    CVM is the actual MPPT algorithm. It exploits the fact that panel voltage at the MPP is about 76% of the open circuit voltage.             //
//    It doesn't require an ammeter to function but you lose the current and power information that you would normally have.                     //
//    Every 15 seconds, it puts the panel into open circuit and calculates what the voltage should be at the MPP.                                //
//    The algorithm's job is just to dump as much current into the battery as possible. It has no safetys or voltage limits.                     //
//    It relies on other blocks of code to monitor the battery voltage and put the charger into float (or absorb).                               //
//                                                                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CVM(){                                                // Constant Voltage Method MPPT without a high voltage limit (doesn't need an ammeter)
  while(b > 0){                                            // do this once
    time = millis();                                       // read millis and store in variable
    b--;  
  }
  if((millis() - time) >= check){                          // if it's been more than the check time:
    update_Vcvm();  
  }
  //if (Vcvm < batteryVolts) Vcvm = (batteryVolts + 10);     // to fix a stutter on initial startup
  stepAmount = (Vcvm - panelVolts)/10;                     // take bigger steps when the voltage is far from the target
  if (stepAmount < 1) stepAmount = 1;                      // the minimum step has to be one
  if (panelVolts > Vcvm){                                  // if the calculated MPP voltage is lower than the panel voltage,
    if (pulseWidth < 245){                                 // this is to keep pulseWidth from overflowing
      pulseWidth = pulseWidth + stepAmount;                // put more load on the panel
    }
    else pulseWidth = 245;                                 // this is to keep pulseWidth from overflowing
  }
  else if(panelVolts < Vcvm){                              // or if the panel voltage is less than the calculated MPP voltage,
    if (pulseWidth > 75){                                  // this is to keep pulseWidth from overflowing
      pulseWidth = pulseWidth - stepAmount;                // remove some of the load from the panel
    }
    else pulseWidth = 75;                                  // this is to keep pulseWidth from overflowing
  }
}

void update_Vcvm(){
  disable_charger();                            // disable the MOSFET driver
  delay(10);                                    // wait for the voltage to level out
  Voc = 0;
  for(int i=0;i<100;i++){
    Voc += analogRead(panelMeter);              // read the panel voltage 100 times and add the values together
  }
  Voc = Voc/100;
  Voc = (Voc*488)/1197;                         // multiply it by the scaling factor to produce a number in deci Volts
  Vcvm = (Voc*76)/100;                          // Vcvm is 76% of Voc
  b = 1;                                        // reset the timer
}

void run_fan(){
  switch(charger_state){
    case bulk:
    digitalWrite(fan, HIGH);
    break;
    default:
    digitalWrite(fan, LOW);
    break;
  }
}

void run_load(){
  if (batteryVolts > lowBatt){                  // If the battery voltage is above 11V
    digitalWrite(load, LOW);                    // Turn on the load MOSFET (LOW)
    Load = "On";                                // Write it in a string so that we can see it on the serial port
  }
  else{                                         // If the battery voltage is below 11V 
    digitalWrite(load, HIGH);                   // Turn off the load MOSFET (HIGH)
    Load = "Off";                               // Write it in a string so that we can see it on the serial port
  }
}

void error_blink(){                             // function for blinking the LED when there is an error
  if((millis() - time2) >= 200){                 // fast 1/5 second blink without delay
    LEDstate = !LEDstate;
    digitalWrite(13, LEDstate);
    time2 = millis();
  }
}

void sleep_blink(){                             // function for blinking the LED when sleeping
  if((millis() - time2) >= 2000){                // slow 2 second blink without delay
    LEDstate = !LEDstate;
    digitalWrite(13, LEDstate);
    time2 = millis();
  }
}

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('A');   // send a capital A
    delay(300);
  }
}


void sendtogui() {
  if (Serial.available() > 0) {
    inByte = Serial.read();
    Serial.write(panelADC);
    Serial.write(batteryADC);
    Serial.write(pwm);
    Serial.write(state);
  }
}


void print_data() {                             // Print all the information to the serial port
  
  Serial.print("Voc:");
  Serial.print(Voc);
  Serial.print("\t");
  Serial.print("Vpanel:");
  Serial.print(panelVolts);
  Serial.print("\t");
  Serial.print("Vcvm:");
  Serial.print(Vcvm);
  Serial.print("\t");
  Serial.print("Vbatt:");
  Serial.print(batteryVolts);
  Serial.print("\t");
  Serial.print("PWM:");
  Serial.print(pwm);
  Serial.print("%");
  Serial.print("\t");
  Serial.print("pulseWidth:");
  Serial.print(pulseWidth);
  Serial.print("\t");
  Serial.print("Enabled:");
  Serial.print(enable);
  Serial.print("\t");
  Serial.print("Charger State:");
  Serial.print(SOC);
  Serial.print("\t");
  Serial.print("Load State:");
  Serial.println(Load);
  
}

