//----------------------------------------------------------------------------------------------------
//
//  Author: Adam Plavinskis
//          aplavins@gmail.com
//
//  This code is for an arduino UNO based Solar MPPT charge controller.
//  It is based of work done by Julian Ilett (256.co.uk), Debasish Dutta/deba168,
//  and Tim Nolan (www.timnolan.com).
//
//  The code could be more streamlined by eliminating the use of floats but I find it
//  easier to code and easier to understand.
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
//  -Setting the pulseWidth to values less than 40% for even a few milliseconds will cause the low side MOSFET
//  to short out and fail (sometimes violently).
//
//----------------------------------------------------------------------------------------------------


#include <PWM.h>

// Wiring:
// A0 - Voltage divider (solar)
// A1 - Voltage divider (battery)
// D2 - Load Mosfet
// D3 - 2104 MOSFET driver IN    (needs to be 3 or 11 with safe timer)
// D4 - 2104 MOSFET driver SD

// Voltage dividers:
// If you are using different resistor values, you'll have to modify these multipliers. A find and replace works well.
// I also reccomend using 1% tolerance resistors for more accurate readings.
// In version 3 I used a multiplyer to compensate for the arduino's voltage regulator output but I feel that the difference is minor.
// ADC volts/div = 0.004887585532747
// panel voltage divider: (+) 100k (sig) 14.7k (-) = 0.128160418482999
// example: 30V input, ADC reads x. Multiply that by 0.004887585532747 = V at the input. Divide by 0.128160418482999 to get input voltage.
// scaling factor is ADC value * 0.038136466707895 = input voltage
// battery voltage divider: (+) 75k (sig) 25k (-) = 0.25
// example: 13V output to the battery, ADC reads x. Multiply that by 0.004887585532747 = V at the input. Divide by 0.25 to get the input voltage.
// scaling factor is ADC value * 0.019550342130987 = battery voltage

const int panelMeter = A0;    // Analog input pin for PV voltmeter
const int batteryMeter = A1;  // Analog input pin for battery voltmeter
const int driver = 3;         // PWM output pin to mosfet driver (needs to be 3 or 11 with safe timer)
const int shutDown = 4;       // connected to shutdown pin of mosfet driver
const int load = 2;           // digital pin controlling the load MOSFET
const int Vbulk = 13;         // Bulk voltage set-point
const int lowBatt = 11;       // Low battery voltage set-point
const int Vmax = 15;          // Maximum voltage the batteries should hit before throwing an error
const long check = 15000;     // 15 seconds in milliseconds
const float Vfloat = 13.6;    // Float voltage set-point

float panelVolts = 0;         // Solar panel Voltage
float batteryVolts = 12;      // Battery Voltage
float Voc = 0;                // Panel open-circuit voltage
float Vcvm = 0;               // Estimted voltage for MPP with CVM
int pulseWidth = 245;         // Digital value of pwm (should never be 0 or 255)
int lastpulseWidth = 245;     // remember the value of pulseWidth for random resets
int pwm = 0;                  // Percentage of PWM
int b = 1;                    // do once
int stepAmount = 1;           // Scaling factor of pwm for large differences in voltage
int32_t frequency = 50000;    // Frequency (in HZ)
unsigned long time = 0;       // Timer variable for timed charging cycles

String enable = "starting";   // string of text to show enable pin of MOSFET driver
String SOC = "Initializing";  // string of text to show the charger state
String Load = "Off";          // string of text to show the load state

enum charger_mode {no_battery, sleep, bulk, Float, error} charger_state;  // enumerated variable that holds state for charger state machine

void setup() {
  
  Serial.begin(115200);                                       // faster communication means faster tracking and less voltage overshoot / undershoot
  pinMode(13, OUTPUT);                                        // set the LED pin as an output
  pinMode(load, OUTPUT);                                      // set the load pin as an output
  pinMode(shutDown, OUTPUT);                                  // set the shutDown pin as an output
  disable_charger();                                          // make sure the MOSFET driver is off
  InitTimersSafe();                                           // This is part of the PWM library. It allows you to set any* PWM frequency you want
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
}

void loop() {          // Main loop
  
  
  read_data();         // read the analog inputs
  mode_select();       // use that info to decide what charging state we should be in
  set_charger();       // run the selected charger sequence
  run_load();          // turn the load on or off, depending on the battery voltage
  print_data();        // print data to the serial port so that humans know what you're doing

}

void read_data() {                                   // function for reading analog inputs
  
  for(int i=0;i<100;i++){                            // read the inputs 100 times and add them together. The ADC has a capacitance that messes with the readings.
    panelVolts+=analogRead(panelMeter);              // read the input voltage from solar panel
    delay(1);                                        // wait a bit between readings
  }
  for(int i=0;i<100;i++){                            // read the inputs 100 times and add them together. The ADC has a capacitance that messes with the readings.
    batteryVolts+=analogRead(batteryMeter);          // read the battery voltage 
    delay(1);                                        // wait a bit between readings
  }

  panelVolts = panelVolts/100;                       // devide the result by 100 to get the average value
  batteryVolts = batteryVolts/100;                   // devide the result by 100 to get the average value
  
  panelVolts = panelVolts*0.038136466707895;         // multiply the averaged ADC value by the scaling factor to get a number in volts
  batteryVolts = batteryVolts*0.019550342130987;     // multiply the averaged ADC value by the scaling factor to get a number in volts
  
}

void mode_select(){
  if (batteryVolts < 10) charger_state = no_battery;                                          // If battery voltage is below 10, there is no battery connected or dead / wrong battery
  else if (batteryVolts > Vmax) charger_state = error;                                        // If battery voltage is over 15, there's a problem
  else if ((batteryVolts > 10) && (batteryVolts < Vmax) && (panelVolts > batteryVolts + 1)){  // If battery voltage is in the normal range and there is light on the panel
    if (batteryVolts <= (Vfloat-0.1)) charger_state = bulk;                                   // If battery voltage is less than 13.5, go into bulk charging
    else if (batteryVolts >= Vfloat) charger_state = Float;                                   // If battery voltage is above 13.6, go into float charging
  }
  else charger_state = sleep;                                                                 // If there's no light on the panel, go to sleep
}

void set_charger(){                                                                           // function for selecting the charging state
  
  switch (charger_state){                                                                     // skip to the state that is currently set
    
    case no_battery:                                                                          // if none of the other cases are satisfied,
      disable_charger();                                                                      // turn off the MOSFET driver
      error_blink();                                                                          // blink LED to indicate an error
      SOC = "No Battery!"; 
      break;
    
    case sleep:                                                                               // the charger is in the sleep state
      disable_charger();                                                                      // disable the charger so that current doesn't leak back into the solar panel
      SOC = "Sleep";
      sleep_blink();
      break;
      
    case bulk:                                                                                // the charger is in the bulk state
      SOC = "Bulk";
      CVM();                                                                                  // begin the MPPT algorithm
      run_charger();                                                                          // enable the MOSFET driver
      digitalWrite(13, HIGH);                                                                 // turn the LED on to indicate bulk
      //bulk_blink();                                                                         // this takes too long
      break;
      
    case Float:                                                                               // the charger is in the float state, it uses PWM instead of MPPT
      SOC = "Float";
      pulseWidth = 245;                                                                       // set the pulseWidth to max (MPP doesn't matter when the battery is full)
      if (batteryVolts < Vfloat) run_charger();                                               // If battery voltage is below 13.6 enable the MOSFET driver
      else disable_charger();                                                                 // If above, disable the MOSFET driver
      digitalWrite(13, LOW);                                                                  // Turn the LED off to indicate float
      //float_blink();                                                                        // this takes too long
      break;
      
    case error:                                                                               // if there's something wrong
      disable_charger();                                                                      // turn off the MOSFET driver
      error_blink();                                                                          // blink LED to indicate an error
      SOC = "Error";
      break;                                                                                  // this state needs a reset to escape from
      
    default:                                                                                  // if none of the other cases are satisfied,
      disable_charger();                                                                      // turn off the MOSFET driver
      error_blink();                                                                          // blink LED to indicate an error
      SOC = "Off"; 
      break;
  }
}

void run_charger(){
  pulseWidth = constrain (pulseWidth, 100, 245);     // prevent overflow of pulse width and not fully on or off for the charge pump
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
  if (Vcvm < batteryVolts) Vcvm = (batteryVolts + 1);      // to fix a stutter on initial startup
  stepAmount = abs(Vcvm - panelVolts);                     // take bigger steps when the voltage is far from the target
  if (stepAmount < 1) stepAmount = 1;                      // the minimum step has to be one
  if (panelVolts > Vcvm){                                  // if the calculated MPP voltage is lower than the panel voltage,
    if (pulseWidth < 245){                                 // this is to keep pulseWidth from overflowing
      pulseWidth = pulseWidth + stepAmount;                // put more load on the panel
    }
    else pulseWidth = 245;                                 // this is to keep pulseWidth from overflowing
  }
  else if(panelVolts < Vcvm){                              // or if the panel voltage is less than the calculated MPP voltage,
    if (pulseWidth > 100){                                 // this is to keep pulseWidth from overflowing
      pulseWidth = pulseWidth - stepAmount;                // remove some of the load from the panel
    }
    else pulseWidth = 100;                                 // this is to keep pulseWidth from overflowing
  }
}

void update_Vcvm(){
  disable_charger();                            // disable the MOSFET driver
  delay(100);                                   // wait for the voltage to level out
  for(int i=0;i<100;i++){                       // Take 100 readings and add them together
    Voc+=analogRead(panelMeter);                // read the input voltage from solar panel
    delay(1);                                   // wait a bit between each reading
  }
  Voc = Voc/100;                                // divide the result by 100 to get the average value
  Voc = Voc*0.038136466707895;                  // multiply it by the scaling factor to produce a number in volts
  Vcvm = Voc*0.76;                              // Vcvm is 76% of Voc
  b = 1;                                        // reset the timer
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
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
}

void sleep_blink(){                             // function for blinking the LED when sleeping
  digitalWrite(13, HIGH);
  delay(2000);
  digitalWrite(13, LOW);
  delay(2000);
}

void bulk_blink(){                              // function for blinking the LED in bulk
  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);
  delay(100);
  digitalWrite(13, HIGH);
  delay(400);
  digitalWrite(13, LOW);
  delay(100);
  digitalWrite(13, HIGH);
  delay(600);
  digitalWrite(13, LOW);
  delay(100);
}

void float_blink(){                             // function for blinking the LED in float
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
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

