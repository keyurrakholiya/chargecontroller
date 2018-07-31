/*  The circuit:
   LCD RS pin to digital pin 12
   LCD Enable pin to digital pin 11
   LCD D4 pin to digital pin 5
   LCD D5 pin to digital pin 4
   LCD D6 pin to digital pin 3
   LCD D7 pin to digital pin 2
   LCD R/W pin to ground
   LCD VSS pin to ground
   LCD VCC pin to 5V
   10K resistor:
   ends to +5V and ground
   wiper to LCD VO pin (pin 3)


   Solar panel = 100 watt
   Voc = 22.4 volt
   Vmp = 17.8 volt
   Isc = 5.92 amps
   Imp = 5.62 amps
   Battery lead acid  = 12 Volt
*/
// include the library code:

#include <LiquidCrystal.h>
#define v_ref 4.92 // the exact value of voltage on vcc and gnd sensor
#define v_ref_volt 4.92 //the exact value of voltage on vin arduino
#define sen 0.1     // 0.1 V per amp (20 amp range)
                    // 0.066 V per amp(30 amp range)
#define R1 99 //Resistor R1 value for voltage
#define R2 14.7   //Resistor R2 value for voltage

// Code for current  ACS712 (20A) and voltage sensor
int     temp,temp2,temp3,temp4 = 0;
double  sum,sum2,sum3,sum4 = 0;
double  AMPS_SCALE ,AMPS_SCALE2= 0;
double  amps_in,amps_out= 0;
double  power_in,power_out = 0;
double  eff = 0;
double  VOLTS_SCALE ,VOLTS_SCALE2= 0;
double  volt ,volt2= 0;
double  SolarVolt=0, SolarCurrent=0,SolarPower = 0,OldPower;
float   max_battery_volt = 13.4, max_battery_currrent = 20;
float BatteryVolt;
int pwm_duty;
int buck = 6; //pwm pin 
enum charger_mode {OFF, MPPT, FLOAT} mode;  // enumerated variable that holds state for charger state machine


LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  Serial.begin(9600);
  lcd.begin(20, 4);
  pinMode(A0,INPUT);// input voltage sensor
  pinMode(A1,INPUT);//input current sensor
  pinMode(A2,INPUT);//output voltage sensor
  pinMode(A3,INPUT);//output current sensor

  pinMode(buck,OUTPUT); //pwm_output
}


/*****************************************ANALOG PIN A0************************************************/
float volt_in()
{
  for (int i = 0; i < 1000; i++)         // loop through reading raw adc values 1000 number of times
  {
    temp2 = analogRead(A0);              // read the input pin
    sum2 += temp2;                       // store sum for averaging
    delayMicroseconds(50);
  }
  sum2 = sum2 / 1000;                  // divide sum by 1000 to get average
 /*****************************************Calibration  for input voltage */
  VOLTS_SCALE = (v_ref_volt /1023) * ((R2+R1) / R2); // The voltage divider resistors are R1=99k and R2=14.7k // v_ref_volt is vin pin's voltage on arduino
  volt = VOLTS_SCALE * sum2 ;
  return volt;
}
/*****************************************ANALOG PIN A1************************************************/
float current_in()
{
  for (int i = 0; i < 1000; i++) // loop through reading raw adc values 1000 number of times
  {
    temp = analogRead(A1);   // read the input pin
    sum += temp;             // store sum for averaging
    delayMicroseconds(50);
  }
  sum = sum / 1000;          // divide sum by 1000 to get average(for accuracy)
/*****************************************Calibration  for input current*/ 
  AMPS_SCALE = (v_ref /1023) / sen;   // Sensitivity = 100mV(for 20A)
  amps_in = AMPS_SCALE * sum -(v_ref /(sen*2)) ;  //v_ref/2 is middle
  return amps_in;   
}
/*****************************************ANALOG PIN A2************************************************/
float volt_out()
{
  for (int i = 0; i < 1000; i++)         // loop through reading raw adc values 1000 number of times
  {
    temp4 = analogRead(A2);              // read the input pin
    sum4 += temp4;                       // store sum for averaging
    delayMicroseconds(50);
  }
  sum4 = sum4 / 1000;                  // divide sum by 1000 to get average
/*****************************************Calibration  for output voltage  */
  VOLTS_SCALE2 = (v_ref_volt /1023) * ((R2+R1) / R2); // The voltage divider resistors are R1=99k and R2=14.7k //// v_ref_volt is vin pin's voltage on arduino
  volt2 = VOLTS_SCALE2 * sum4 ;
  return volt2;
}
/*****************************************ANALOG PIN A3************************************************/
float current_out()
{
  for (int i = 0; i < 1000; i++) // loop through reading raw adc values 1000 number of times
  {
    temp3 = analogRead(A3);   // read the input pin
    sum3 += temp3;             // store sum for averaging
    delayMicroseconds(50);
  }
  sum3 = sum3 / 1000;          // divide sum by 1000 to get average(for accuracy)
/* ***************************************Calibration  for output current  */
  AMPS_SCALE2 = (v_ref /1023)/ sen;   // Sensitivity = 0.100mV(for 20A)
  amps_out = AMPS_SCALE2 * sum3 -(v_ref /(sen*2)) ;  //v_ref/2 is middle
  return amps_out;
}



void loop() {

  read_data();
  mode_select();
  run_charger();


  
}

void read_data()
{
  
  SolarVolt = volt_in();
  SolarCurrent = current_in();
  BatteryVolt = volt_out();
  SolarPower = SolarVolt * SolarCurrent ;
}

void mode_select()
{
  if( SolarVolt < BatteryVolt + 2) //here 2V is start margin
  {
    mode = OFF;//off mode
  }
  else if(BatteryVolt > max_battery_volt) //max_battery_volt = 13.4 volt
  {
    mode = FLOAT; // float mode : charging voltagae will be reduced
  }
  else 
  {
    mode = MPPT; //mppt mode
  }
}

void run_charger()
{
  switch(mode){

    case OFF:
      pwm_duty = 255;
      analogWrite(buck,pwm_duty);  //lowest voltage from buck
      break;

    case FLOAT:
      if(BatteryVolt > max_battery_volt)
      {
        pwm_duty++;
        pwm_duty = constrain (pwm_duty, 0, 255); //it will decrease the output volt of buck
        analogWrite(buck,pwm_duty); 
      }
      else 
      {
        pwm_duty--;
        pwm_duty =  constrain (pwm_duty, 0, 255);
        analogWrite(buck,pwm_duty); 
      }
      break;

    case MPPT:
      if(SolarPower < OldPower) 
      {
        pwm_duty--;                               //increasing output voltage 
        pwm_duty =  constrain (pwm_duty, 0, 255);
        analogWrite(buck,pwm_duty); 
      }
      else
       {
        pwm_duty++;                               //decreasing output voltage
        pwm_duty =  constrain (pwm_duty, 0, 255);
        analogWrite(buck,pwm_duty); 
      }
      OldPower = SolarPower; 
      break;
  }
}

