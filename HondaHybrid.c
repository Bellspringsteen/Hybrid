#include "C:\Documents and Settings\Enter\My Documents\Hybrid\Hybrid-v2\HondaHybrid.h"

/*
This is the main controller for the super capacitor hybrid scooter a.k.a RED

CONTROL BOX INPUTS
Athrottle - analog input representing the users input for speed, 10 bit
Vspeed - a 16 bit representation of the speed of the vehicle
Acaps-  analog input representing the current voltage of the supercapacitors, 10 bit

CONTROL BOX OUTPUT
ICEthrottle - a 16 bit number sent to the internal combustion engine(ICE) throttle 
ELECthrottle - a 16 bit number sent as analog value representing the demanded electric power

SPECIFICS OF I/O

Athrottle - the electric throttle is a 3 wire unit with a 5 volt,ground, and analog out
   representing how far throttle is turned. Pin AN0 is the input.
   
Vspeed - There is a encoder in the rear wheel/electric motor that is used by the electric
   power controller. Tapping in to this this Main Controller receives a 5 Volt square wave
   with 42 pulses. There are two options for measuring speed. The first is to measure the 
   length of the pulses. The second is to measure the rate of pulses. This controller currently
   uses the rate method. The int_ccp2 interupt is called on each falling edge and stores the 
   Vspeed value as the max number of 16bit (65535-timer0_since_last_reset). 
   
   The RED scooter has a circumpherence of 16.3*2*pi = 102.4
   X m/h * 63360 inch/mile * 1/3600 h/sec * 1/(102.4 inch/rev) = .171875* X
   Examples 50 mph is 8.9 ms 1 overflow of timer0
   Example 5 mph is 89 ms or 13 time overflow
   
   #int_timer0 increments the counter number_of_overflows each time it overflows
   On int ccp2 we multiply number_of_overflows * 256 + timer0 to get timer0_since_last_reset
   
Acaps - the analog voltage which is the 5volt representation of the voltage in the caps which
   have a value of between 0-48v. The voltages are factored so that the voltage on AN1 goes
   from 0 - 4.8 volts (TODO check this factor, think its 1/11 not 1/10)
   
ICEthrottle -  #int_timer1 is set to make sure the period follows the required below
   The period for the servo is .020 seconds which is 50,000 operations
   The leftmost position of the server is .001 seconds which is 2,500
   The rightmost position of the server is .002 seconds which is 5,000
   
   The output pin is pin_B1.->

ELECthrottle
   The electric power controller takes an analog voltage as input. The output voltage
   is set by the DAC. The pins used are C3,C4,C5. if CHARGING_STATE is true then 
   
ALGORITHM
   The algorithm is basically a PID controller. In broad terms we are trying to set the
   ICEthrottle and ICEelectric to maximize the use of electric and accelerate to meet the
   requirements set by the rider with the Athrottle signal. When the demanded power is 
   less than the power output possible by the ICE and the caps are in the CHARGING_STATE 
   then the ICE is turned up to the max and the ELECthrottle is set to regen to
   charge the capacitors.If CHARGING_STATE is false then the caps are supplying the drivig power
   and the ICE is set to "off", the caps supply the power. Unless the user requested supply
   is enough to require both power sources.
   
   If CHARGING_STATE is true. Then pin_b2 is set as high which will close the brake number
   for the electric power controller. 
   
   If CHARGING_STATE is false then the pin_b2 is set low which opens brake.
   
   


The encoder has 42 on and offs per rotation, i.e. 21 steps

Number of operations per second is CLOCK/4/Timer Divisions
Number of operations per second is 20,000,000/4/128 = 39062.5 
Each operation takes 1/39062.5 = .256 micro (x10^-6) seconds
Seconds to Overflow timer0 8bit timer = .256x10^-6 * 256 = 6.55ms



*/
#define servo_pin PIN_B1  //Setting servo out pin to be hardware pin b1
static int16 left_position = 2500;
static int16 right_position = 5000;
static int16 servo_period   = 65356-50000;

unsigned int16 current_servo_position=2500;
int1 SERVO_PIN_TO_BE_SET_HIGH_ON_NEXT_TIMER = 0;
int1 test_switch = 0;
unsigned int16 test_counter = 0;

#int_timer0
void timer0_isr(){

test_counter++;
if (test_counter>100){
test_counter=0;
if (test_switch){
 output_high(PIN_B2); 
 test_switch = 0;
}
else{
 output_low(PIN_B2); 
 test_switch = 1;
}
}
}


#int_timer1
void isr()
{

   if(SERVO_PIN_TO_BE_SET_HIGH_ON_NEXT_TIMER)
      { 
         output_high(servo_pin);                     //Set the servo control pin to high 
         SERVO_PIN_TO_BE_SET_HIGH_ON_NEXT_TIMER = 0; 
         set_timer1(65356-current_servo_position);                 //Set timer for the position high pulse
      } 
   else 
      { 
         output_low(servo_pin);                      // Set the servo control pin to low  
         SERVO_PIN_TO_BE_SET_HIGH_ON_NEXT_TIMER = 1; 
         set_timer1(servo_period+current_servo_position);          //Set timer for the low position the length is the difference between 
                                                     //the total int16 lenght - high pulse length
      }  

}



#int_ccp2
void isr2()
{

if (test_switch){
 output_high(PIN_B2); 
 test_switch = 0;
}
else{
 output_low(PIN_B2); 
 test_switch= 1;
}
   
   
}



void main()
{  
   
   setup_adc_ports(NO_ANALOGS);
   setup_adc(ADC_OFF);
   setup_spi(FALSE);
   
   //setup_counters(RTCC_INTERNAL,RTCC_DIV_2);
   //setup_timer_1(T1_DISABLED);
   //setup_timer_2(T2_DISABLED,0,1);
   setup_port_a(ALL_ANALOG);
   setup_adc(ADC_CLOCK_INTERNAL);
   init_dac();
 
   setup_timer_1(T1_DIV_BY_2| T1_INTERNAL); 
   setup_timer_0(RTCC_INTERNAL|RTCC_DIV_128);
   setup_ccp2(CCP_CAPTURE_RE);    // Configure CCP2 to capture fall
   enable_interrupts(INT_CCP2);   // Setup interrupt on falling edge
   enable_interrupts(INT_TIMER1);   // Setup interrupt on falling edge
   //enable_interrupts(INT_TIMER0);
   enable_interrupts(GLOBAL);
   while(TRUE) {
   if (current_servo_position<left_position){
      current_servo_position = left_position;
   }
   else if (current_servo_position > right_position){
      current_servo_position = left_position;
   }
   else {
      current_servo_position++;
   }
      //fprintf(MONITOR,"Hello");
      //left_adjust++;
      //output_low(servo_pin);
      delay_ms(10);
      //left_adjust--;
      //delay_ms(1000);
      //output_high(servo_pin);
      //delay_ms(1000);
      write_dac(current_servo_position-1000);
   }
   

}



