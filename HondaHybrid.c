#include "C:\Documents and Settings\Enter\My Documents\Hybrid\Hybrid-v2\HondaHybrid.h"

#define servo_pin PIN_B1  //Setting servo out pin to be hardware pin b1
/*
Clock Interrupt Settings

Number of operations per second is CLOCK/4/Timer Divisions
Number of operations per second is 20,000,000/4/2 = 2,500,000 

Each operation takes 1/2,500,000 = 400 nano seconds

The period for the servo is .020 seconds which is 50,000 operations
The leftmost position of the server is .001 seconds which is 2,500
The rightmost position of the server is .002 seconds which is 5,000

*/
static int16 left_position = 2500;
static int16 right_position = 5000;
static int16 servo_period   = 65356-50000;

unsigned int16 current_servo_position=2500;
int1 SERVO_PIN_TO_BE_SET_HIGH_ON_NEXT_TIMER = 0;

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



void main()
{  
   
   setup_adc_ports(NO_ANALOGS);
   setup_adc(ADC_OFF);
   setup_spi(FALSE);
   setup_counters(RTCC_INTERNAL,RTCC_DIV_2);
   setup_timer_1(T1_DISABLED);
   setup_timer_2(T2_DISABLED,0,1);
   setup_port_a(ALL_ANALOG);
   setup_adc(ADC_CLOCK_INTERNAL);
   //init_dac();
 

   
   //printf("\n\rHigh time (sampled every second):\n\r");
   //setup_ccp1(CCP_COMPARE_INT);    // Configure CCP1 to capture rise
   //setup_ccp2(CCP_CAPTURE_FE);    // Configure CCP2 to capture fall
   setup_timer_1(T1_DIV_BY_2| T1_INTERNAL); 

   //setup_timer_2(T2_DIV_BY_16,255,1);
   enable_interrupts(INT_TIMER1);   // Setup interrupt on falling edge
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
      //write_dac(output);
   }
   

}



