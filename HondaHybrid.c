#include "C:\Documents and Settings\Enter\Desktop\Alex Bell\My Documents\MISC\Tilt accelerometer velocity\project\HondaHybrid.h"
unsigned int16 value1=0;
   unsigned int16 value2=0;
   signed int16 position=0;
   signed int16 change=0;
   int1 SWITCH_SERVO = 0;
   long rise,fall,pulse_width;

   unsigned int16 output = 0;

#define servo_pin PIN_B1

#define SHORT_TIME      0.0009      // Shortest pulse width high time 
#define CENTER_TIME     0.0015      // The high time for center 
#define LONG_TIME       0.0021      // Longest pulse width high time 
#define PULSE_TIME      0.0200      // The total time for the pulse 

#ifndef TIMER_RATE 
#define TIMER_RATE      getenv("CLOCK") / 4 / T1_DIV_BY_2 
#endif 
#define SHORT_TICKS     (int16)((float)TIMER_RATE * SHORT_TIME) 
#define CENTER_TICKS    (int16)((float)TIMER_RATE * CENTER_TIME) 
#define LONG_TICKS      (int16)((float)TIMER_RATE * LONG_TIME) 
#define LOW_TICKS       (int16)(((float)TIMER_RATE * (PULSE_TIME - CENTER_TIME)) - 42) 
#define PULSE_CHANGE    (int16)(LONG_TICKS - CENTER_TICKS) 

signed int16 left_adjust; 
   
   
#int_timer1
void isr()
{
   //static int1 SWITCH_SERVO = 0;

  
      if(SWITCH_SERVO) 
      { 
         
         output_high(servo_pin);         // Set the servo control pin to low 
         //CCP_1 =500;   // Set CCP1 to interrupt for next high pulse 
         SWITCH_SERVO = 0; 
         set_timer1(65356-position);
      } 
      else 
      { 

         output_low(servo_pin);        // Set the servo control pin to high 
         //CCP_1 =20000;// Set CCP1 to interrupt for next low pulse 
         SWITCH_SERVO = 1; 
         set_timer1(13071+position);
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
   if (position<2617){
      position = 2617;
   }
   else if (position > 5200){
      position = 2617;
   }
   else {
      position++;
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
   
   /*
   set_adc_channel(0);
   delay_us(20);
   value1=read_adc();
   set_adc_channel(1);
   delay_us(20);
   value2=read_adc();
   
   
   change = (value2-value1);
   //printf("start");
   //printf("High %ld ",value1);
   //printf("High %ld ",value2);
   //printf("value1 %ld",change);
   
   if ((change>10)|| (change <-10)){
   position = position + change;
   if (position>2400){
      position=2400;
   }
   else if (position<600){
      position=600;
   }
   output_high(PIN_C1);
   delay_us(position);
   output_low(PIN_C1);
   delay_ms(1000);
   }
   }
   
*/
}



