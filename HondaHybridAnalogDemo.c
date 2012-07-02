#include "C:\Documents and Settings\Enter\Desktop\Alex Bell\My Documents\MISC\Tilt accelerometer velocity\project\HondaHybrid.h"
unsigned int16 value1=0;
   unsigned int16 value2=0;
   signed int16 position=1500;
   signed int16 change=0;
   long rise,fall,pulse_width;

   unsigned int16 output = 0;
#int_ccp1
void isr()
{
   rise = CCP_1;
   fall = CCP_2;

   pulse_width = fall - rise;     // CCP_1 is the time the pulse went high
}                                 // CCP_2 is the time the pulse went low
                                  // pulse_width/(clock/4) is the time

                                  // In order for this to work the ISR
                                  // overhead must be less than the
                                  // low time.  For this program the
                                  // overhead is 45 instructions.  The
                                  // low time must then be at least
                                  // 9 us.



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
   init_dac();
 

   
   printf("\n\rHigh time (sampled every second):\n\r");
   setup_ccp1(CCP_CAPTURE_RE);    // Configure CCP1 to capture rise
   //setup_ccp2(CCP_CAPTURE_FE);    // Configure CCP2 to capture fall
   setup_timer_1(T1_INTERNAL);    // Start timer 1
   setup_timer_2(T2_DIV_BY_16,255,1);
   enable_interrupts(INT_CCP1);   // Setup interrupt on falling edge
   enable_interrupts(GLOBAL);
   while(TRUE) {
      output++;
      if (output>4096){
         output = 0;
      }
      delay_ms(100);
      write_dac(output);
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

