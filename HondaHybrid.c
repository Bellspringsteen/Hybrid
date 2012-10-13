#include "C:\Documents and Settings\Enter\My Documents\Hybrid\Hybrid-v2\HondaHybrid.h"

/*
This is the main controller for the super capacitor hybrid scooter a.k.a RED
B2 close contactor
CONTROL BOX INPUTS
Athrottle- A0 - analog input representing the users input for speed, 10 bit
Vspeed - CCp2-a 16 bit representation of the speed of the vehicle
Acaps- A1 - analog input representing the current voltage of the supercapacitors, 10 bit

CONTROL BOX OUTPUT
ICEthrottle- B1 - a 16 bit number sent to the internal combustion engine(ICE) throttle 
ELECthrottle-  - a 16 bit number sent as analog value representing the demanded electric power

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
#define ADC_DELAY delay_us(200)
#define Acaps_pin PIN_A1
#define Acaps_channel 1
#define Athrottle_pin PIN_A0
#define Athrottle_channel 0

static int16 left_position = 2500;
static int16 right_position = 5000;
static int16 servo_period   = 65356-50000;
unsigned int16 current_servo_position=2500;
int1 SERVO_PIN_TO_BE_SET_HIGH_ON_NEXT_TIMER = 0;

int1 test_switch = 0;
unsigned int16 test_counter = 0;

unsigned int8 number_of_timer0_interupts_since_reset =0;
unsigned int16 timer0_since_last_reset= 0;
unsigned int16 vSpeed= 0;
unsigned int16 ELECthrottle = 0;
unsigned int16 ICEthrottle = 0;
unsigned int16 Athrottle = 0;
unsigned int16 Acaps = 0;
int1 CURRENTLY_CHARGING = 0;

/*
The #int_timer0 interupt is triggered on each timer0 8bit interupt
the function simply increments a overflow counter to be used by the ccp2
interupt to calculate speed
NUMBER OF OPERATIONS = 
*/
#int_timer0
void timer0_isr(){
number_of_timer0_interupts_since_reset++;
}

/*
#int_timer1 is used by the ICEservo throttle to regulate the timing pulses. The 
timer is setup for a pulsetrain of 20ms period. This is done as follows,

Period is 1/(CLOCK/4opsperclock)*(startingPostionOfClock) = 20 ms
for this setup 1/(20000000/4)*(50000) = 20 ms

The timer1 starts at 0 and counts up. So we set the beggining of the clock at
servo_period which is 65356-50000 so that the total time is 20ms.
*/
#int_timer1
void isr()
{

//Make sure that the position is within the left and right positions of the servo
   if (current_servo_position<left_position){
      current_servo_position = left_position;
   }
   else if (current_servo_position > right_position){
      current_servo_position = left_position;
   }

   if(SERVO_PIN_TO_BE_SET_HIGH_ON_NEXT_TIMER)
      { 
         output_high(servo_pin);        //Set the servo control pin to high 
         SERVO_PIN_TO_BE_SET_HIGH_ON_NEXT_TIMER = 0; 
         set_timer1(65356-current_servo_position); //Set timer for the position high pulse
      } 
   else 
      { 
         output_low(servo_pin);                      // Set the servo control pin to low  
         SERVO_PIN_TO_BE_SET_HIGH_ON_NEXT_TIMER = 1; 
         set_timer1(servo_period+current_servo_position);          //Set timer for the low position the length is the difference between 
                                                     //the total int16 lenght - high pulse length
      }  
}


/*
#int_ccp2 is called on the falling edge of the encoder pulse. We calculate the time
between pulses. 
TODO will have to put some kind of smoothing mechanism
*/
#int_ccp2
void isr2()
{
//timer0_since_last_reset = number_of_timer0_interupts_since_reset*256 + timer0;
//vSpeed = 65535-timer0_since_last_reset;
}


void main()
{  
   
   setup_adc_ports(NO_ANALOGS);
   setup_adc(ADC_OFF);
   setup_spi(FALSE);
   
   //setup_counters(RTCC_INTERNAL,RTCC_DIV_2);
   setup_timer_1(T1_DISABLED);
   setup_timer_2(T2_DISABLED,0,1);
   setup_port_a(ALL_ANALOG);
   setup_adc(ADC_CLOCK_INTERNAL);
   
   init_dac();
 
   setup_timer_1(T1_DIV_BY_2| T1_INTERNAL); 
   setup_timer_0(RTCC_INTERNAL|RTCC_DIV_128);
   setup_ccp2(CCP_CAPTURE_RE);    // Configure CCP2 to capture fall
   enable_interrupts(INT_CCP2);   // Setup interrupt on falling edge
   enable_interrupts(INT_TIMER1);   // Setup interrupt on falling edge
   enable_interrupts(GLOBAL);
   
   while(TRUE) {
 
      //GET INPUTS
      //Vspeed happens in interrupts
      set_adc_channel(Acaps_channel);
      ADC_DELAY;
      Acaps = read_adc();
      
      set_adc_channel(Athrottle_channel);
      ADC_DELAY;
      Athrottle = read_adc();

      //CONTROL BOX

      //output_low(PIN_B1);
      //delay_ms(1000);
      //output_high(PIN_B1);
      //delay_ms(1000);
      
      //delay_ms(Athrottle);
      //output_high(PIN_B1);
      //delay_ms(Athrottle);
      //output_low(PIN_B1);
      
      current_servo_position=left_position+(Athrottle/1024.0)*(2500);
      //printf("Analog Cap %d Analog Throttle %d\n",(int) Acaps, (int) Athrottle);
      
   
      //SET OUTPUTS 
      //The writing of the ICEThrottle happens in interupts and all that is
      //required is updating ICEthrottle
      //write_dac(ELECthrottle);
   }
   

}



