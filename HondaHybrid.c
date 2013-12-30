//#define DEBUG
//#define BOARDV1
//#define TEST
#include "HondaHybrid.h"
#include "pid.h"
/*
This is the main controller for the super capacitor hybrid scooter a.k.a RED
B2 close contactor
CONTROL BOX INPUTS
Athrottle- A0 - analog input representing the users input for speed, 10 bit
Vspeed - CCp2-a 16 bit representation of the speed of the vehicle
Acaps- A1 - analog input representing the current voltage of the supercapacitors, 10 bit
There are 18 Supercaps with a maximum voltage of 2.7 (2.85 is breakdown) volts for a total of 48.6 V
The circuit has a 10/110 voltage divider into the ADC. So minimum voltage of 18 Volts is 335 and 48 Volts is 893. Measured 38.2V equal to 726

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
#define brake_pin PIN_B4
#define ADC_DELAY delay_us(20)
#define Acaps_pin PIN_A1
#define Acaps_channel 1
#define Athrottle_pin PIN_A0  //Voltage goes from 1.5 (306)to 4.1(836)
#define Athrottle_Min 316
#define Athrottle_Max 860
#define Athrottle_3quarter 520
#define Athrottle_Full Athrottle_Max-Athrottle_Min
#define Athrottle_channel 0
#ifdef BOARDV1
#define Electric_Controller_Switch PIN_B0
#define Contactor_Switch PIN_B2
#else
#define Contactor_Switch PIN_B5
#define Controller_Power_Switch PIN_B2
#define Electric_Controller_Switch PIN_B0 //this is the switch from acceleration/breaking with driving high as breaking
#define ALGORITHM_INPUT_SWITCH PIN_A2
#endif
#define A_CAPS_MAX 725//893
#define A_CAPS_MIN 400//335
#define A_CAPS_MID_LOW (A_CAPS_MIN + 100) //This is the low end of the middle ACaps range. The range in which discharge and charge are allowed. 
#define A_CAPS_MID_HIGH (A_CAPS_MAX - 100) //This is the low end of the middle ACaps range. The range in which discharge and charge are allowed. 
#define V_SPEED_REGEN_MIN 250 //Why is there a minimum speed? Because below this no regenerative action is possible with electric motor
#define INSUFFICIENT_BRAKING_RUNNAWAY_ERROR 50000

//PID Values
#define K_P 1.50
#define K_I 0.00
#define K_D 0.20

struct PID_DATA pidData;
#define TIME_INTERVAL 157 //TODO replace

#define left_position 2500//4450
#define right_position 4600
#define servo_difference  right_position-left_position
#define servo_difference_div 5200
#define ELEC_CONTROLLER_OFFSET 900
//const float Athrottle_servo_factor = ((float) servo_difference)/((float) Athrottle_FULL);
#define servo_period   65356-50000
unsigned int16 current_servo_position=right_position;
int1 SERVO_PIN_TO_BE_SET_HIGH_ON_NEXT_TIMER = 0;

unsigned int16 number_of_timer0_interupts_since_reset =0;
unsigned int16 vSpeed= 0;
signed int16 ELECthrottle = 0;
unsigned int16 ICEthrottle = 0;
unsigned int16 Athrottle = 0;
unsigned int16 Acaps = 0;
int1 ICE_ON = 0;
int1 CURRENTLY_CHARGING = 0;
int1 RUNNAWAY_CHECK = 0;
signed int16 returnedValue =0;

enum{ 
   EVERYTHING_OFF,
   SPEED_TO_LOW_ICE_DIRECT,
   CHARGING_ALLOWED, 
   DISCHARGING_ALLOWED, 
   CHARGING_AND_DISCHARING_ALLOWED,
   INSUFFICIENT_BRAKING_RUNAWAY,
   USER_INPUT_OFF
} CHARGING_STATE;

/*
The #int_timer0 interupt is triggered on each timer0 8bit interupt
the function simply increments a overflow counter to be used by the ccp2
interupt to calculate speed
NUMBER OF OPERATIONS = 
*/
#int_timer0
void timer0_isr(){
   
      number_of_timer0_interupts_since_reset=number_of_timer0_interupts_since_reset+256;
      //current_servo_position=current_servo_position+1;
      
   
   if (number_of_timer0_interupts_since_reset>=1024){
      vSpeed = 0;
      number_of_timer0_interupts_since_reset=1024;
      
      
   }
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
      current_servo_position = right_position;
   }
//printf("Current servo position %ld",current_servo_position);
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
#ifdef BOARDV1
#int_ccp2
#else
#int_ccp1
#endif
void isr2()
{
if (number_of_timer0_interupts_since_reset>10){
vSpeed  = 1280 - (number_of_timer0_interupts_since_reset+get_timer0());
//vSpeed  = -(get_timer0());
set_timer0(0);
number_of_timer0_interupts_since_reset = 0;
}
}

void trickBreaking(){
write_dac((unsigned int16) 400+ELEC_CONTROLLER_OFFSET);
delay_ms(500);      
}

void printfLogf(char string){
   #ifdef DEBUG
      printf("IN PrintfLog");
      printf("%c",string);
   #else
      //delay_ms(250);
   #endif
}

#ifdef TEST

/*
TEST SUITE
*/

void createHeartbeat(){
  //This creates a heartbeat on pin B1
  while (1){
           output_high(servo_pin);        //Set the servo control pin to high
           delay_ms(1000);
           output_low(servo_pin);
           delay_ms(1000);
    }
}

void wiperAnalogVoltage(){
 
 //signed int16 test =0;
 unsigned int16 wiperValue = 0;
    while (1){
printf("Wiper Value is now %ld",wiperValue);
  
  if (wiperValue>(4095)){
     wiperValue = 0;
    printf("Wiper Value is now %ld",wiperValue);
  }
  wiperValue=wiperValue+10;
  write_dac(wiperValue);
  delay_ms(10);
    }
}

void heartbeatElectricControllerPower(){
    while (1){

  output_high(Contactor_Switch);        //Set the servo control pin to high
           delay_ms(1000);
           output_low(Contactor_Switch);
           delay_ms(1000);
    }
}

void wiperServo(){
    unsigned int32 wiperValue = 0;
  current_servo_position = left_position;
    while (1){
  printf("Servo Value %ld",current_servo_position);
  if ((current_servo_position+10)>right_position){
    current_servo_position = left_position;//servo_difference;
  }
  current_servo_position =current_servo_position+10;
  //delay_ms(1);
    }
}

void printAnalogThrottleInput(){
    set_adc_channel(Athrottle_channel);
      ADC_DELAY;
      Athrottle = read_adc();
  printf("Analog Throttle is %ld",Athrottle );
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

   init_dac();

   setup_timer_1(T1_DIV_BY_2| T1_INTERNAL);
   setup_timer_0(RTCC_INTERNAL|RTCC_DIV_128);
       // Configure CCP2 to capture fall
   #ifdef BOARDV1
   setup_ccp2(CCP_CAPTURE_RE);
   enable_interrupts(INT_CCP2);   // Setup interrupt on falling edge
   #else
   setup_ccp1(CCP_CAPTURE_RE);
   enable_interrupts(INT_CCP1);
   #endif
   enable_interrupts(INT_TIMER0);
   enable_interrupts(INT_TIMER1);   // Setup interrupt on falling edge
   enable_interrupts(GLOBAL);


   CHARGING_STATE = EVERYTHING_OFF;
   output_high(Electric_Controller_Switch);
   output_high(Controller_Power_Switch);
   write_dac(0);
   ICE_ON=TRUE;
   output_high(Contactor_Switch);
   output_high(brake_pin);
   pid_Init(K_P*SCALING_FACTOR,K_I*SCALING_FACTOR,K_D*SCALING_FACTOR, & pidData);
   delay_ms(3000);
   current_servo_position =right_position-1000;
   delay_ms(3000);
   current_servo_position =right_position;
   //write_dac(1000);
   //delay_ms(10000);
   //output_high(Contactor_Switch);
   //output_high(brake_pin);
   //output_high(Electric_Controller_Switch);
   while(TRUE) {
      
      //GET INPUTS
      //Vspeedhappens in interrupts
      //set_adc_channel(Acaps_channel);
      //ADC_DELAY;
      //Acaps = read_adc();
      
      //set_adc_channel(Athrottle_channel);
      //ADC_DELAY;
      //Athrottle = read_adc();
      //current_servo_position = right_position - (Athrottle-Athrottle_Min)*4;
      //#ifdef DEBUG
            printf("State: Weak HybridTEZT \n");  
        // #else
         //   delay_ms(250);
        /// #endif
      //if ((Athrottle>Athrottle_3quarter)&&(Acaps>A_CAPS_MIN)){
         //CURRENTLY_CHARGING=1;
         
         output_low(Electric_Controller_Switch);
         write_dac(0);
         output_low(brake_pin);
         //set electric motor to drive
         //#ifdef DEBUG
           // printf("drive \n");  
         //#else
           // delay_ms(250);
         //#endif
         
         
      //}
      }

}

 

#else
void main()
{  
   
   setup_adc_ports(NO_ANALOGS);
   setup_adc(ADC_OFF);
   setup_spi(FALSE);
   
   setup_counters(RTCC_INTERNAL,RTCC_DIV_2);
   setup_timer_1(T1_DISABLED);
   setup_timer_2(T2_DISABLED,0,1);
   setup_port_a(AN0_AN1_AN3);
   setup_adc(ADC_CLOCK_INTERNAL);
   
   init_dac();
 
   setup_timer_1(T1_DIV_BY_2| T1_INTERNAL); 
   setup_timer_0(RTCC_INTERNAL|RTCC_DIV_128);
   #ifdef BOARDV1
   setup_ccp2(CCP_CAPTURE_RE);
   enable_interrupts(INT_CCP2);   // Setup interrupt on falling edge
   #else
   setup_ccp1(CCP_CAPTURE_RE);
   enable_interrupts(INT_CCP1);
   #endif
   enable_interrupts(INT_TIMER0);
   enable_interrupts(INT_TIMER1);   // Setup interrupt on falling edge
   enable_interrupts(GLOBAL);
   
   
   
   
   CHARGING_STATE = EVERYTHING_OFF;
   output_high(Electric_Controller_Switch);
   output_high(Controller_Power_Switch);
   write_dac(0);
   ICE_ON=TRUE;
   output_high(Contactor_Switch);
   output_high(brake_pin);
   pid_Init(K_P*SCALING_FACTOR,K_I*SCALING_FACTOR,K_D*SCALING_FACTOR, & pidData);
   delay_ms(3000);
   current_servo_position =right_position-1000;
   delay_ms(3000);
   current_servo_position =right_position;
   //write_dac(1000);
   //delay_ms(10000);
   //output_high(Contactor_Switch);
   //output_high(brake_pin);
   //output_high(Electric_Controller_Switch);
   while(TRUE) {
      
      //GET INPUTS
      //Vspeedhappens in interrupts
      set_adc_channel(Acaps_channel);
      ADC_DELAY;
      Acaps = read_adc();
      
      set_adc_channel(Athrottle_channel);
      ADC_DELAY;
      Athrottle = read_adc();
      if (!input(ALGORITHM_INPUT_SWITCH)){
      delay_ms(1);
      current_servo_position = right_position - (Athrottle-Athrottle_Min)*4;
      #ifdef DEBUG
            printf("State: Weak Hybrid %ld\n",vSpeed);  
         //#else
           // delay_ms(250);
         #endif
      if ((Athrottle>Athrottle_3quarter)&&(Acaps>A_CAPS_MIN)){
         //CURRENTLY_CHARGING=1;

         output_low(Electric_Controller_Switch);
         write_dac(2000);
         output_low(brake_pin);
         //set electric motor to drive
         #ifdef DEBUG
            printf("drivex \n");  
         //#else
          //  delay_ms(250);
         #endif
         
         
      }else if ((Acaps<A_CAPS_MAX)&&(vSpeed>V_SPEED_REGEN_MIN)){
         // set electric motor to charge
         #ifdef DEBUG
            printf("breakingy \n");  
         //#else
           // delay_ms(250);
         #endif
         if (CURRENTLY_CHARGING==1){
        //        trickBreaking();
         }
         CURRENTLY_CHARGING=0;
         
         output_high(Electric_Controller_Switch);
         write_dac(1200+ELEC_CONTROLLER_OFFSET);
         output_high(brake_pin);
      }else {
         //set electric motor to zero
         #ifdef DEBUG
            printf("turn off motor \n");  
         //#else
           // delay_ms(250);
         #endif
         CURRENTLY_CHARGING=1;
         write_dac(0);
      }
      }else{
      delay_ms(1000);
      if (Athrottle<Athrottle_Min){
         Athrottle=Athrottle_Min;
      }
      //SET THE STATE
      if (Acaps> (A_CAPS_MAX +10)){
         //FREAK OUT
         //printf("State: Freak Out \n");
        output_low(Electric_Controller_Switch);
         #ifdef DEBUG
            printf("State: FREAK OUT \n");  
         //#else
           // delay_ms(250);
         #endif
         write_dac(0);
         //ICE_ON = FALSE;
         ICEthrottle = 0;
         //output_high(Contactor_Switch);
         CHARGING_STATE =EVERYTHING_OFF;
        //return;
        //break;
      }
      else if (Athrottle<(Athrottle_Min+5)){
         CHARGING_STATE = USER_INPUT_OFF;
         #ifdef DEBUG
            printf("State: Throttle Off \n");  
         //#else
           // delay_ms(250);
         #endif
         
         }
      else if (ICE_ON&&(vSpeed<V_SPEED_REGEN_MIN)){
         #ifdef DEBUG
            printf("State: Speed To Low %ld \n",vSpeed);  
         //#else
           // delay_ms(250);
         #endif
         CHARGING_STATE=SPEED_TO_LOW_ICE_DIRECT;
      }
      else if (checkRunnaway(& pidData)){// INSUFFICIENT_BRAKING_RUNNAWAY_ERROR){
         #ifdef DEBUG
            printf("State: RUNNAWAY \n");  
         //#else
           // delay_ms(250);
         #endif
         ICE_ON=TRUE;
         CHARGING_STATE=INSUFFICIENT_BRAKING_RUNAWAY;
      }
      else if ((Acaps > A_CAPS_MAX)&&(CHARGING_STATE==CHARGING_ALLOWED||CHARGING_STATE==CHARGING_AND_DISCHARING_ALLOWED)){
         //Stop Charging they are full
         
         #ifdef DEBUG
            printf("State: Caps Full \n");  
         //#else
           // delay_ms(250);
         #endif
         
        ICE_ON=FALSE;
        CHARGING_STATE=DISCHARGING_ALLOWED;
      }
      else if ((Acaps < A_CAPS_MIN)&& CHARGING_STATE!=CHARGING_ALLOWED){
         //Stop running electric, the caps are almost empty
        
        #ifdef DEBUG
            printf("State: Caps Empty \n");  
         //#else
           // delay_ms(250);
         #endif

        ICE_ON=TRUE;
        CHARGING_STATE=CHARGING_ALLOWED;
      }
     else if (A_CAPS_MID_LOW < Acaps < A_CAPS_MID_HIGH){
        #ifdef DEBUG
            printf("State: Normal \n");  
         //#else
           // delay_ms(250);
         #endif
         CHARGING_STATE=CHARGING_AND_DISCHARING_ALLOWED;
     }
     
     
//PID CONTROLLER
      //Servo to mirror Athrottle -> 
      //current_servo_position=right_position-(Athrottle-Athrottle_Min)*Athrottle_servo_factor;//(Athrottle/Athrottle_Full)*servo_difference;//(vSpeed/65536.0)*(2500);
      ////printf("Analog Cap %d Analog Throttle %Lu\n",(int) Acaps,Athrottle);
      //current_servo_position =right_position-vSpeed+200;
      ////printf("Speed %lu \n",speeder);
      //SET OUTPUTS 
      //The writing of the ICEThrottle happens in interupts and all that is
      //required is updating ICEthrottle
      
      returnedValue = pid_Controller((Athrottle-AThrottle_Min),vSpeed,& pidData);
      ELECthrottle = ELECthrottle+returnedValue;
      //printf("Throttle %ld and electhrottle %ld \n",Athrottle,ELECthrottle);
      
     
      if (ELECthrottle>2500){
         ELECthrottle=2500;
      }
      else if (ELECthrottle<-1000){
         ELECthrottle = -1000;
      }
      
      
      //SET OUTPUTS
      if (CHARGING_STATE==EVERYTHING_OFF||CHARGING_STATE==USER_INPUT_OFF){
         ICEthrottle = 0;
         ICE_ON = FALSE;
         current_servo_position =right_position;
         write_dac(0);
      }
      else if (CHARGING_STATE==SPEED_TO_LOW_ICE_DIRECT || CHARGING_STATE==INSUFFICIENT_BRAKING_RUNAWAY){
         if (ELECthrottle>0){
            current_servo_position =right_position- (3*ELECthrottle);
         }
         else{
            current_servo_position =right_position;
         }
         ICE_ON = TRUE;
         write_dac(0);
      }
      else{
      if (ELECthrottle<0){
        if (CHARGING_STATE==CHARGING_ALLOWED || CHARGING_STATE ==CHARGING_AND_DISCHARING_ALLOWED){
             if (CURRENTLY_CHARGING==1){
                //trickBreaking();
             }
             //ELECthrottle = 300;
             CURRENTLY_CHARGING=0;
             output_high(Electric_Controller_Switch);
             //printf("BREAKING \n");
             write_dac((abs(ELECthrottle)+ELEC_CONTROLLER_OFFSET));
             output_high(brake_pin);
             
        }
        else{
           //Decrease ICE throttle
           write_dac(0);
      
        }
      }
      else {
        if (CHARGING_STATE==DISCHARGING_ALLOWED || CHARGING_STATE ==CHARGING_AND_DISCHARING_ALLOWED){
             CURRENTLY_CHARGING=1;
             
             output_low(Electric_Controller_Switch);
             //printf("ACCELERATING \n");
             write_dac((abs(ELECthrottle)+ELEC_CONTROLLER_OFFSET));
             output_low(brake_pin);
        }
        else{
           //Increase ICE throttle
           write_dac(0);
      
        }
         
      }
      if (ICE_ON){
         //printf("ICE NORMAL \n");
         current_servo_position = left_position +500;
         //current_servo_position = right_position - (Athrottle-Athrottle_Min)*4;
      
      }
      else{
         current_servo_position =right_position;
         //printf("ICE OFF \n");
      }
      }
           
   }
   }

}

#endif

boolean checkRunnaway(struct PID_DATA *pid)
{
//return (pid->sumError > INSUFFICIENT_BRAKING_RUNNAWAY_ERROR);
return FALSE;
}

/*! \brief Initialisation of PID controller parameters.
 *
 *  Initialise the variables used by the PID algorithm.
 *
 *  \param p_factor  Proportional term.
 *  \param i_factor  Integral term.
 *  \param d_factor  Derivate term.
 *  \param pid  Struct with PID status.
 */
void pid_Init(int16 p_factor, int16 i_factor, int16 d_factor, struct PID_DATA *pid)
// Set up PID controller parameters
{
  // Start values for PID controller
  pid->sumError = 0;
  pid->lastProcessValue = 0;
  // Tuning constants for PID loop
  pid->P_Factor = p_factor;
  pid->I_Factor = i_factor;
  pid->D_Factor = d_factor;
  // Limits to avoid overflow
  pid->maxError = MAX_INT / (pid->P_Factor + 1);
  ////printf("Max %ld factor %ld and pid %ld",MAX_INT,pid->I_Factor,pid->maxError);
  pid->maxSumError = MAX_I_TERM / (pid->I_Factor + 1);
}


/*! \brief PID control algorithm.
 *
 *  Calculates output from setpoint, process value and PID status.
 *
 *  \param setPoint  Desired value.
 *  \param processValue  Measured value.
 *  \param pid_st  PID status struct.
 */
int16 pid_Controller(int16 setPoint, int16 processValue, struct PID_DATA *pid_st)
{
  signed int16 error, p_term, d_term;
  signed int32 i_term, ret, temp;
    processValue=processValue/4;
  ////printf("input %ld speed %ld ",setPoint,processValue);
  error = setPoint - processValue;
  
  // Calculate Pterm and limit error overflow
  
  if (error > (signed int16) pid_st->maxError){
    p_term = MAX_INT;
    ////printf("p greater error %ld a %ld p %ld",error,pid_st->maxError,p_term);

  }
  else if (error < (signed int16) -pid_st->maxError){
    p_term = -MAX_INT;
    ////printf("p less error %ld a %ld p %ld",error,-pid_st->maxError,p_term);

  }
  else{
    p_term = (signed int16) (pid_st->P_Factor * (float) error);
    ////printf("error %ld a %ld p %ld",error,pid_st->maxError,p_term);
  }
  
  // Calculate Iterm and limit integral runaway
  temp = pid_st->sumError + error;
  if(temp > (signed int32)pid_st->maxSumError){
    i_term = MAX_I_TERM;
    pid_st->sumError = pid_st->maxSumError;
    ////printf("\n greater temp %ld a %ld sum %ld",temp,pid_st->maxSumError,pid_st->sumError);

 }
  else if(temp < (signed int32)-pid_st->maxSumError){
    i_term = -MAX_I_TERM;
    pid_st->sumError = -pid_st->maxSumError;
    ////printf("\n less temp %ld a %ld sum %ld",temp,pid_st->maxSumError,pid_st->sumError);

  }
  else{
    pid_st->sumError = temp;
    i_term = pid_st->I_Factor * pid_st->sumError;
    ////printf("\n eror temp %ld i_term %ld sum %ld error %ld ",temp,i_term,pid_st->sumError,error);

  }

  // Calculate Dterm
  d_term = pid_st->D_Factor * (pid_st->lastProcessValue - processValue);
////printf("\n p_term %ld d_term %ld i_term %ld",p_term,d_term,i_term);
  pid_st->lastProcessValue = processValue;

  //ret = (p_term + i_term + d_term) / SCALING_FACTOR;
   ret = (p_term+d_term+(signed int16)i_term) / SCALING_FACTOR;
  if(ret > MAX_INT){
    ret = MAX_INT;
  }
  else if(ret < -MAX_INT){
    ret = -MAX_INT;
  }

  return((signed int16)ret);
}

/*! \brief Resets the integrator.
 *
 *  Calling this function will reset the integrator in the PID regulator.
 */
void pid_Reset_Integrator(pidData_t *pid_st)
{
  pid_st->sumError = 0;
}

