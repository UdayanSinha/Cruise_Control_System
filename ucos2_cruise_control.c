#include <stdio.h>
#include "system.h"
#include "includes.h"
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"

#define DEBUG 1

#define HW_TIMER_PERIOD 100 /* 100ms */

/* Button Patterns */

#define GAS_PEDAL_FLAG      0x08
#define BRAKE_PEDAL_FLAG    0x04
#define CRUISE_CONTROL_FLAG 0x02
/* Switch Patterns */

#define TOP_GEAR_FLAG       0x00004
#define LOW_GEAR_FLAG       0x00002
#define ENGINE_FLAG         0x00001

/* LED Patterns */

#define LED_RED_0 0x00001 // Engine
#define LED_RED_1 0x00002 // low Gear
#define LED_RED_2 0x00004 // Top Gear

#define LED_GREEN_0 0x01 // Cruise Control activated
#define LED_GREEN_2 0x02 // Cruise Control Button
#define LED_GREEN_4 0x10 // Brake Pedal
#define LED_GREEN_6 0x40 // Gas Pedal

/*
 * Definition of Tasks
 */

#define TASK_STACKSIZE 2048

OS_STK StartTask_Stack[TASK_STACKSIZE]; 
OS_STK ControlTask_Stack[TASK_STACKSIZE]; 
OS_STK VehicleTask_Stack[TASK_STACKSIZE];
OS_STK ButtonIOTask_Stack[TASK_STACKSIZE];
OS_STK SwitchIOTask_Stack[TASK_STACKSIZE];
OS_STK watchTask_Stack[TASK_STACKSIZE];
OS_STK overloadTask_Stack[TASK_STACKSIZE];

// Task Priorities
 
#define STARTTASK_PRIO     5
#define VEHICLETASK_PRIO  10
#define CONTROLTASK_PRIO  12
#define SwitchIOTASK_PRIO 13
#define ButtonIOTASK_PRIO 14
#define overloadTASK_PRIO 15
#define watchTASK_PRIO 16

// Task Periods

#define CONTROL_PERIOD  300
#define VEHICLE_PERIOD  300
#define IOTask_PERIOD 300
#define overloadTask_PERIOD 300

/*
 * Definition of Kernel Objects 
 */

// Mailboxes
OS_EVENT *Mbox_Throttle;
OS_EVENT *Mbox_Velocity;

// Semaphores
OS_EVENT *ioSemaphore;
OS_EVENT *vehicleSemaphore;
OS_EVENT *controlSemaphore;
OS_EVENT *watchSemaphore;
OS_EVENT *overloadSemaphore;


// SW-Timer
OS_TMR *controlTimer;
OS_TMR *watchDog;

/*
 * Types
 */
enum active {on, off};

enum active gas_pedal = off;
enum active brake_pedal = off;
enum active low_gear = off;
enum active top_gear = off;
enum active engine = off;
enum active cruise_control = off; 

/*
 * Global variables
 */
int delay; // Delay of HW-timer 
INT16U led_green = 0; // Green LEDs
INT32U led_red = 0;   // Red LEDs
char overload_flag=0;

int buttons_pressed(void)
{
  return ~IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_KEYS4_BASE);    
}

int switches_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);    
}

/*
 * ISR for HW Timer
 */
alt_u32 alarm_handler(void* context)
{
  OSTmrSignal(); /* Signals a 'tick' to the SW timers */
  
  return delay;
}

//timer callback function
void ControlTimerCallback(void *ptmr, void *callback_arg)
{
    if(overload_flag==0)           //no overload
    {
        OSSemPost(vehicleSemaphore); // Releasing the key
        OSSemPost(controlSemaphore); // Releasing the key
        OSSemPost(ioSemaphore); // Releasing the key
        OSSemPost(ioSemaphore); // Releasing the key
        OSSemPost(overloadSemaphore); // Releasing the key
    }
}

//watchdog callback function
void watchDogCallback(void *ptmr, void *callback_arg)
{
    INT8U err;
    OSSemSet(vehicleSemaphore, 0, &err);
    OSSemSet(controlSemaphore, 0, &err);
    OSSemSet(ioSemaphore, 0, &err);
    OSSemSet(ioSemaphore, 0, &err);
    OSSemPost(overloadSemaphore); // Releasing the key
    overload_flag=1;
    printf("OVERLOAD ALERT!!!!!!!!!!!!!!!!!!!\n");
}

static int b2sLUT[] = {0x40, //0
                 0x79, //1
                 0x24, //2
                 0x30, //3
                 0x19, //4
                 0x12, //5
                 0x02, //6
                 0x78, //7
                 0x00, //8
                 0x18, //9
                 0x3F, //-
};

/*
 * convert int to seven segment display format
 */
int int2seven(int inval){
    return b2sLUT[inval];
}

/*
 * output current velocity on the seven segement display
 */
void show_velocity_on_sevenseg(INT8S velocity){
  int tmp = velocity;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;
  int out;

  if(velocity < 0){
     out_sign = int2seven(10);
     tmp *= -1;
  }else{
     out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);
  out&=0x00003FFF;
  out = int2seven(0) << 21 |
            out_sign << 14 |
            out_high << 7  |
            out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_LOW28_BASE,out);
}

/*
 * shows the target velocity on the seven segment display (HEX5, HEX4)
 * when the cruise control is activated (0 otherwise)
 */
void show_target_velocity(INT8U target_vel)
{
    
    int out;
    int tmp1=target_vel/10;
    int tmp2=target_vel%10;
    if(cruise_control==on)
    {
        out=0x0FFFC000;
        out|=(int2seven(tmp2) | int2seven(tmp1)<<7);
    }
    else
    {
        out=0x0FFFFFFF;
    }
    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE,out);
}

/*
 * indicates the position of the vehicle on the track with the four leftmost red LEDs
 * LEDR17: [0m, 400m)
 * LEDR16: [400m, 800m)
 * LEDR15: [800m, 1200m)
 * LEDR14: [1200m, 1600m)
 * LEDR13: [1600m, 2000m)
 * LEDR12: [2000m, 2400m]
 */
void show_position(INT16U position)
{
    led_red&=0x00FFF;
    if(position>=0 && position<=4000)
    {
        led_red|=0x20000;
    }
    else if(position>=4000 && position<=8000)
    {
        led_red|=0x10000;
    }
    else if(position>=8000 && position<=12000)
    {
        led_red|=0x08000;
    }
    else if(position>=12000 && position<=16000)
    {
        led_red|=0x04000;
    }
    else if(position>=16000 && position<=20000)
    {
        led_red|=0x02000;
    }
    else if(position>=20000 && position<=24000)
    {
        led_red|=0x01000;
    }
    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red);
}

/*
 * The function 'adjust_position()' adjusts the position depending on the
 * acceleration and velocity.
 */
 INT16U adjust_position(INT16U position, INT16S velocity,
                        INT8S acceleration, INT16U time_interval)
{
  INT16S new_position = position + velocity * time_interval / 1000
    + acceleration / 2  * (time_interval / 1000) * (time_interval / 1000);

  if (new_position > 24000) {
    new_position -= 24000;
  } else if (new_position < 0){
    new_position += 24000;
  }
  
  show_position(new_position);
  return new_position;
}
 
/*
 * The function 'adjust_velocity()' adjusts the velocity depending on the
 * acceleration.
 */
INT16S adjust_velocity(INT16S velocity, INT8S acceleration,  
               enum active brake_pedal, INT16U time_interval)
{
  INT16S new_velocity;
  INT8U brake_retardation = 200;

  if (brake_pedal == off)
    new_velocity = velocity  + (float) (acceleration * time_interval) / 1000.0;
  else {
    if (brake_retardation * time_interval / 1000 > velocity)
      new_velocity = 0;
    else
      new_velocity = velocity - brake_retardation * time_interval / 1000;
  }
  
  return new_velocity;
}
//this task resets watchdog
void watchTask(void *pdata)
{
    INT8U err;
    while(1)
    {
        OSSemPend(watchSemaphore, 0, &err); // Trying to access the key
        if(err==OS_ERR_NONE)
        {
            OSTmrStop(watchDog, OS_TMR_OPT_NONE, NULL, &err);         //reset watchdog
            OSTmrStart(watchDog, &err);              //start watchdog
        }
    }
}
//this task adds dummy load 
void overload(void *pdata)
{
    INT8U err;
    int switch_state, prev_state, i;
    while(1)
    {
        OSSemPend(overloadSemaphore, 0, &err); // Trying to access the key
        if(err==OS_ERR_NONE)
        {
            switch_state=((switches_pressed())>>4) & 0x003F;
            switch_state*=48;          //2% increment
            if(switch_state>2400)
            {
                switch_state=2400;
            }
            printf("DUMMY LOAD: %d\n", switch_state);
            for(i=0; i<switch_state; i++);
            
            if(overload_flag==1 && switch_state!=prev_state)
            {
                overload_flag=0;
            }
        }
        prev_state=switch_state;
        OSSemPost(watchSemaphore);
    }
}
//this task reads switch I/O 
void SwitchIO(void *pdata)
{
    int switch_state;
    void* msg;
    INT8U err;
    INT16S* current_velocity;
    printf("Switch IO task created\n");
    while(1)
    {
        OSSemPend(ioSemaphore, 0, &err); // Trying to access the key
        switch_state=switches_pressed();
        if(err==OS_ERR_NONE)
        {
            if((switch_state & ENGINE_FLAG)==1)           //engine on
            {
                engine=on;
                led_red|=LED_RED_0;
            }
            else                                      //engine off
            {
                msg=OSMboxPend(Mbox_Velocity, 0, &err);
                if(err==OS_ERR_NONE)
                {
                    current_velocity=(INT16S*) msg;
                    if(*current_velocity==0)                //velocity=0
                    {
                        engine=off;
                        led_red&=(~LED_RED_0);
                    }
                }
            }
            if((switch_state & LOW_GEAR_FLAG)==2)                     //low gear on
            {
                low_gear=on;
                led_red|=LED_RED_1;
            }
            else                                                      //low gear off
            {
                low_gear=off;
                led_red&=(~LED_RED_1);
                if(top_gear==off)
                {
                    cruise_control=off;
                    led_green&=(~LED_GREEN_0);
                }
            }
            if((switch_state & TOP_GEAR_FLAG)==4)                     //top gear on
            {
                top_gear=on;
                led_red|=LED_RED_2;    
            }
            else                                                     //top gear off
            {
                top_gear=off;
                led_red&=(~LED_RED_2);
                cruise_control=off;
                led_green&=(~LED_GREEN_0);
            }
        }
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green);
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red);
    }
}
//this task reads button I/O 
void ButtonIO(void *pdata)
{
    unsigned char button_state;
    void* msg;
    INT8U err;
    INT16S* current_velocity;
    while(1)
    {
        OSSemPend(ioSemaphore, 0, &err); // Trying to access the key
        button_state=buttons_pressed();
        if(err==OS_ERR_NONE)
        {
            msg=OSMboxPend(Mbox_Velocity, 0, &err);
            current_velocity=(INT16S*) msg;
            if((button_state & CRUISE_CONTROL_FLAG)==2)                                //cruise control  
            {
                if(brake_pedal==off && gas_pedal==off && top_gear==on && err==OS_ERR_NONE && *current_velocity>=200)
                {
                    cruise_control=on;
                    led_green|=LED_GREEN_0;
                }
                led_green|=LED_GREEN_2;
            }
            else
            {
                led_green&=(~LED_GREEN_2);
            }
            if((button_state & GAS_PEDAL_FLAG)==8)                 //gas pedal
            {
                gas_pedal=on;
                cruise_control=off;
                led_green&=(~LED_GREEN_0);
                led_green|=LED_GREEN_6;
            }
            else
            {
                gas_pedal=off;
                led_green&=(~LED_GREEN_6);
            }
            if((button_state & BRAKE_PEDAL_FLAG)==4)                //brake pedal
            {
                brake_pedal=on;
                cruise_control=off;
                led_green&=(~LED_GREEN_0);
                led_green|=LED_GREEN_4;
            }
            else
            {
                brake_pedal=off;
                led_green&=(~LED_GREEN_4);
            }
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green);
        }
    }
}
/*
 * The task 'VehicleTask' updates the current velocity of the vehicle
 */
void VehicleTask(void* pdata)
{ 
  INT8U err;  
  void* msg;
  INT8U* throttle;
  INT8S acceleration;  /* Value between 40 and -20 (4.0 m/s^2 and -2.0 m/s^2) */
  INT8S retardation;   /* Value between 20 and -10 (2.0 m/s^2 and -1.0 m/s^2) */
  INT16U position = 0; /* Value between 0 and 20000 (0.0 m and 2000.0 m)  */
  INT16S velocity = 0; /* Value between -200 and 700 (-20.0 m/s amd 70.0 m/s) */
  INT16S wind_factor;   /* Value between -10 and 20 (2.0 m/s^2 and -1.0 m/s^2) */

  printf("Vehicle task created!\n");

  while(1)
    {
      err = OSMboxPost(Mbox_Velocity, (void *) &velocity);
      
      OSSemPend(vehicleSemaphore, 0, &err); // Trying to access the key
      /* Non-blocking read of mailbox: 
       - message in mailbox: update throttle
       - no message:         use old throttle
      */
      msg = OSMboxPend(Mbox_Throttle, 1, &err); 
      if (err == OS_NO_ERR) 
         throttle = (INT8U*) msg;
      /* Retardation : Factor of Terrain and Wind Resistance */
      if (velocity > 0)
         wind_factor = velocity * velocity / 10000 + 1;
      else 
         wind_factor = (-1) * velocity * velocity / 10000 + 1;
         
      if (position < 4000) 
         retardation = wind_factor; // even ground
      else if (position < 8000)
          retardation = wind_factor + 15; // traveling uphill
        else if (position < 12000)
            retardation = wind_factor + 25; // traveling steep uphill
          else if (position < 16000)
              retardation = wind_factor; // even ground
            else if (position < 20000)
                retardation = wind_factor - 10; //traveling downhill
              else
                  retardation = wind_factor - 5 ; // traveling steep downhill
                  
      acceleration = *throttle / 2 - retardation;     
      position = adjust_position(position, velocity, acceleration, 300); 
      velocity = adjust_velocity(velocity, acceleration, brake_pedal, 300); 
      printf("Position: %dm\n", position / 10);
      printf("Velocity: %4.1fm/s\n", velocity /10.0);
      printf("Throttle: %dV\n", *throttle / 10);
      show_velocity_on_sevenseg((INT8S) (velocity / 10));
    }
} 
 
/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */

void ControlTask(void* pdata)
{
    INT8U err;
    INT8U throttle = 40; /* Value between 0 and 80, which is interpreted as between 0.0V and 8.0V */
    void* msg;
    INT16S* current_velocity;
    INT16S target_velocity;
    INT16S threshold[3];
    char flag=0;

    printf("Control Task created!\n");

    while(1)
    {
      OSSemPend(controlSemaphore, 0, &err); // Trying to access the key
      msg = OSMboxPend(Mbox_Velocity, 1, &err);
      if(err==OS_ERR_NONE)
        {
            current_velocity = (INT16S*) msg;
            if(engine==on)
            {
                if(cruise_control==on)                    //maintain velocity at +-2
                {
                    if(flag==0)
                    {
                        if((*current_velocity>20) && (*current_velocity<25))
                        {
                            target_velocity=25;
                        }
                        else
                        {
                            target_velocity=*current_velocity;
                        }
                        threshold[0]=target_velocity-10;
                        threshold[1]=target_velocity+5;
                        threshold[2]=target_velocity+10;
                        flag=1;
                    }
                    if(*current_velocity<threshold[0])                                //more throttle needed
                    {
                        throttle+=10;    
                    }
                    else if((*current_velocity>threshold[0]) && (*current_velocity<threshold[1]))         //more throttle needed
                    {
                        throttle+=5;
                    }
                    else if((*current_velocity>threshold[1]) && (*current_velocity<threshold[2]))       //less throttle needed
                    {
                        throttle-=5;
                    }
                    else if(*current_velocity>threshold[2])  //less throttle needed
                    {
                        throttle-=10;
                    }
                    show_target_velocity(target_velocity/10);             //cruise control target velocity
                }
                else                                           //cruise control not active
                {
                    flag=0;
                    show_target_velocity(0);
                    if(gas_pedal==on)
                    {
                        throttle+=5;
                        if(low_gear==off && top_gear==off)   //1st gear, limit throttle to 20
                        {
                            if(throttle>20)
                            {
                                throttle=20;
                            }
                        }
                        else               
                        {
                            if(low_gear==on && top_gear==off)   //2nd gear, limit throttle to 30
                            {
                                if(throttle>30)
                                {
                                   throttle=30;
                                }
                            }
                            else if(low_gear==off && top_gear==on)  //3rd gear, limit throttle to 40
                            {
                                if(throttle>40)
                                {
                                   throttle=40;
                                }
                            }
                        }
                    }
                }
            }
            else                     //engine off, no throttle
            {
                throttle=0;
            }
        }
    //throttle must not blow up
    if(throttle>200)
    {
        throttle=0;
    }
    else if(throttle>80)
    {
        throttle=80;
    }
    OSSemPost(watchSemaphore); // Releasing the key
    err=OSMboxPost(Mbox_Throttle, (void *) &throttle);
    }
}

/* 
 * The task 'StartTask' creates all other tasks kernel objects and
 * deletes itself afterwards.
 */ 

void StartTask(void* pdata)
{
  INT8U err;
  void* context;

  static alt_alarm alarm;     /* Is needed for timer ISR function */
  
  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000; 
  printf("delay in ticks %d\n", delay);

  /* 
   * Create Hardware Timer with a period of 'delay' 
   */
  if (alt_alarm_start (&alarm,
      delay,
      alarm_handler,
      context) < 0)
      {
          printf("No system clock available!n");
      }

  /* 
   * Create and start Software Timer 
   */
   //create periodic timer                
  controlTimer = OSTmrCreate(0,
                             CONTROL_PERIOD/delay,
                             OS_TMR_OPT_PERIODIC,
                             ControlTimerCallback,
                             (void*)0,
                             "Barbara",
                             &err);
  if(err==OS_ERR_NONE)
  {
    printf("SW Timer created\n");
  }
                             
  OSTmrStart(controlTimer, &err);              //start timer
  if(err==OS_ERR_NONE)
  {
    printf("SW Timer started\n");
  }
  
  //create watchdog timer                
  watchDog = OSTmrCreate(4,
                             0,
                             OS_TMR_OPT_ONE_SHOT,
                             watchDogCallback,
                             (void*)0,
                             "watchdog",
                             &err);
  if(err==OS_ERR_NONE)
  {
    printf("Watchdog created\n");
  }
                             
  OSTmrStart(watchDog, &err);              //start watchdog
  if(err==OS_ERR_NONE)
  {
    printf("Watchdog started\n");
  }

  /*
   * Creation of Kernel Objects
   */
   
  //Semaphores
  ioSemaphore=OSSemCreate(2); // counting semaphore (2 keys)
  vehicleSemaphore=OSSemCreate(1); // binary semaphore (1 key)
  OSSemSet(vehicleSemaphore, 0, &err);     //set key to 0
  controlSemaphore=OSSemCreate(1); // binary semaphore (1 key)
  watchSemaphore=OSSemCreate(1); // binary semaphore (1 key)
  OSSemSet(watchSemaphore, 0, &err);     //set key to 0
  overloadSemaphore=OSSemCreate(1); // binary semaphore (1 key)
  
  // Mailboxes
  Mbox_Throttle = OSMboxCreate((void*) 0); /* Empty Mailbox - Throttle */
  Mbox_Velocity = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
   
  /*
   * Create statistics task
   */

  OSStatInit();

  /* 
   * Creating Tasks in the system 
   */


  err = OSTaskCreateExt(
       ControlTask, // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &ControlTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       CONTROLTASK_PRIO,
       CONTROLTASK_PRIO,
       (void *)&ControlTask_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
       VehicleTask, // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &VehicleTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       VEHICLETASK_PRIO,
       VEHICLETASK_PRIO,
       (void *)&VehicleTask_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);
       
  //creates IO tasks
  err = OSTaskCreateExt
    (SwitchIO,                     // Pointer to task code
     NULL,                         // Pointer to argument that is
                                   // passed to task
     &SwitchIOTask_Stack[TASK_STACKSIZE-1], // Pointer to top of task stack
     SwitchIOTASK_PRIO,               // Desired Task priority
     SwitchIOTASK_PRIO,               // Task ID
     &SwitchIOTask_Stack[0],                // Pointer to bottom of task stack
     TASK_STACKSIZE,               // Stacksize
     NULL,                         // Pointer to user supplied memory
                                   // (not needed here)
     OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
     OS_TASK_OPT_STK_CLR           // Stack Cleared                                 
    );     
    
  err = OSTaskCreateExt
    (ButtonIO,                     // Pointer to task code
     NULL,                         // Pointer to argument that is
                                   // passed to task
     &ButtonIOTask_Stack[TASK_STACKSIZE-1], // Pointer to top of task stack
     ButtonIOTASK_PRIO,               // Desired Task priority
     ButtonIOTASK_PRIO,               // Task ID
     &ButtonIOTask_Stack[0],                // Pointer to bottom of task stack
     TASK_STACKSIZE,               // Stacksize
     NULL,                         // Pointer to user supplied memory
                                   // (not needed here)
     OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
     OS_TASK_OPT_STK_CLR           // Stack Cleared                                 
    );
    
  err = OSTaskCreateExt
    (watchTask,                     // Pointer to task code
     NULL,                         // Pointer to argument that is
                                   // passed to task
     &watchTask_Stack[TASK_STACKSIZE-1], // Pointer to top of task stack
     watchTASK_PRIO,               // Desired Task priority
     watchTASK_PRIO,               // Task ID
     &watchTask_Stack[0],                // Pointer to bottom of task stack
     TASK_STACKSIZE,               // Stacksize
     NULL,                         // Pointer to user supplied memory
                                   // (not needed here)
     OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
     OS_TASK_OPT_STK_CLR           // Stack Cleared                                 
    );
    
  err = OSTaskCreateExt
    (overload,                     // Pointer to task code
     NULL,                         // Pointer to argument that is
                                   // passed to task
     &overloadTask_Stack[TASK_STACKSIZE-1], // Pointer to top of task stack
     overloadTASK_PRIO,               // Desired Task priority
     overloadTASK_PRIO,               // Task ID
     &overloadTask_Stack[0],                // Pointer to bottom of task stack
     TASK_STACKSIZE,               // Stacksize
     NULL,                         // Pointer to user supplied memory
                                   // (not needed here)
     OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
     OS_TASK_OPT_STK_CLR           // Stack Cleared                                 
    );
  
  printf("All Tasks and Kernel Objects generated!\n");

  /* Task deletes itself */

  OSTaskDel(OS_PRIO_SELF);
}

/*
 *
 * The function 'main' creates only a single task 'StartTask' and starts
 * the OS. All other tasks are started from the task 'StartTask'.
 *
 */

int main(void) {

  printf("Lab: Cruise Control\n");
 
  OSTaskCreateExt(
     StartTask, // Pointer to task code
         NULL,      // Pointer to argument that is
                    // passed to task
         (void *)&StartTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack 
         STARTTASK_PRIO,
         STARTTASK_PRIO,
         (void *)&StartTask_Stack[0],
         TASK_STACKSIZE,
         (void *) 0,  
         OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
         
  OSStart();
  
  return 0;
}
