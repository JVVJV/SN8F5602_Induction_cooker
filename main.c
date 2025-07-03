/******************** (C) COPYRIGHT 2024 SONiX *******************************
* COMPANY:	SONiX
* DATE:		  2025/01
* AUTHOR:		HCW
* IC:			  SR56F27
*____________________________________________________________________________
* REVISION		Date				User		Description
* 0.1         2025/02 
*____________________________________________________________________________
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#define __XRAM_SFR_H__
#include <SN8F5602.h>
#include "timer0.h"
#include "comparator.h"
#include "OP_amp.h"
#include "gpio.h"
#include "ADC.h"
#include "PWM.h"
#include "system.h"
#include "communication.h"
#include "power.h"
#include "temperature.h"
#include "buzzer.h"
#include "I2C.h"
#include "timer2.h"

/*_____ D E F I N I T I O N S ______________________________________________*/
#define SYSTEM_TICKS_PER_10MS   80    // Count per 10 ms (125 us * 80 = 10 ms)
#define SYSTEM_10MS_PER_SECOND  100   // Number of 10 ms units per second

TaskType current_task = TASK_HEAT_CONTROL;

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void Warmup_Delay(void);

/*****************************************************************************
* Function		: main
* Description	: Test loop
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void main (void)
{
  DEGCMD = 0x00; // Disable SWAT pin
  
  #if TUNE_MODE == 1
  OPM = 0x01;
  OPCAL -= 6; 
  #endif
  
  // Initialize modules
  SystemCLK_Init();       // Initialize system frequency
  GPIO_Init();            // Initialize GPIO configuration
  Timer0_Init();          // Initialize Timer0 for 125 us interrupts
	Timer2_Init();          // Initialize Timer2 for T2SF
  Comparator_Init();      // Initialize comparators (CM0, CM1, CM2)
  OP_Amp_Init();          // Initialize operational amplifiers for current measurement
  PWM_Init();             // Initialize PWM for IGBT driving
  Buzzer_Init();          // Initialize fan control
  I2C_Init();             // Initialize I2C interface
  
  CNTdown_Timer_Init();   // Initialize countdown timer module
  
  EAL = 1;                // Enable global interrupt
  
  Warmup_Delay();         // 30 ms delay
  
  ADC_Init();             // Initialize ADC
  
  Measure_AC_Low_Time();  // Measure AC low time for IGBT energy ramp and burst mode heating start
  Detect_AC_Frequency();  // 50Hz or 60Hz
  Measure_Base_Current(); //  Measure base current for OP offset
  Surge_Protection_Modify(); 
  
  // Reset timing-related flags used for scheduling or synchronization
  ISR_f_125us = 0;
  ISR_f_CM3_AC_Zero_sync = 0;
  ISR_f_CM3_AC_sync = 0;
  
  // Enter main loop
  while (1) {
    WDTR = 0x5A; // Clear watchdog
    I2C_Communication();
			
    // 125 us timing logic
    if (ISR_f_125us) {
      ISR_f_125us = 0;  // Clear 125 us flag
      
			// Update system time
      Update_System_Time();  
      
      // Regular tasks
      Power_read();
      Zero_Crossing_Task();
      Frequency_jitter();
       
      // Subtask loop
      switch (current_task) {
        case TASK_HEAT_CONTROL:
            Heat_Control();
            current_task = TASK_POWER_CONTROL; // Next task
            break;
        
        case TASK_POWER_CONTROL:
            Power_Control();
            current_task = TASK_QUICK_CHANGE_DETECT; // Next task
            break;  
        
        case TASK_QUICK_CHANGE_DETECT:
            Quick_Change_Detect();
            current_task = TASK_TEMP_MEASURE; // Next task
            break;

        case TASK_TEMP_MEASURE:
            Temp_Measure();
            current_task = TASK_TEMP_PROCESS; // Next task
            break;
          
        case TASK_TEMP_PROCESS:
            Temp_Process();
            current_task = TASK_CURRENT_POT_CHECK; // Next task
            break;

        case TASK_CURRENT_POT_CHECK:
            Pot_Detection_In_Heating();
            current_task = TASK_SHUTDOWN; // Next task
            break;

        case TASK_SHUTDOWN:
            //Shutdown_Task();  // Execute shutdown task
            current_task = TASK_ERROR_PROCESS; // Next task
            break;

        case TASK_ERROR_PROCESS:
            Error_Process();  // Handle error task
            current_task = TASK_HEAT_CONTROL;  // Back to first task
            break;

        default:  break;
      }
    } //(ISR_f_125us) end
    
    // Burst mode
    Periodic_Power_Control();
    

  } //while end
 
}

void Warmup_Delay(void)
{
  uint8_t cnt = 0;
  WDTR = 0x5A; // Clear watchdog
  
  while(cnt < 240) //125us* 240 = 30ms
  {    
    if (ISR_f_125us)
    {
      ISR_f_125us = 0;
      cnt++;
    }
  }  
}


