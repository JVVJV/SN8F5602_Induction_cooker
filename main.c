/******************** (C) COPYRIGHT 2024 SONiX *******************************
* COMPANY:	SONiX
* DATE:		  2025/01
* AUTHOR:		HCW
* IC:			  SR56F27
*____________________________________________________________________________
* REVISION		Date				User		Description
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

/*_____ D E F I N I T I O N S ______________________________________________*/


#define SYSTEM_TICKS_PER_10MS 80  // 每 10 ms 的計數 (125 μs * 80 = 10 ms)
#define SYSTEM_10MS_PER_SECOND 100  // 每秒包含的 10 ms 計數

TaskType current_task = TASK_HEAT_CONTROL; // 當前任務

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
  
  // 初始化模塊
  SystemCLK_Init();       // 初始化 系統頻率
  GPIO_Init();            // 初始化 GPIO 配置
  Timer0_Init();          // 初始化 Timer0，用於產生 125 微秒中斷
  Comparator_Init();      // 初始化所有比較器 (CM0, CM1, CM2)
  OP_Amp_Init();          // 初始化運算放大器，用於電流量測
  PWM_Init();             // 初始化PWM for IGBT driving
  Buzzer_Init();          // 初始化風扇驅動
  I2C_Init();           // 初始化 I2C 接口
  
  CNTdown_Timer_Init();   // 初始化倒數計時模組
  
  EAL = 1;                // 啟用全域中斷
  
  Warmup_Delay();         // 30ms
  ADC_Init();             // 初始化 ADC，用於多通道量測
  
  Measure_AC_Low_Time();  // 量測AC low tume, 用於IGBT C級能量漸放時間 & 啟動間歇加熱時間點
  Detect_AC_Frequency();
  
  // 設置初始系統狀態
  system_state = STANDBY;  // 系統默認進入待機模式

  #if TUNE_MODE == 1
//  power_setting = 800000;
  #endif
  
  // system_ticks reset to zero
  system_ticks = 0;
  
  // 進入主程式循環
  while (1) {
    I2C_Communication();
    
    // 125 μs 定時邏輯
    if (f_125us) {
      f_125us = 0;  // 清除 125 微秒旗標
            
      // 更新系統時間
      Update_System_Time();  
      
      // 獨立執行任務
      Power_read();    
          
      // 任務循環
      switch (current_task) {
        case TASK_HEAT_CONTROL:
            Heat_Control();
            current_task = TASK_POWER_CONTROL; // 切換到下一個任務
            break;
        
        case TASK_POWER_CONTROL:
            Power_Control();
            current_task = TASK_QUICK_CHANGE_DETECT; // 切換到下一個任務
            break;  
        
        case TASK_QUICK_CHANGE_DETECT:
            //Quick_Change_Detect();
            current_task = TASK_TEMP_MEASURE; // 切換到下一個任務
            break;

        case TASK_TEMP_MEASURE:
            Temp_Measure();
            current_task = TASK_TEMP_PROCESS; // 切換到下一個任務
            break;
          
        case TASK_TEMP_PROCESS:
            Temp_Process();
            current_task = TASK_CURRENT_POT_CHECK; // 切換到下一個任務
            break;

        case TASK_CURRENT_POT_CHECK:
            //Pot_Detection_In_Heating();  // 執行電流檢鍋任務
            current_task = TASK_SHUTDOWN; // 切換到下一個任務
            break;

        case TASK_SHUTDOWN:
            //Shutdown_Task();  // 執行關機任務
            current_task = TASK_ERROR_PROCESS; // 切換到下一個任務
            break;

        case TASK_ERROR_PROCESS:
            //Error_Process();  // 處理錯誤任務
            current_task = TASK_HEAT_CONTROL;  // 循環回到第一個任務
            break;

        default:  break;
      }
    } //(f_125us) end
    
    // **間歇加熱模式：即時檢查 AC 訊號變化**
    Periodic_Power_Control();
    
  } //while end
 
}

void Warmup_Delay(void)
{
  uint8_t cnt = 0;
  
  while(cnt < 240) //125us* 240 = 30ms
  {    
    if (f_125us)
    {
      f_125us = 0;
      cnt++;
    }
  }
  
}
