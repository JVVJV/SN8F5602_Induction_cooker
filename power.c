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
#include "power.h"
#include "system.h"
#include "comparator.h"
#include "PWM.h"
#include "ADC.h"
#include "buzzer.h"
#include <math.h>

/*_____ D E C L A R A T I O N S ____________________________________________*/
bit f_heating_initialized = 0;     // 加熱功能是否已初始化
bit f_En_check_current_change = 1;

static uint32_t current_sum = 0;
static uint32_t voltage_sum = 0;
/*_____ D E F I N I T I O N S ______________________________________________*/


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void current_read(void);
void voltage_read(void);
void stop_heating(void);
void shutdown_process(void);
void finalize_shutdown(void);


void Power_read(void)
{
  static uint8_t pwr_read_cnt = 0;     // 紀錄執行次數
  
  current_read();  // 量測系統電流  
  voltage_read();  // 量測系統電壓
  
  // 計數 Power_read 執行次數
  pwr_read_cnt++;
  if (pwr_read_cnt >= MEASUREMENTS_PER_60HZ) 
  {  // 每 60 Hz 重置
    //P10 = 1; //HCW**
    
    //voltage_avg = voltage_sum/MEASUREMENTS_PER_60HZ/4096*5*61.40477; // V
    voltage_avg = (voltage_sum*591)>>20;
    //current_avg = current_sum/MEASUREMENTS_PER_60HZ/4096*5*1.25/47/0.01*1000;  //mA
    current_avg = (current_sum*6399)>>18;
    
    current_power = (uint32_t)current_avg * voltage_avg;
    
    #if TUNE_MODE == 1
    tune_cnt++;
//    if(tune_cnt >= 300)
//    {
//      PW0M = 0;
//      while(1);
//    }
    #endif
    
    pwr_read_cnt = 0;
    current_sum = 0;
    voltage_sum = 0;
    //P10 = 0; //HCW**
    
    //current_task = TASK_POWER_CONTROL;  // 重置任務狀態為TASK_POWER_CONTROL
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void current_read(void)
{
  uint16_t current_new = 0;        // 紀錄此次量測的電流數值
  //static uint16_t current_new = 0;        // 紀錄此次量測的電流數值
  //static uint16_t current_IIR_old = 0;    // 紀錄前一次 IIR 運算後的電流值
  
  
  // 更新 current_IIR_old 為上次的 IIR 新值
  //current_IIR_old = current_IIR_new;

  // 使用通用函式量測系統電流
  ADC_measure_4_avg(CURRENT_ADC_CHANNEL, &current_new);

  // IIR 運算
  //current_IIR_new = (current_new * 2 + current_IIR_old * 98) / 100;
  current_sum += current_new;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void voltage_read(void)
{
  uint16_t voltage_new = 0;        // 紀錄此次量測的電壓數值
  //static uint16_t voltage_new = 0;        // 紀錄此次量測的電壓數值
  //static uint16_t voltage_IIR_old = 0;    // 紀錄前一次 IIR 運算後的電壓值
  
  // 更新 voltage_IIR_old 為上次的 IIR 新值
  //voltage_IIR_old = voltage_IIR_new;

  // 使用通用函式量測電網電壓
  ADC_measure_4_avg(VOLTAGE_ADC_CHANNEL, &voltage_new);

  // IIR 運算
  //voltage_IIR_new = (voltage_new * 2 + voltage_IIR_old * 98) / 100;
  voltage_sum += voltage_new;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 定義電壓和電流快速變化及上下限
#define VOLTAGE_UPPER_LIMIT 280        // 電壓上限 (單位：適當的測量單位，例如伏特)
#define VOLTAGE_LOWER_LIMIT 90         // 電壓下限 (單位：適當的測量單位，例如伏特)
#define VOLTAGE_CHANGE_THRESHOLD 20    // 電壓快速變化閾值 (單位：伏特)
#define CURRENT_CHANGE_THRESHOLD 10    // 電流快速變化閾值 (單位：安培)

void Quick_Change_Detect() {
  // 前一次的測量值
  static uint16_t last_voltage = 0;     // 上次測量的電壓值
  static uint16_t last_current = 0;     // 上次測量的電流值  
  
//    // 檢查電壓是否超過上下限
//    if (voltage_avg > VOLTAGE_UPPER_LIMIT) {
//        error_flags.f.Voltage_overshoot = 1;  // 電壓超過上限
//        system_state = PAUSE;         // 切換系統狀態為暫停
//        stop_heating();               // 停止加熱操作
//    } else if (voltage_avg < VOLTAGE_LOWER_LIMIT) {
//        error_flags.f.Voltage_undershoot = 1;  // 電壓低於下限
//        system_state = PAUSE;         // 切換系統狀態為暫停
//        stop_heating();               // 停止加熱操作
//    } else {
//        error_flags.f.Voltage_overshoot = 0;  // 清除上限標誌
//        error_flags.f.Voltage_undershoot = 0; // 清除下限標誌
//    }

    // 檢查IGBT 過壓
    // CM2 out 持續時間超過 5ms -> stop_heating
    
//    // 檢查電壓是否快速變化
//    if (abs(voltage_IIR_new - last_voltage) > VOLTAGE_CHANGE_THRESHOLD) {
//        error_flags.f.Voltage_quick_change = 1;  // 電壓快速變化
//    } else {
//        error_flags.f.Voltage_quick_change = 0;  // 清除快速變化標誌
//    }

//    // 檢查電流是否快速變化
//    if (abs(current_avg - last_current) > CURRENT_CHANGE_THRESHOLD) {
//        error_flags.f.Current_quick_large = 1;  // 電流快速變化
//    } else {
//        error_flags.f.Current_quick_large = 0;  // 清除快速變化標誌
//    }

//    // 更新上次的測量值
//    last_voltage = voltage_avg;
//    last_current = current_avg;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void stop_heating(void)
{
    // 實現停止加熱邏輯
    P01 = 1;
    PW0M = 0;
    
    f_heating_initialized = 0;
}
void shutdown_process(void)
{
    // 一次性停止加熱並保存狀態
    stop_heating();
    //save_system_state();
}
void finalize_shutdown(void)
{
  _nop_(); //HCW**

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct {
    uint16_t heat_time;  // 加熱時間 (ms)
    uint16_t rest_time;  // 休息時間 (ms)
} IntermittentConfig;

const IntermittentConfig code intermittent_table[] = {
    {1000, 5000}, // 1檔: 加熱 1 秒，休息 5 秒
    {1000, 2000}, // 2檔: 加熱 1 秒，休息 2 秒
    {2000, 1000}  // 3檔: 加熱 2 秒，休息 1 秒
};

void Heat_Control(void)
{
  static uint8_t level = 0;
  
  // 如果系統狀態為 ERROR
  if (system_state == ERROR) {
      return;
  }

  // 如果功率設定為 0，直接切換至待機狀態
  if (power_setting == 0) {
      f_pot_detected = 0; // 重置鍋具檢測標誌
      system_state = STANDBY; // 切換系統狀態為待機
      stop_heating(); // 停止加熱
      return;
  }

  // 一般加熱模式
  if (power_setting >= 800000) {
      // 檢查鍋具狀態
      if (f_pot_detected == 1) {
          target_power = power_setting; // 設定為實際功率
          BUZZER_ENABLE;
          system_state = HEATING;      // 切換系統狀態為 HEATING
      } else {
          Pot_Detection(); // 執行鍋具檢測邏輯
      }
  }

  // 間歇加熱模式
  if (power_setting < 800000) {
    static bit intermittent_phase = 0;      // 0 表示加熱階段，1 表示休息階段
    uint8_t intermittent_timer_id = 4;  // 間歇加熱專用計時器 ID

    if (f_pot_detected == 1) {
        if (!cntdown_timer_expired(intermittent_timer_id)) {
            // 如果計時器未到期，保持當前狀態
            return;
        }
        
        // 間歇檔位判斷
        if (power_setting == 200000) {
            level = 0; // 1檔
        } else if (power_setting == 400000) {
            level = 1; // 2檔
        } else if (power_setting == 600000) {
            level = 2; // 3檔
        }  
      
        if (intermittent_phase == 0) {
            // 加熱階段
            target_power = 1200000; // 設定實際功率為 1200W
            system_state = HEATING; // 切換系統狀態為 HEATING
            cntdown_timer_start(intermittent_timer_id, intermittent_table[level].heat_time);
            intermittent_phase = 1; // 切換到休息階段
        } else {
            // 休息階段
            target_power = 0;        // 停止加熱
            system_state = STANDBY;  // 切換系統狀態為待機
            stop_heating();          // 停止加熱硬體
            cntdown_timer_start(intermittent_timer_id, intermittent_table[level].rest_time);
            intermittent_phase = 0; // 切換到加熱階段
        }
    } else {
        Pot_Detection(); // 執行鍋具檢測邏輯
    }
  }
}

#define CURRENT_CHANGE_CHECK_DELAY 2000 // 電流變化檢查延遲時間 (2000ms， 1ms 為基準)

// 初始化加熱功能
void init_heating(void)
{ 
  #if TUNE_MODE == 1
  //PW0D = PWM_MAX_WIDTH;
  #endif
  
  //FW啟動PWM第一次charge
  PW0M1 |= mskPGOUT;
  while((PW0M1&mskPGOUT) != 0); // Wait pulse end
  
  PW0M &= ~mskPW0EN;            // Disable PWM
  CM0M |= mskCM0SF;             // Enable CM0 pulse trigger
  //CM2M |= mskCM2SF;             // 開啟 CM2SF(IGBT高壓防護), Init()已開
  PW0M |= mskPW0EN;             // Enable PWM
  
  cntdown_timer_start(CNTDOWN_TIMER_CURRENT_CHANGE, CURRENT_CHANGE_CHECK_DELAY); // 啟動2000ms倒數計時
  
  f_En_check_current_change = 0;   // 暫停檢查電流變化
  f_heating_initialized = 1;       // 設置初始化標誌
}

