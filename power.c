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

/*_____ D E F I N I T I O N S ______________________________________________*/


/*_____ D E C L A R A T I O N S ____________________________________________*/
bit f_heating_initialized = 0;     // 加熱功能是否已初始化
bit f_En_check_current_change = 1;

uint8_t level = 0;

static uint32_t current_sum = 0;
static uint32_t voltage_sum = 0;



/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void current_read(void);
void voltage_read(void);
void stop_heating(void);
void pause_heating(void);
void IGBT_C_slowdown(void);
void shutdown_process(void);
void finalize_shutdown(void);
uint16_t current_lookup_interpolation(uint16_t adc_val);

void Power_read(void)
{
  static uint8_t pwr_read_cnt = 0;     // 紀錄執行次數
  static uint16_t current_adc_avg = 0;
  
  current_read();  // 量測系統電流  
  voltage_read();  // 量測系統電壓
  
  // 計數 Power_read 執行次數
  pwr_read_cnt++;
  if (pwr_read_cnt >= measure_per_AC_cycle)
  { // 每 50/60 Hz 重置
    //P10 = 1; //HCW**
    if(f_AC_50Hz){
      //voltage_avg = voltage_sum/MEASUREMENTS_PER_50HZ(160)/4096*5*61.40477; // V (->avg)
      //voltage_avg = (voltage_sum*1965)>>22;
      
      //voltage_RMS_V = voltage_sum/MEASUREMENTS_PER_50HZ(160)/4096*5*(1/0.0163)*1.11; // V (->avg->RMS)
      //"0.0163" comes from voltage divider calculation.
      //For a full-wave rectified sine wave, Form Factor(K) = 1.11, meaning Vrms is 1.11*Vavg .
      voltage_RMS_V = (voltage_sum*2179)>>22;
      
      //current_adc_avg = current_sum/MEASUREMENTS_PER_50HZ(160);  // (->avg)
      current_adc_avg = (current_sum * 3277) >> 19;
      
    }
    else{
      //voltage_avg = voltage_sum/MEASUREMENTS_PER_60HZ(133)/4096*5*61.40477; // V (->avg)
      //voltage_avg = (voltage_sum*591)>>20;
      
      //voltage_RMS_V = voltage_sum/MEASUREMENTS_PER_60HZ(133)/4096*5*(1/0.0163)*1.11; // V (->avg->RMS)
      //"0.0163" comes from voltage divider calculation.
      //For a full-wave rectified sine wave, Form Factor(K) = 1.11, meaning Vrms is 1.11*Vavg .
      voltage_RMS_V = (voltage_sum*5243)>>23;
      
      //current_adc_avg = current_sum/MEASUREMENTS_PER_60HZ(133); // (->avg)
      current_adc_avg = (current_sum * 1971) >> 18;
    }
    
    // Remove current_base
    if(current_adc_avg > current_base){
      current_adc_avg -= current_base; 
    }
    else {
      current_adc_avg = 0;
    }
    
    // mA (->RMS)
    current_RMS_mA = current_lookup_interpolation(current_adc_avg);
    
    current_power = (uint32_t)voltage_RMS_V * current_RMS_mA;
    
    #if TUNE_MODE == 1
//    tune_cnt++;
//    if(tune_cnt >= 300)
//    {
//      PW0M = 0;
//      while(1);
//    }
    
//    tune_record = ;
//    tune_record = ;
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
typedef struct {
    uint16_t adc;     // ADC valuw without current_base
    uint16_t current; // 對應電流值 (單位：mA)
} LookupEntry;


#define TABLE_SIZE 9
// ADC_Code(base)
const LookupEntry code lookupTable[TABLE_SIZE] = {
    {0,    6},
    {500,    1813},
    {1000,   3620},
    {1500,   5428},
    {2000,   7235},
    {2500,   9042},
    {3000,   10849},
    {3500,   12656},
    {4000,   14464}
};
// To save computational resources on an 8-bit MCU, we use fixed-point arithmetic.
// We choose SCALE_BITS = 8 (i.e., multiply by 256).
// The fixed slope is 3.614397; its fixed-point representation is: 3.614397 * 256 ≈ 925.
#define SCALE_BITS 8
#define FIXED_SLOPE 925

// current_lookup_interpolation uses the pre-calculated fixed slope for interpolation.
// It takes an ADC value as input and returns the corresponding current value (in IRMS mA).
uint16_t current_lookup_interpolation(uint16_t adc_val) 
{
  uint8_t i;
  uint16_t adc_low, current_low, diff;
  uint16_t interpolated_current;
  
  // If ADC value is below the minimum in the table, return the minimum current.
  if (adc_val <= lookupTable[0].adc)
      return lookupTable[0].current;
  
  // If ADC value is above the maximum in the table, return the maximum current.
  if (adc_val >= lookupTable[TABLE_SIZE - 1].adc)
      return lookupTable[TABLE_SIZE - 1].current;

  // Find the interval in which adc_val falls
  for (i = 0; i < TABLE_SIZE - 1; i++) {
    if (adc_val < lookupTable[i+1].adc) {
      adc_low = lookupTable[i].adc;
      current_low  = lookupTable[i].current;
      diff = adc_val - adc_low;
      // interpolated_current = current_low  + (diff * FIXED_SLOPE) / 256
      interpolated_current = current_low  + (((uint32_t)diff * FIXED_SLOPE) >> SCALE_BITS);
      return interpolated_current;
    }
  }
  
  // Theoretically should not reach here.
  return lookupTable[TABLE_SIZE - 1].current;
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
#define DEFAULT_BASE_CURRENT_ADC   316   // 預設基準電流 ADC 值
#define BASE_CURRENT_TOLERANCE     8   // 允許誤差百分比（8%）

// 在編譯期間計算允許的上下限
#define BASE_CURRENT_LOWER_LIMIT  (((uint16_t)DEFAULT_BASE_CURRENT_ADC * (100 - BASE_CURRENT_TOLERANCE)) / 100)
#define BASE_CURRENT_UPPER_LIMIT  (((uint16_t)DEFAULT_BASE_CURRENT_ADC * (100 + BASE_CURRENT_TOLERANCE)) / 100)


uint16_t current_base = 0;

void Measure_Base_Current(void) {
  uint16_t measured_value;

  // 量測 ADC 基準電流值
  ADC_measure_4_avg(CURRENT_ADC_CHANNEL, &measured_value);
  
  // 檢查是否超出允許範圍
  if (measured_value < BASE_CURRENT_LOWER_LIMIT || measured_value > BASE_CURRENT_UPPER_LIMIT) {
      current_base = DEFAULT_BASE_CURRENT_ADC;  // 超出範圍則使用預設值
  } else {
      current_base = measured_value;  // 正常範圍內則使用測得值
  }
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
//    if (voltage_RMS_V > VOLTAGE_UPPER_LIMIT) {
//        error_flags.f.Voltage_overshoot = 1;  // 電壓超過上限
//        system_state = PAUSE;         // 切換系統狀態為暫停
//        stop_heating();               // 停止加熱操作
//    } else if (voltage_RMS_V < VOLTAGE_LOWER_LIMIT) {
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
//    if (abs(current_RMS_mA - last_current) > CURRENT_CHANGE_THRESHOLD) {
//        error_flags.f.Current_quick_large = 1;  // 電流快速變化
//    } else {
//        error_flags.f.Current_quick_large = 0;  // 清除快速變化標誌
//    }

//    // 更新上次的測量值
//    last_voltage = voltage_RMS_V;
//    last_current = current_RMS_mA;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void stop_heating(void)
{
  // 實現停止加熱邏輯
  P01 = 1;  //PWM Pin
  PW0M = 0;
  CM0M &= ~mskCM0SF;        // Patch: Disable CM0 pulse trigger, if enable pulse trigger PGOUT can't trigger.
  
  f_pot_detected = 0;       // 重置鍋具檢測標誌
  f_heating_initialized = 0;
  //power_setting = 0;
}

void pause_heating(void)
{
  //P10 = ~P10 ;//HCW**
  
  // 暫停加熱
  PW0M &= ~(mskPW0EN|mskPW0PO|mskPWM0OUT); // Disable PWM / pulse / normal PWM function
  CM0M &= ~mskCM0SF;            // Patch: Disable CM0 pulse trigger, if enable pulse trigger PGOUT can't trigger.
  f_heating_initialized = 0;    // Clear加熱已初始化標誌
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
    uint8_t heat_cycle;  // 加熱週期數
    uint8_t rest_cycle;  // 休息週期數
} PeriodicConfig;

const PeriodicConfig code Periodic_table[] = {
    {8, 2}, // level 0: 加熱 8 週期，休息 2 週期（對應 800000mW）
    {5, 5}, // level 1: 加熱 5 週期，休息 5 週期（對應 500000mW）
    {2, 8}  // level 2: 加熱 2 週期，休息 8 週期（對應 200000mW）
};

void Heat_Control(void)
{
  // 如果系統狀態為 ERROR
  if (system_state == ERROR) {
      return;
  }

  // 如果功率設定為 0，直接切換至待機狀態
  if (power_setting == 0) {
      system_state = STANDBY; // 切換系統狀態為待機
      stop_heating(); // 停止加熱
      return;
  }

  // 檢查鍋具狀態
  if (f_pot_detected == 0) {
    // 啟動Fan
    BUZZER_ENABLE;
    
    Pot_Detection(); // 執行鍋具檢測邏輯
    return;
  }
  
  // 一般加熱模式
  if (power_setting > 800000) {     
    target_power = power_setting; // 設定為實際功率
    system_state = HEATING;       // 切換系統狀態為 HEATING
  }
  // 間歇加熱模式
  else {
    // 檔位判斷
    switch (power_setting) {
      case 800000: level = 0; break; // 1檔
      case 500000: level = 1; break; // 2檔
      case 200000: level = 2; break; // 3檔
      default: return; // 確保安全
    } 
    system_state = PERIODIC_HEATING; // 切換系統狀態為間歇加熱模式
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef enum {
    PERIODIC_HEAT_PHASE,
    PERIODIC_HEAT_END_PHASE,  // **確保最後一個加熱週期結束後，先等 `ac_half_low_ticks_avg`**
    PERIODIC_REST_PHASE,
    PERIODIC_SLOWDOWN_PHASE
} PeriodicHeatState;


void Periodic_Power_Control(void) {
  static PeriodicHeatState periodic_heat_state = PERIODIC_REST_PHASE;
  static uint8_t sync_count = 0;       
  static uint8_t phase_start_tick = 0;  // **共用變數：記錄 SLOWDOWN_PHASE & HEAT_END_PHASE 開始時間*
  static bit f_first_entry = 1;  // **標記是否為第一次進入 PERIODIC_HEATING**
  uint8_t elapsed_ticks;
  
  // **確保只有在 PERIODIC_HEATING 狀態下執行**
  if (system_state != PERIODIC_HEATING) {
      f_first_entry = 1;  // **當離開 PERIODIC_HEATING 時，重置標誌**
      return;
  }
  
  // **初始化狀態，僅在進入 PERIODIC_HEATING 的第一個周期執行**
  if (f_first_entry) {
    sync_count = 0;
    
    // **判斷是否已經初始化過加熱**
    if (f_heating_initialized) {
        periodic_heat_state = PERIODIC_HEAT_PHASE;
    } else {
        periodic_heat_state = PERIODIC_REST_PHASE;
    }
    
    f_first_entry = 0;  // **清除首次進入標誌**
  }  
  
  // **檢查是否有新的 AC 訊號週期**
  if (f_CM3_AC_sync) {
      f_CM3_AC_sync = 0;
      sync_count++;
  }
  
  // **計算經過時間**
  elapsed_ticks = system_ticks - phase_start_tick;

  switch (periodic_heat_state) {
    case PERIODIC_REST_PHASE:
      if (sync_count >= Periodic_table[level].rest_cycle) {
        sync_count = 0;
        IGBT_C_slowdown();
        phase_start_tick = system_ticks;
        periodic_heat_state = PERIODIC_SLOWDOWN_PHASE;
      }
      break;

    case PERIODIC_SLOWDOWN_PHASE:
      // **等待 `ac_half_low_ticks_avg` 週期時間，再進入加熱**
      if (elapsed_ticks >= ac_half_low_ticks_avg) {
        pause_heating();    // Stop IGBT_C_slowdown PWM
        init_heating(PERIODIC);
        periodic_heat_state = PERIODIC_HEAT_PHASE;
      }
      break;

    case PERIODIC_HEAT_PHASE:
      if (sync_count >= Periodic_table[level].heat_cycle) {
        sync_count = 0;
        phase_start_tick = system_ticks;
        periodic_heat_state = PERIODIC_HEAT_END_PHASE;
      }
      break;

    case PERIODIC_HEAT_END_PHASE:
      // **等待 `ac_half_low_ticks_avg`，才真正停止加熱**
      if (elapsed_ticks >= ac_half_low_ticks_avg) {
        pause_heating();
        periodic_heat_state = PERIODIC_REST_PHASE;
      }
      break;
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define CURRENT_CHANGE_CHECK_DELAY 2000 // 電流變化檢查延遲時間 (2000ms， 1ms 為基準)

// 初始化加熱功能
void init_heating(HeatingMode heating_mode)
{ 
  #if TUNE_MODE == 1
  //PW0D = PWM_MAX_WIDTH;
  #endif
  
  // **設定 PWM 寬度**
  if (heating_mode == NORMAL) {
    PW0D = PWM_MIN_WIDTH;  // **一般模式：最小 PWM**
  } else { 
    PW0D = recorded_1000W_PW0D;  // **間歇模式：使用 `1000W` 參考值**
  }
  
  // Enable PWM
  PW0M = mskPW0EN | PW0_DIV1 | PW0_HOSC | PW0_INVERS | mskPW0PO;
  
  //FW啟動PWM第一次charge
  PW0M1 |= mskPGOUT;
  while((PW0M1&mskPGOUT) != 0); // Wait pulse end
  
  PW0M &= ~mskPW0EN;            // Disable PWM
  CM0M |= mskCM0SF;             // Enable CM0 pulse trigger
  PW0M |= mskPW0EN;             // Enable PWM
  
  cntdown_timer_start(CNTDOWN_TIMER_CURRENT_CHANGE, CURRENT_CHANGE_CHECK_DELAY); // 啟動2000ms倒數計時
  
  f_En_check_current_change = 0;   // 暫停檢查電流變化
  f_heating_initialized = 1;       // 設置初始化標誌
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define SLOWDOWN_PWM_WIDTH  24    // 750ns / 31.25ns = 24 clks
#define SLOWDOWN_PWM_PERIOD 600   // 18.75us / 31.25ns = 600 clks

void IGBT_C_slowdown(void) {
    PW0Y = SLOWDOWN_PWM_PERIOD;   // **設定 PWM 週期**
    PW0D = SLOWDOWN_PWM_WIDTH;    // **設定 PWM 脈衝寬度**

    // **開啟 PWM**
    PW0M = mskPW0EN | PW0_DIV1 | PW0_HOSC | PW0_INVERS | mskPWM0OUT;
}

