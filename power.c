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
volatile bit f_heating_initialized = 0;     // 加熱功能是否已初始化
bit f_power_updated = 0;

uint8_t level = 0;
uint16_t voltage_adc_new = 0;  // Store the measured voltage value (adc_code)
uint16_t current_adc_new = 0;  // Store the measured current value (adc_code)
uint16_t PW0D_backup = 0;
static uint8_t pwr_read_cnt = 0;      // Number of measurements during heating
static uint32_t current_adc_sum = 0;
static uint32_t voltage_adc_sum = 0;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void measure_power_signals(void);
void stop_heating(void);
void pause_heating(void);
void IGBT_C_slowdown(void);
void shutdown_process(void);
void finalize_shutdown(void);
void Start_Frequency_jitter(void);

uint16_t current_lookup_interpolation(uint16_t adc_val);

void Power_read(void)
{
  static uint16_t current_adc_avg = 0;
  static bit last_power_measure_valid = 0;   // Track previous state of f_power_measure_valid
 
  // Measure the system current & voltage
  ADC_measure_4_avg(CURRENT_ADC_CHANNEL, &current_adc_new);
  ADC_measure_4_avg(VOLTAGE_ADC_CHANNEL, &voltage_adc_new);

  // Handle PERIODIC_HEATING mode separately
  if (system_state == PERIODIC_HEATING) {
    if (last_power_measure_valid != f_power_measure_valid) {
      reset_power_read_data();
    }
    last_power_measure_valid = f_power_measure_valid;  // Update state tracking
  }
  
  // Accumulate Data
  current_adc_sum += current_adc_new;
  voltage_adc_sum += voltage_adc_new;
  pwr_read_cnt++;
 
  // Calculate power when reaching a full AC cycle
  if (pwr_read_cnt >= measure_per_AC_cycle)
  {
    #if AC_FREQ_MODE == AC_FREQ_MODE_FORCE_50HZ
      //voltage_RMS_V = voltage_adc_sum/MEASUREMENTS_PER_50HZ(160)/4096*5*(1/0.0163)*1.11; // V (->avg->RMS)
      //"0.0163" comes from voltage divider calculation.
      //For a full-wave rectified sine wave, Form Factor(K) = 1.11, meaning Vrms is 1.11*Vavg .
      voltage_RMS_V = (voltage_adc_sum*2179)>>22;
      
      //current_adc_avg = current_adc_sum/MEASUREMENTS_PER_50HZ(160);  // (->avg)
      current_adc_avg = (current_adc_sum * 3277) >> 19;
    
    #elif AC_FREQ_MODE == AC_FREQ_MODE_FORCE_60HZ
      //voltage_RMS_V = voltage_adc_sum/MEASUREMENTS_PER_60HZ(133)/4096*5*(1/0.0163)*1.11; // V (->avg->RMS)
      //"0.0163" comes from voltage divider calculation.
      //For a full-wave rectified sine wave, Form Factor(K) = 1.11, meaning Vrms is 1.11*Vavg .
      voltage_RMS_V = (voltage_adc_sum*5243)>>23;
      
      //current_adc_avg = current_adc_sum/MEASUREMENTS_PER_60HZ(133); // (->avg)
      current_adc_avg = (current_adc_sum * 1971) >> 18;
    #endif
    
    // Remove current_base
    if(current_adc_avg > current_base){
      current_adc_avg -= current_base; 
    }else{
      current_adc_avg = 0;
    }
    
    // mA (->RMS)
    current_RMS_mA = current_lookup_interpolation(current_adc_avg);
    //
    if (f_power_measure_valid) {
      // Caculate power
      current_power = (uint32_t)voltage_RMS_V * current_RMS_mA;
      f_power_updated = 1;
    }
    
    pwr_read_cnt = 0;
    current_adc_sum = 0;
    voltage_adc_sum = 0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void measure_power_signals(void)
{
  // Measure system current using ADC
  ADC_measure_4_avg(CURRENT_ADC_CHANNEL, &current_adc_new);
  
  // Measure AC voltage using ADC
  ADC_measure_4_avg(VOLTAGE_ADC_CHANNEL, &voltage_adc_new);

  // **Only accumulate the measured values when heating is active**
  if (f_heating_initialized) {
    current_adc_sum += current_adc_new;
    voltage_adc_sum += voltage_adc_new;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Resets internal variables used in power reading.
 *
 * This clears the ADC accumulation counters and sample count
 * used by Power_read(), typically called at startup or heating init.
 */
void reset_power_read_data(void)
{
  f_power_updated = 0;
  pwr_read_cnt = 0;
  current_adc_sum = 0;
  voltage_adc_sum = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct {
    uint16_t adc;     // ADC value without current_base
    uint16_t current; // 對應電流值 (單位：mA)
} LookupEntry;

#define TABLE_SIZE 9

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
#define DEFAULT_BASE_CURRENT_ADC   316    // 預設基準電流 ADC 值
#define BASE_CURRENT_TOLERANCE     8      // 允許誤差百分比（8%）

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
#define VOLTAGE_UPPER_LIMIT          255     // V 過壓觸發
#define VOLTAGE_RECOVER_HIGH         245     // V 過壓回復

#define VOLTAGE_LOWER_LIMIT          170     // V 欠壓觸發
#define VOLTAGE_RECOVER_LOW          180     // V 欠壓回復

#define CURRENT_UPPER_LIMIT_mA      9600    // 9.6A = 9600 mA
#define CURRENT_RECOVER_LIMIT_mA    9400   // 9.4A 過流回復

//#define VOLTAGE_CHANGE_THRESHOLD 20    // 電壓快速變化閾值 (單位：伏特)
//#define CURRENT_CHANGE_THRESHOLD 10    // 電流快速變化閾值 (單位：安培)

void Quick_Change_Detect() {
  // 前一次的測量值
//  static uint16_t last_voltage = 0;     // 上次測量的電壓值
//  static uint16_t last_current = 0;     // 上次測量的電流值  

    // === 過壓檢查與回復 ===
    if (error_flags.f.Over_voltage) {
        if (voltage_RMS_V < VOLTAGE_RECOVER_HIGH) {
            error_flags.f.Over_voltage = 0;
        }
    } else if (voltage_RMS_V > VOLTAGE_UPPER_LIMIT) {
        error_flags.f.Over_voltage = 1;
        // Simply stop the heating logic
        P01 = 1;  //PWM Pin
        PW0M = 0;
        PWM_INTERRUPT_DISABLE;
    }
  
    // === 欠壓檢查與回復 ===
    if (error_flags.f.Low_voltage) {
        if (voltage_RMS_V > VOLTAGE_RECOVER_LOW) {
            error_flags.f.Low_voltage = 0;
        }
    } else if (voltage_RMS_V < VOLTAGE_LOWER_LIMIT) {
        error_flags.f.Low_voltage = 1;
        // Simply stop the heating logic
        P01 = 1;  //PWM Pin
        PW0M = 0;
        PWM_INTERRUPT_DISABLE;
    }
    
    // === 過電流檢查與回復 ===
    if (error_flags.f.Over_current) {
        if (current_RMS_mA < CURRENT_RECOVER_LIMIT_mA) {
            error_flags.f.Over_current = 0;
        }
    } else if (current_RMS_mA > CURRENT_UPPER_LIMIT_mA) {
        error_flags.f.Over_current = 1;
        // Simply stop the heating logic
        P01 = 1;  //PWM Pin
        PW0M = 0;
        PWM_INTERRUPT_DISABLE;
    }
    
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
  PWM_INTERRUPT_DISABLE;
  
  CM0M &= ~mskCM0SF;        // Patch: Disable CM0 pulse trigger, if enable pulse trigger PGOUT can't trigger.
  
  PWM_Request_Reset();
  
  power_setting = 0;
  f_pot_detected = 0;       // 重置鍋具檢測標誌
  f_heating_initialized = 0;
}

void pause_heating(void)
{
  // 暫停加熱
  PW0M &= ~(mskPW0EN|mskPW0PO|mskPWM0OUT); // Disable PWM / pulse / normal PWM function
  PWM_INTERRUPT_DISABLE;
  CM0M &= ~mskCM0SF;            // Patch: Disable CM0 pulse trigger, if enable pulse trigger PGOUT can't trigger.
  f_heating_initialized = 0;    // Clear加熱已初始化標誌
}

void shutdown_process(void)
{    
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
    {4, 16}  // level 2: 加熱 4 週期，休息 16 週期（對應 200000mW）
};

void Heat_Control(void)
{
  static uint8_t idata prev_level = 0xFF;  // // For PERIODIC_HEATING, use invalid default to ensure first-time reset
  
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
    Fan_Enable();     // drive fan at normal or full speed
    Pot_Detection();  // 執行鍋具檢測邏輯
    return;
  }
  
  // 一般加熱模式
  if (power_setting > 800000) {
    
  // NOTE: power capping is currently disabled — always use the requested power HCW***
//    // If power_setting exceeds 1000W and IGBT heat warning is active, apply power cap at 1000W to prevent thermal stress.
//    if (warning_flags.f.IGBT_heat_warning) {
//        target_power = 1000000;
//    } else {
//        target_power = power_setting;
//    }
    
    target_power  = power_setting;
    
    system_state = HEATING;       // 切換系統狀態為 HEATING
    
    if (f_heating_initialized) {
      f_power_measure_valid = 1;
    } else {
      // Safety check: If switching from PERIODIC_HEATING to HEATING 
      // before heating has been initialized, make sure to disable any
      // leftover PWM output or PWM ISR (e.g., from slowdown phase).
      pause_heating();
    }
  }
  else // 間歇加熱模式
  {
    switch (power_setting) {    // 檔位判斷
      case 800000: level = 0; break; // 1檔
      case 500000: level = 1; break; // 2檔
      case 200000: level = 2; break; // 3檔
      default: return; // 確保安全
    }
    // Reset AC sync counter if level has changed, 
    // prevent periodic heating mode from switching periodic state at the wrong timing when level changes.
    if (level != prev_level)
    {
      periodic_AC_sync_cnt = 0;
      prev_level = level;
    }
    
    target_power = PERIODIC_TARGET_POWER;
    system_state = PERIODIC_HEATING; // 切換系統狀態為間歇加熱模式
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define POWER_MEASURE_DELAY_CYCLES  2 // Number of AC sync cycles to delay before measuring power

PeriodicHeatState periodic_heat_state = PERIODIC_REST_PHASE;
uint8_t periodic_AC_sync_cnt = 0;
bit f_power_measure_valid  = 0;  // Flag to indicate current measurement stabilization

void Periodic_Power_Control(void) {
  static uint8_t phase_start_tick = 0;  // 記錄 SLOWDOWN_PHASE & HEAT_END_PHASE 開始時間
  static uint8_t elapsed_ticks;
  static bit f_first_entry = 1;         // 標記是否為第一次進入 PERIODIC_HEATING
  static bit f_periodic_pulse_init = 1; // 標記是否需要 PULSE_WIDTH_PERIODIC_START
  
  // Only run in PERIODIC_HEATING state
  if (system_state != PERIODIC_HEATING) {
    // Reset entry flags when leaving periodic heating  
    f_first_entry = 1;
    f_periodic_pulse_init = 1;
    return;
  }
  
  //  Initialization on first entry to periodic heating
  if (f_first_entry) {
    periodic_AC_sync_cnt = 0;
    f_power_measure_valid = 0;
    ISR_f_CM3_AC_Periodic_sync = 0;
    
    // Choose starting phase based on whether heating was already initialized
    if (f_heating_initialized) {
        periodic_heat_state = PERIODIC_HEAT_PHASE;
    } else {
        periodic_heat_state = PERIODIC_REST_PHASE;
    }
    
    f_first_entry = 0;  // clear f_first_entry
  }  
  
  // Count AC sync events
  if (ISR_f_CM3_AC_Periodic_sync) {
      ISR_f_CM3_AC_Periodic_sync = 0;
      periodic_AC_sync_cnt++;
  }
  
  // Compute elapsed ticks since the start of current phase
  elapsed_ticks = system_ticks - phase_start_tick;

  switch (periodic_heat_state) {
    case PERIODIC_REST_PHASE:
      // After rest_cycle AC periods, enter slowdown phase
      if (periodic_AC_sync_cnt >= Periodic_table[level].rest_cycle) {
        periodic_AC_sync_cnt = 0;
        
        // Backup PW0D before IGBT_C_slowdown
        PW0D_lock = 1;
        PW0D_backup = PW0D;
        PW0D_lock = 0;
        
        IGBT_C_slowdown();
        phase_start_tick = system_ticks;
        periodic_heat_state = PERIODIC_SLOWDOWN_PHASE;
      }
      break;

    case PERIODIC_SLOWDOWN_PHASE:
      // Wait half-cycle average time before resuming heating
      if (elapsed_ticks >= ac_half_low_ticks_avg) {
        pause_heating();    // Stop IGBT_C_slowdown PWM
        
        // Restore PW0D after IGBT_C_slowdown
        PW0D_lock = 1;
        PW0D = PW0D_backup;
        PW0D_lock = 0;
        
        init_heating(HEATING_IMMEDIATE, f_periodic_pulse_init ? PULSE_WIDTH_PERIODIC_START : PULSE_WIDTH_NO_CHANGE);
        f_periodic_pulse_init = 0;
        
        periodic_heat_state = PERIODIC_HEAT_PHASE;
      }
      break;

    case PERIODIC_HEAT_PHASE:
      // Delay power measurement until current signal stabilizes (after two AC sync events)
      if (periodic_AC_sync_cnt == POWER_MEASURE_DELAY_CYCLES) {
        f_power_measure_valid = 1;
      }
    
      if (periodic_AC_sync_cnt >= Periodic_table[level].heat_cycle) {
        periodic_AC_sync_cnt = 0;
        phase_start_tick = system_ticks;
        periodic_heat_state = PERIODIC_HEAT_END_PHASE;
      }
      break;

    case PERIODIC_HEAT_END_PHASE:
      // Wait half-cycle then actually stop heating
      if (elapsed_ticks >= ac_half_low_ticks_avg) {
        pause_heating();
        f_power_measure_valid = 0;
        periodic_heat_state = PERIODIC_REST_PHASE;
      }
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define POT_HEATING_CURRENT_DELAY_MS 16  // Delay pot detection after heating starts

void init_heating(uint8_t sync_ac_low, PulseWidthSelect pulse_width_select)
{ 
  // Wait for AC low if required
  if (sync_ac_low) {
    ISR_f_CM3_AC_sync = 0; // Clear AC sync flag before waiting
    
    while (!ISR_f_CM3_AC_sync); // Block execution until AC low is detected
    f_block_occurred = 1;
  }
  
  // Patch: Protect for PWM accidentally switch mode
  PW0M &= ~mskPW0EN;    // Disable PWM
  CM0M &= ~mskCM0SF;    // Disable CM0 pulse trigger
  
  // Set PWM duty cycle based on pulse_width_select
  switch (pulse_width_select) {
    case PULSE_WIDTH_MIN:  // Set PW0D to minimum width
        PW0D = PWM_INIT_WIDTH;
        break;
    case PULSE_WIDTH_PERIODIC_START:  // PERIODIC mode reference value set to 1000W HCW**
        PW0D = 280;
        break;
    case PULSE_WIDTH_NO_CHANGE:  // **Keep PW0D unchanged**
    default:
        break;
  }
  
  // Enable PWM
  PW0M = mskPW0EN | PW0_DIV1 | PW0_HOSC | PW0_INVERS | mskPW0PO;
  
  // FW starts PWM for the first charge
  PW0M1 |= mskPGOUT;
  while((PW0M1&mskPGOUT) != 0); // Wait pulse end
  
  PW0M &= ~mskPW0EN;            // Disable PWM
  CM0M |= mskCM0SF;             // Enable CM0 pulse trigger
  PW0M = mskPW0EN | PW0_DIV1 | PW0_HOSC | PW0_INVERS | mskPW0PO;  // Enable PWM
  
//  // Start a 16ms countdown, not use now.
//  cntdown_timer_start(CNTDOWN_TIMER_POT_HEATING_CURRENT_DELAY , POT_HEATING_CURRENT_DELAY_MS); 
  
  reset_power_read_data();
  f_heating_initialized = 1;       // Mark heating as initialized
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IGBT_C_slowdown(void) {
    // Patch: Protect for PWM accidentally switch mode
    PW0M &= ~mskPW0EN;    // Disable PWM
    CM0M &= ~mskCM0SF;    // Disable CM0 pulse trigger
  
    PW0Y = SLOWDOWN_PWM_PERIOD;   // **設定 PWM 週期**
    PW0D = SLOWDOWN_PWM_START_WIDTH;    // **設定 PWM 脈衝寬度**
    
    PW0F_CLEAR;
    PWM_INTERRUPT_ENABLE;
  
    // Enable PWM
    PW0M = mskPW0EN | PW0_DIV1 | PW0_HOSC | PW0_INVERS | mskPWM0OUT;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Zero crossing trigger ticks (1 tick = 125us)
#define ZERO_CROSS_TRIGGER_TICKS_50HZ   13  // 1.625ms
#define ZERO_CROSS_TRIGGER_TICKS_60HZ   12  // 1.500ms

#if AC_FREQ_MODE == AC_FREQ_MODE_FORCE_50HZ
  #define ZERO_CROSS_TRIGGER_TICKS ZERO_CROSS_TRIGGER_TICKS_50HZ
#elif AC_FREQ_MODE == AC_FREQ_MODE_FORCE_60HZ
  #define ZERO_CROSS_TRIGGER_TICKS ZERO_CROSS_TRIGGER_TICKS_60HZ
#endif

static uint8_t real_zero_cross_timer = 0;
static bit f_zero_cross_task_active = 0;

void Reset_Zero_Crossing_Task(void)
{
    ISR_f_CM3_AC_Zero_sync = 0;
    f_zero_cross_task_active = 0;
    real_zero_cross_timer = 0;
}

void Zero_Crossing_Task(void)
{
  // If a blocking condition occurred, reset task state
  if (f_block_occurred) {
      f_block_occurred = 0;
      Reset_Zero_Crossing_Task();
  }

  // Triggered by comparator ISR on AC zero-cross
  if (ISR_f_CM3_AC_Zero_sync) {
      ISR_f_CM3_AC_Zero_sync = 0;
      f_zero_cross_task_active = 1;
      real_zero_cross_timer = 0;
  }
  
  if (!f_zero_cross_task_active)
    return;

  real_zero_cross_timer++;

//  // Runtime decision based on measured AC frequency
//  if (real_zero_cross_timer >= (f_AC_50Hz ? ZERO_CROSS_TRIGGER_TICKS_50HZ : ZERO_CROSS_TRIGGER_TICKS_60HZ)) {

  if (real_zero_cross_timer >= ZERO_CROSS_TRIGGER_TICKS) {    
    f_zero_cross_task_active = 0;
    real_zero_cross_timer = 0;

    // Real_zero task
    Start_Frequency_jitter();  // Execute zero-cross triggered function(s)
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Controls the PWM frequency jittering sequence for EMI optimization or switching noise reduction.
 *
 * This function implements a multi-phase state machine that gradually adjusts the PWM duty
 * (PW0D) through controlled jittering (decreasing and increasing phases).
 *
 * The jitter process is triggered by Start_Frequency_jitter(), and runs over the following states:
 *
 *  - JITTER_HOLD:          Initial idle delay before jitter starts
 *  - JITTER_HOLD_GAP:      Adds a fixed gap (2 ticks) before enabling PWM interrupts to prevent abrupt transitions
 *  - JITTER_DECREASE:      PWM0_ISR is enabled, and will decrement PW0D gradually
 *  - JITTER_INCREASE:      PWM0_ISR continues, now incrementing PW0D back
 *  - JITTER_INCREASE_GAP:  PWM interrupts are disabled, followed by a delay before finalizing
 *
 * Two control flags are used:
 *  - f_jitter_in_progress: Indicates the jitter sequence is running (for general control flow)
 *  - f_jitter_active:      Specifically indicates that PWM0_ISR is allowed to modify PW0D
 *
 * The f_jitter_active flag prevents race conditions between the main control loop and PWM0_ISR.
 * Without this guard, simultaneous writes to PW0D from the main program and interrupt context
 * could lead to undefined hardware behavior or unpredictable results.
 *
 * This function must be called every 125us tick from the main control loop.
 */

// JITTER_HOLD duration (unit: 125us ticks)
#define JITTER_HOLD_TICKS_50HZ     23  // 2.875ms
#define JITTER_DEC_TICKS_50HZ      16  // 2ms
#define JITTER_INC_TICKS_50HZ      16  // 2ms

#define JITTER_HOLD_TICKS_60HZ     21  // 2.625ms
#define JITTER_DEC_TICKS_60HZ      12  // 1.5ms
#define JITTER_INC_TICKS_60HZ      12  // 1.5ms

#if AC_FREQ_MODE == AC_FREQ_MODE_FORCE_50HZ
  #define JITTER_HOLD_TICKS  JITTER_HOLD_TICKS_50HZ
  #define JITTER_DEC_TICKS   JITTER_DEC_TICKS_50HZ
  #define JITTER_INC_TICKS   JITTER_INC_TICKS_50HZ
#elif AC_FREQ_MODE == AC_FREQ_MODE_FORCE_60HZ
  #define JITTER_HOLD_TICKS  JITTER_HOLD_TICKS_60HZ
  #define JITTER_DEC_TICKS   JITTER_DEC_TICKS_60HZ
  #define JITTER_INC_TICKS   JITTER_INC_TICKS_60HZ
#endif

static bit f_jitter_in_progress  = 0;
bit f_jitter_active = 0;                    // 抖頻啟動旗標
static uint8_t jitter_state_start_tick = 0; // 記錄啟動時間點
volatile uint8_t jitter_adjust_cnt = 0;
FrequencyJitterState Frequency_jitter_state = JITTER_HOLD;

void Frequency_jitter(void)
{
  static uint8_t elapsed;

  // 若不在加熱狀態，不執行任何動作
  if (!f_heating_initialized) {
    f_jitter_in_progress  = 0;
    f_jitter_active = 0;
    Frequency_jitter_state = JITTER_HOLD;
    return;
  }

  // 狀態尚未被觸發，什麼都不做
  if (!f_jitter_in_progress )
    return;

  elapsed = system_ticks - jitter_state_start_tick;

  switch (Frequency_jitter_state)
  {
    case JITTER_HOLD:
        if (elapsed >= JITTER_HOLD_TICKS) {
            f_jitter_active = 1;
            Frequency_jitter_state = JITTER_DECREASE;
            jitter_state_start_tick = system_ticks;
            PW0F_CLEAR;
            PWM_INTERRUPT_ENABLE;
        }
        break;

    case JITTER_DECREASE:
        if (elapsed >= JITTER_DEC_TICKS) {
            Frequency_jitter_state = JITTER_INCREASE;
            jitter_state_start_tick = system_ticks;
        }
        break;

    case JITTER_INCREASE:
        if (elapsed >= JITTER_INC_TICKS) {
            PWM_INTERRUPT_DISABLE;
            f_jitter_active = 0;
            f_jitter_in_progress = 0;
            Frequency_jitter_state = JITTER_HOLD;
            jitter_state_start_tick = system_ticks;
        }
        break;
        
    default:
        break;
  }
}

void Start_Frequency_jitter(void)
{   
    f_jitter_in_progress  = 1;
    jitter_adjust_cnt = 0;
    Frequency_jitter_state = JITTER_HOLD;
    jitter_state_start_tick = system_ticks;
}


