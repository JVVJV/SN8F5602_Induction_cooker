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
#include "system.h"
#include "temperature.h"
#include "comparator.h"
#include "PWM.h"
#include "config.h"
#include "power.h"
#include "communication.h"
#include "I2C.h"

/*_____ D E F I N I T I O N S ______________________________________________*/
#define SYSTEM_TICKS_PER_1MS    8     // 125us*8 = 1ms
#define SYSTEM_1MS_PER_SECOND   1000  // 1ms*1000 = 1s

#define SHUTDOWN_DURATION_SECONDS 60  // 60 seconds
#define SHUTDOWN_TEMP_THRESHOLD   50  // 50°C

/*_____ D E C L A R A T I O N S ____________________________________________*/
static uint16_t cntdown_timers[MAX_CNTDOWN_TIMERS]; // countdown timer array
volatile bit ISR_f_125us = 0;

//bit exit_shutdown_flag = 0;
//bit f_shutdown_in_progress = 0;     // shutdown in-progress flag
//static bit shutdown_triggered = 0;  // mark if shutdown logic is triggered

bit f_pot_detected = 0;
//bit f_pot_analyzed = 0;
//uint16_t recorded_1000W_PW0D = 0; // pot_analyze not use now.

bit f_block_occurred = 0;
bit f_power_switching = 0;

uint32_t power_setting = 0;
uint32_t target_power = 0;

volatile uint8_t system_ticks = 0;    // system tick counter (unit: 125 μs)
uint16_t system_time_1ms = 0;         // system time (unit: 1 ms)
uint16_t system_time_1s = 0;          // system time (unit: 1 s)

uint8_t measure_per_AC_cycle = MEASUREMENTS_PER_60HZ;

SystemState system_state = STANDBY;   // initial system state is standby, should not switch state in ISR, 
                                      // it may switch back in main loop. 

  #if TUNE_MODE == 1
//  uint16_t xdata tune_cnt = 0;
//  uint32_t xdata tune_record1 = 0;
//  uint32_t xdata tune_record2 = 0;
  #endif

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void Pot_Detection(void);
void Pot_Detection_In_Heating(void);
void shutdown_process(void);
void finalize_shutdown(void);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SystemCLK_Init(void)
{
  // 32MHz /2 = 16MHz
  CLKSEL = SYS_CLK_DIV2;
  CLKCMD = 0x69;
  
  // ILRC Calibration 
  CLKCAL |= 0x80;
  FRQCMD = 0x4B;
  while((CLKCAL&0x80) != 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Shutdown_Task(void)
{   
//  static uint32_t shutdown_start_time = 0;        // Shutdown start time (seconds)
//  uint8_t elapsed_time;

//  // Check whether system_state is SHUTTING_DOWN
//  if (system_state != SHUTTING_DOWN) {
//      return;  // Exit if not in SHUTTING_DOWN state
//  }

//  // Initialize shutdown logic
//  if (!shutdown_triggered) {
//      shutdown_start_time = system_time_1s;       // Record shutdown start time
//      shutdown_process();                        // Execute one-time shutdown logic
//      shutdown_triggered = 1;
//  }

//  // Check if shutdown escape is allowed
//  if (exit_shutdown_flag) {
//      exit_shutdown_flag = 0;  // Clear CMD flag
//      shutdown_triggered = 0;       // Reset shutdown logic
//      shutdown_start_time = 0;      // Reset start time
//      f_shutdown_in_progress = 0;    // End shutdown
//      return;                      // Exit the shutdown process
//  }

//  // Calculate elapsed time
//  elapsed_time = system_time_1s - shutdown_start_time;

//  // Determine whether shutdown conditions are met
//  if (elapsed_time >= SHUTDOWN_DURATION_SECONDS &&
//      IGBT_TEMP_C < SHUTDOWN_TEMP_THRESHOLD &&
//      TOP_TEMP_C < SHUTDOWN_TEMP_THRESHOLD) {
//      finalize_shutdown();             // Complete shutdown operations
//      system_state = STANDBY;          // Change system state to STANDBY
//      shutdown_triggered = 0;         // Reset shutdown logic
//      f_shutdown_in_progress = 0;        // Clear shutdown flag
//  }
}



/**
 * @brief  Initialize all countdown timers to zero.
 *
 * Sets every timer in the module to expired (value = 0).
 */
void CNTdown_Timer_Init() 
{
  uint8_t id;
    for (id = 0; id < MAX_CNTDOWN_TIMERS; id++) 
    {
        cntdown_timers[id] = 0; // Initialize all timers to 0
    }
}

/**
 * @brief   Start a countdown timer.
 * @param   timer_id  ID of the timer (0 .. MAX_CNTDOWN_TIMERS-1).
 * @param   duration  Initial countdown value in ticks (e.g., milliseconds).
 *
 * Sets the specified timer to the given duration. It will decrement
 * once per call to cntdown_timer_update().
 */
void cntdown_timer_start(uint8_t timer_id, uint16_t duration) {
        cntdown_timers[timer_id] = duration; // set initial timer value
}

/**
 * @brief   Check if a countdown timer has expired.
 * @param   timer_id  ID of the timer to check.
 * @return  1 if the timer has reached zero (expired), 0 otherwise.
 *
 * A timer is considered expired when its value is 0.
 */
uint8_t cntdown_timer_expired(uint8_t timer_id) {
    return (cntdown_timers[timer_id] == 0) ? 1 : 0;
}

/**
 * @brief  Update all countdown timers.
 * @note   Must be called at a fixed rate (e.g., in a 1 ms SysTick handler).
 *
 * Decrements each non-zero timer by one. Timers that reach zero remain at zero.
 */
void cntdown_timer_update() {
  uint8_t i;
    for (i = 0; i < MAX_CNTDOWN_TIMERS; i++) {
        if (cntdown_timers[i] > 0) {
            cntdown_timers[i]--; // decrement timer
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Update_System_Time() {
    static uint8_t last_1ms_ticks = 0;      // last recorded tick count for 1ms
    static uint16_t last_1s_time_1ms = 0;   // last recorded time for 1s

    // calculate 1 ms from system_ticks
    if ((system_ticks - last_1ms_ticks) >= SYSTEM_TICKS_PER_1MS) {
        system_time_1ms++;              // increase 1ms counter
        last_1ms_ticks = system_ticks;  // update last tick value
        cntdown_timer_update();         // update all countdown timers
    }

    // calculate 1 s from system_time_1ms
    if ((system_time_1ms - last_1s_time_1ms) >= SYSTEM_1MS_PER_SECOND) {
        system_time_1s++;                   // increase 1 second counter
        last_1s_time_1ms = system_time_1ms; // update last ms value
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t target_current = 0;        //  mA  HCW*** not use now

#define RATED_VOLTAGE_THRESHOLD     200     // Rated voltage threshold (V)
#define LOW_VOLTAGE_CURRENT_LIMIT   6000    // Max current when voltage is low (mA)
#define HIGH_POWER_LEVEL            2200000 // 2200K (mW)
#define PWM_ADJUST_QUICK_CHANGE     3       // Value to decrease PWM width when voltage changes quickly

uint16_t current_RMS_mA = 0;
uint16_t voltage_RMS_V = 0;
uint32_t current_power = 0;                 //mW

//HCW*** handle PW0D request in PWM0_ISR now. 
//void Increase_PWM_Width(uint8_t val) {
//  if ((PW0D + val) <= PWM_MAX_WIDTH) { // Ensure it does not exceed the maximum value
//      PW0D += val;
//  } else {
//      PW0D = PWM_MAX_WIDTH; // Prevent exceeding the maximum width
//  }
//}

//void Decrease_PWM_Width(uint8_t val) {
//  if ((PW0D >= (val + PWM_MIN_WIDTH))) { // Ensure it is not less than the minimum
//      PW0D -= val;
//  } else {
//      PW0D = PWM_MIN_WIDTH; // Prevent going below the minimum width
//  }
//}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile char PW0D_delta_req_pwr_ctrl = 0;
volatile bit PW0D_lock = 0;

void Power_Control(void)
{
    // check 4ms countdown timer
    if (!cntdown_timer_expired(CNTDOWN_TIMER_POWER_CONTROL)) {
        return; // not yet 4ms
    }
    // restart countdown timer
    cntdown_timer_start(CNTDOWN_TIMER_POWER_CONTROL, 4); // 4ms
  
    // If heating is not initialized, handle according to system_state
    if (!f_heating_initialized) {
        if (system_state == HEATING) {
            init_heating(HEATING_SYNC_AC, PULSE_WIDTH_MIN);
        }
        
        return;  // Exit if heating is not initialized (PERIODIC_HEATING case)
    }
    
    // If current exceeds 9.2A RMS, immediately request PWM decrease  HCW***
    if (current_RMS_mA > 9200) {
        PW0D_delta_req_pwr_ctrl = -1;
        PWM_INTERRUPT_ENABLE;    // enable PWM ISR for adjustment
        return;                  // skip further checks
    }
    
//    // When power exceeds high power threshold
//    if (current_power > HIGH_POWER_LEVEL) {
//        // Voltage changes rapidly, decrease PWM width
//        PW0D_delta_req_pwr_ctrl = -1;
//        PWM_INTERRUPT_ENABLE;   // enable PWM ISR for adjustment
//        return;                 // skip further checks
//    }
    
    #if BELOWRATED_VOLTAGE_CONSTANT_CURRENT_MODE == 1
    // Limit current when voltage is below the Rated Voltage
    if (voltage_RMS_V < RATED_VOLTAGE_THRESHOLD) {
        if (current_RMS_mA > LOW_VOLTAGE_CURRENT_LIMIT) {
            PW0D_delta_req_pwr_ctrl = -1;
            PWM_INTERRUPT_ENABLE;   // Force decrease PWM if current is too high
            return;
        }
    }
    #endif
    
//    // Patch for PW0D shrink to 0 by CM2SF HCW**
//    if(PW0D < PWM_MIN_WIDTH) 
//    {
//      PW0M &= ~mskPW0EN;
//      PW0D = PWM_MIN_WIDTH;
//      PW0M = mskPW0EN | PW0_DIV1 | PW0_HOSC | PW0_INVERS | mskPW0PO; // Enable PWM
//    }
    
//    // Compare target current with actual measured current HCW*** not use now
//    if (target_current > current_RMS_mA) {
//      Increase_PWM_Width(1); 
//    } else {
//      Decrease_PWM_Width(error_flags.f.Current_quick_large ? PWM_ADJUST_QUICK_CHANGE : 1);
//    }
    
    // Compare target power with actual measured current_power
    if (target_power > current_power) {
      PW0D_delta_req_pwr_ctrl = 1;
    } else {
      PW0D_delta_req_pwr_ctrl = -1;
    }
    PWM_INTERRUPT_ENABLE;   // Enable PWM ISR
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PWM_Request_Reset(void)
{
    PW0D_req_CMP2_isr       = FALSE;
    PW0D_delta_req_pwr_ctrl = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define POT_CONFIRM_INTERVAL  2   // Pot confirmation countdown time (2ms)
#define POT_CHECK_INTERVAL    500 // Pot detection interval countdown value (500ms)
#define POT_PULSE_THRESHOLD   30  // Threshold for pot missing
#define POT_PULSE_MIN         2   // Minimum pulses for valid pot check

PotDetectionState pot_detection_state = POT_IDLE;
volatile uint8_t pot_pulse_cnt = 0;   // pot pulse counter

void Pot_Detection() {  
  
  if (!cntdown_timer_expired(CNTDOWN_TIMER_POT_DETECT)) {
    return;   // detection interval not reached, return
  }
  
  if (pot_detection_state == POT_IDLE) {
    ISR_f_CM3_AC_sync = 0;      // Clear AC sync flag before waiting
    while (!ISR_f_CM3_AC_sync); // Block execution until AC low is detected
    f_block_occurred = 1;
  }
  
  switch (pot_detection_state) {
    case POT_IDLE:
      // Start pot detection
      // Start pot confirmation countdown
      cntdown_timer_start(CNTDOWN_TIMER_POT_DETECT, POT_CONFIRM_INTERVAL);
      
      pot_pulse_cnt = 0;  // reset pulse counter
      pot_detection_state = POT_CHECKING;
      
      // Patch: Protect for PWM accidentally switch mode
      PW0M &= ~mskPW0EN;    // Disable PWM
      CM0M &= ~mskCM0SF;    // Disable CM0 pulse trigger
      PW0D = POT_DETECT_PULSE_WIDTH ;   // set PW0D value for pot detection (6us)
      PW0Y = PWM_MAX_WIDTH;             // set PW0Y to max value as protection
      // Trigger pot detection
      PW0M = mskPW0EN | PW0_DIV1 | PW0_HOSC | PW0_INVERS | mskPW0PO; // set PWM0 to pulse mode
      PW0M1 |= (mskPGOUT);              // trigger pot detection pulse
      CLEAR_CM0_IRQ_FLAG;
      CM0_IRQ_ENABLE;
      break;
      
    case POT_CHECKING:
      // Wait for pot detection to complete
      if (!cntdown_timer_expired(CNTDOWN_TIMER_POT_DETECT)) {
        return;
      }
      // Pot detection complete, end logic
      CM0_IRQ_DISABLE;
      // Restart detection interval timer
      cntdown_timer_start(CNTDOWN_TIMER_POT_DETECT, POT_CHECK_INTERVAL);
        
      if (pot_pulse_cnt < POT_PULSE_MIN) {
        #if COIL_PROBLEM_DETECT == 1
          error_flags.f.Coil_problem = 1;  // Coil fault detected
        #else
          f_pot_detected = 1;
        #endif
      } else if (pot_pulse_cnt < POT_PULSE_THRESHOLD) {
        f_pot_detected = 1;
        
        //pot_detection_state = POT_ANALYZING;
        //f_pot_analyzed = 0;
        //Pot_Analyze();
      } else {
        //pot_detection_state = POT_IDLE;
        f_pot_detected = 0;
        error_flags.f.Pot_missing = 1;  // Raise missing pot flag
      }
      
      pot_detection_state = POT_IDLE;
      break;
      
    case POT_ANALYZING:
//      Pot_Analyze();

//      // If Pot_Analyze is completed
//      if (f_pot_analyzed == 1) {
//        pot_detection_state = POT_IDLE;
//      }
      break;
  }

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Checks pot presence during heating using current measurement.
 *
 *
 * Sets @ref error_flags.f.Pot_missing if current is below 
 * @ref POT_PRESENT_CURRENT_MIN_mA. Does not clear the flag here.
 * The flag will be cleared in @ref Error_Process when system enters ERROR state.
 */

#define POT_PRESENT_CURRENT_MIN_mA  600 // I_RMS mA 

void Pot_Detection_In_Heating(void) 
{
  if (!f_heating_initialized)
  {
    return;
  }
  
  if (f_power_updated)
  {
    if (current_RMS_mA < POT_PRESENT_CURRENT_MIN_mA)
    {
      error_flags.f.Pot_missing = 1;  // Pot not detected due to low current
      // Simply stop the heating logic
      P01 = 1;  //PWM Pin
      PW0M = 0;
      PWM_INTERRUPT_DISABLE;
    }
  }
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HCW*** Cancel the pot analysis process.

//#define POT_ANALYZE_POWER         1000000 // 1000 000mW
//#define DEFAULT_1000W_PW0D_VAL    320     // PWM width  320cnt @32MHz = 10us
//#define POWER_STABILITY_THRESHOLD 50000   // 50 000mW  
//#define POWER_STABLE_TIME         1000    // 1000ms
//#define POWER_SAMPLE_INTERVAL     100     // 100ms  

//PotAnalyzeState pot_analyze_state = PWR_UP;

//void Pot_Analyze(void) {
//    static uint8_t xdata record_count = 0;
//    static uint16_t xdata PW0D_val[4];  // 記錄 4 次 PWM 寬度
//    uint16_t sum;
//    uint8_t i;
//    
//    switch (pot_analyze_state) {
//      case PWR_UP:
//        // **第一步：開始加熱並進入穩定等待狀態**
//        target_power = POT_ANALYZE_POWER;
//        record_count = 0;  // 確保計數器歸零
//        system_state = HEATING;

//        // **啟動 1 秒倒數計時**
//        cntdown_timer_start(CNTDOWN_TIMER_POT_DETECT, POWER_STABLE_TIME);
//        pot_analyze_state = WAIT_STABILIZATION;
//        break;

//      case WAIT_STABILIZATION:        
//        // **等待 1 秒穩定**
//        if (!cntdown_timer_expired(CNTDOWN_TIMER_POT_DETECT)) {
//            return; // 等待時間未到，保持當前狀態
//        }

//        // **1 秒結束後，開始記錄**
//        cntdown_timer_start(CNTDOWN_TIMER_POT_DETECT, POWER_SAMPLE_INTERVAL); // 啟動 100ms 記錄計時
//        pot_analyze_state = RECORDING;
//        break;

//      case RECORDING:
//        // **等待 100ms 取樣間隔**
//        if (!cntdown_timer_expired(CNTDOWN_TIMER_POT_DETECT)) {
//            return;
//        }

//        // **重新啟動 100ms 記錄計時**
//        cntdown_timer_start(CNTDOWN_TIMER_POT_DETECT, POWER_SAMPLE_INTERVAL);

//        // **檢查功率是否穩定**
//        if((current_power >= (POT_ANALYZE_POWER-POWER_STABILITY_THRESHOLD)) &&  
//           (current_power <= (POT_ANALYZE_POWER+POWER_STABILITY_THRESHOLD))     )
//        { PW0D_val[record_count] = PW0D; }
//        else
//        { PW0D_val[record_count] = DEFAULT_1000W_PW0D_VAL; }
//        record_count++;
//        
//        // **當 4 次記錄完成後，計算平均**
//        if (record_count >= 4) {
//          // Align with the AC zero-crossing to prevent surge protection 
//          // from triggering due to rapid voltage rebound after pause_heating.
//          ISR_f_CM3_AC_sync = 0;
//          while(ISR_f_CM3_AC_sync == 0);
//          // 停止加熱
//          pause_heating();
//          
//          sum = 0;
//          for (i = 0; i < 4; i++) {
//              sum += PW0D_val[i];
//          }
//          recorded_1000W_PW0D = (sum>>2);  // 記錄平均值 = sum/4
//          if(recorded_1000W_PW0D > 640)
//          {
//            recorded_1000W_PW0D  = 640; //HCW**
//          }
//          // **分析完成設置 `f_pot_detected = 1`**
//          f_pot_detected = 1;
//          f_pot_analyzed = 1;
//          // **重置狀態，以便下次測量**
//          pot_analyze_state = PWR_UP;
//        }
//        break;
//    }
//    
//}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define AC_PERIOD_COUNT 4   // Average over 4 periods
#define DEBOUNCE_TICKS  2   // Software debounce time (2 * 125us = 250us)
#define PRE_CLEAR_DEBOUNCE_COUNT    3  // Require 3 consecutive HIGHs before clearing interrupt
#define PRE_CLEAR_DEBOUNCE_INTERVAL 2  // Check once per system_tick (250us)

uint8_t idata ac_low_periods[AC_PERIOD_COUNT] = {0};  //  Record the latest four AC periods

uint8_t ac_half_low_ticks_avg;

void Measure_AC_Low_Time(void) {
    uint8_t i;
    uint8_t sum = 0;
    uint8_t start_ticks;
  
//    // Wait ISR_f_CM3_AC_sync         // HCW*** It’s not necessary because, after applying the Warmup_Delay and the 4ms debounce in the CM3_ISR, 
//    while (ISR_f_CM3_AC_sync == 0);   // we can ensure the CM3_last_sync_tick is only set on the AC’s rising edge.
  
    // Wait for the AC signal to stabilize and collect four measurements
    for (i = 0; i < AC_PERIOD_COUNT; i++) {
     
      ISR_f_CM3_AC_sync = 0;  // clear ISR_f_CM3_AC_sync

      // Wait for the interrupt to occur
      while (ISR_f_CM3_AC_sync == 0);
      
      // Start timing
      start_ticks = system_ticks;

      // Software debounce: wait at least DEBOUNCE_TICKS before detecting
      while ((system_ticks - start_ticks) < DEBOUNCE_TICKS);
      
      // Wait for CM3 output to become LOW
      while (CMOUT & mskCM3OUT);
      
      // Record the time
      ac_low_periods[i]  = system_ticks - start_ticks;
      WDTR = 0x5A; // Clear watchdog
    }

    // Calculate the average of four periods
    for (i = 0; i < AC_PERIOD_COUNT; i++) {
        sum += ac_low_periods[i];
    }
    ac_half_low_ticks_avg = sum / (AC_PERIOD_COUNT*2);  // Average = sum / (AC_PERIOD_COUNT*2)
    
    if ((ac_half_low_ticks_avg>16) || (ac_half_low_ticks_avg<4))
    {ac_half_low_ticks_avg = 6;}
    
    ac_half_low_ticks_avg += 4;   // special modify for not symmetry divider circuit.
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define AC_FREQUENCY_SAMPLES  4   // Number of measurements to average
#define AC_60HZ_THRESHOLD     73  // 60Hz threshold （73 system_ticks）

bit f_AC_50Hz = 0; // AC frequency flag: 1 = 50Hz, 0 = 60Hz
uint8_t xdata ac_ticks[AC_FREQUENCY_SAMPLES];

void Detect_AC_Frequency(void) {
    uint8_t i;
    uint16_t sum = 0;
    uint8_t start_tick, elapsed_ticks;

    // clear the sync flag before starting measurements
    ISR_f_CM3_AC_sync = 0;
  
    for (i = 0; i < AC_FREQUENCY_SAMPLES; i++) {
        // wait for the new ISR_f_CM3_AC_sync
        while (!ISR_f_CM3_AC_sync);
        ISR_f_CM3_AC_sync = 0;  // clear flag
        
        // record the start time
        start_tick = system_ticks;

         // wait for the next ISR_f_CM3_AC_sync
        while (!ISR_f_CM3_AC_sync);
        ISR_f_CM3_AC_sync = 0;  // clear flag
        
        // compute elapsed ticks between ISR_f_CM3_AC_sync
        elapsed_ticks = system_ticks - start_tick;
        ac_ticks[i] = elapsed_ticks;
      
        WDTR = 0x5A; // Clear watchdog
    }

    // calculate the average tick count
    for (i = 0; i < AC_FREQUENCY_SAMPLES; i++) {
        sum += ac_ticks[i];
    }
    sum /= AC_FREQUENCY_SAMPLES;

    // === Determine AC Frequency (50Hz / 60Hz) ===
    #if AC_FREQ_MODE == AC_FREQ_MODE_FORCE_50HZ
        f_AC_50Hz = 1;
    #elif AC_FREQ_MODE == AC_FREQ_MODE_FORCE_60HZ
        f_AC_50Hz = 0;
    #else
        f_AC_50Hz = (sum >= AC_60HZ_THRESHOLD) ? 1 : 0;
    #endif
    
    // set measurements per AC cycle based on detected frequency
    measure_per_AC_cycle = (f_AC_50Hz) ? MEASUREMENTS_PER_50HZ : MEASUREMENTS_PER_60HZ;
    
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ERROR_RECOVERY_TIME_S  2  // Time required for error recovery (seconds)

ErrorFlags error_flags = {0};
WarningFlags warning_flags = {0};
volatile bit ISR_f_Surge_Overvoltage_error = 0;
volatile bit ISR_f_Surge_Overcurrent_error = 0;
volatile bit ISR_f_Unexpected_halt = 0;

void Error_Process(void)
{
  static uint8_t error_clear_time_1s  = 0; // Record the last time all errors were cleared
    
  // If system is not in ERROR state, check if it needs to enter ERROR
  if (system_state != ERROR) {
    if (  ISR_f_Surge_Overvoltage_error ||  \ 
          ISR_f_Surge_Overcurrent_error ||  \
          ISR_f_I2C_error ||                \
          ISR_f_Unexpected_halt ||          \
          error_flags.all_flags ) 
    {
      system_state = ERROR;
      
      error_clear_time_1s = system_time_1s;  // Record the current time
      
      // Stop_heating again
      stop_heating();

      // pot_detection & pot_analyze ini
      pot_detection_state = POT_IDLE;
      //pot_analyze_state = PWR_UP; // HCW** Cancel the pot analysis process.
      
      // Set I2C status code based on error type
      if (ISR_f_Surge_Overvoltage_error) {
          i2c_status_code = I2C_STATUS_OVERVOLTAGE;
      } else if (ISR_f_Surge_Overcurrent_error) {
          i2c_status_code = I2C_STATUS_OVERCURRENT;
      } else if (error_flags.f.IGBT_overheat) {
          i2c_status_code = I2C_STATUS_IGBT_OVERHEAT;
      } else if (error_flags.all_flags){
          i2c_status_code = I2C_STATUS_OTHER_ERROR;
      }
    }
    return;
  }

  // Check if Surge Overvoltage has recovered
  if (ISR_f_Surge_Overvoltage_error) {
      if (CMOUT & mskCM1OUT) {  // If CM1OUT returns to 1, voltage is back to normal
          ISR_f_Surge_Overvoltage_error = 0;
      }
  }
  
  // Check if Surge Overcurrent has recovered
  if (ISR_f_Surge_Overcurrent_error) {
      if (CMOUT & mskCM4OUT) {  // If CM4OUT returns to 1, current is back to normal
          ISR_f_Surge_Overcurrent_error = 0;
      }
  }
  
  // Clear ISR_f_I2C_error flag after entering ERROR state
  if (ISR_f_I2C_error) {
    ISR_f_I2C_error = 0;
  }
  
  // Clear Unexpected_halt flag after entering ERROR state
  if (ISR_f_Unexpected_halt) {
    ISR_f_Unexpected_halt = 0;
  }
  
  // Clear Pot_missing flag after entering ERROR state
  if (error_flags.f.Pot_missing) {
    error_flags.f.Pot_missing = 0;
  }

  // If any error flags are still active, update the error_clear_time_1s and return
  if (ISR_f_Surge_Overvoltage_error || ISR_f_Surge_Overcurrent_error || ISR_f_I2C_error || error_flags.all_flags) {
      error_clear_time_1s = system_time_1s;  // Record the current time
      return;
  }

  // All errors cleared, check if ERROR_RECOVERY_TIME_S seconds have passed
  if ((uint8_t)(system_time_1s - error_clear_time_1s) >= ERROR_RECOVERY_TIME_S) {
      system_state = STANDBY; // Maintain ERROR_RECOVERY_TIME_S seconds without errors before returning to STANDBY
      i2c_status_code = I2C_STATUS_NORMAL;  // clear status to normal
  }
  
}



