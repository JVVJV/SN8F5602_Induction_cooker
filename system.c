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
#include <math.h>
#include "comparator.h"
#include "PWM.h"
#include "config.h"
#include "power.h"

/*_____ D E F I N I T I O N S ______________________________________________*/
#define SYSTEM_TICKS_PER_1MS    8     // 125us*8 = 1ms
#define SYSTEM_1MS_PER_SECOND   1000  // 1ms*1000 = 1s

#define SHUTDOWN_DURATION_SECONDS 60 // 60秒
#define SHUTDOWN_TEMP_THRESHOLD 50 // 50度C

/*_____ D E C L A R A T I O N S ____________________________________________*/
static uint16_t cntdown_timers[MAX_CNTDOWN_TIMERS]; // 計時器數組
volatile bit f_125us = 0;
bit exit_shutdown_flag = 0;
bit f_shutdown_in_progress = 0;  // 關機進行標誌
bit f_pot_detected = 0;
bit f_pot_analyzed = 0;
uint16_t recorded_1000W_PW0D = 0;

uint32_t power_setting = 0;
uint32_t target_power = 0;            // 目標功率

volatile uint8_t system_ticks = 0;    // 系統計數 (125 μs 為單位)
uint16_t system_time_1ms = 0;         // 系統時間（1 ms 為單位）
uint16_t system_time_1s = 0;          // 系統時間（1 秒為單位

uint8_t measure_per_AC_cycle = MEASUREMENTS_PER_60HZ;

ErrorFlags error_flags;

SystemState system_state = STANDBY;  // 系統初始狀態為待機
  
  #if TUNE_MODE == 1
//  uint16_t tune_cnt = 0;
  uint32_t tune_record = 0;
  #endif

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void Pot_Detection(void);
void Pot_Detection_In_Heating(void);
void shutdown_process(void);
void finalize_shutdown(void);

static bit shutdown_triggered = 0;           // 標記是否已觸發關機邏輯
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SystemCLK_Init(void)
{
  // 32MHz /2 = 16MHz
  CLKSEL = SYS_CLK_DIV2;
  CLKCMD = 0x69;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Shutdown_Task(void)
{    
//  static uint32_t shutdown_start_time = 0;        // 關機開始時間（以秒為單位）
//  uint8_t elapsed_time;

//  // 檢查 system_state 是否為 SHUTTING_DOWN
//  if (system_state != SHUTTING_DOWN) {
//      return;  // 如果不在 SHUTTING_DOWN 狀態，直接退出
//  }

//  // 初始化關機邏輯
//  if (!shutdown_triggered) {
//      shutdown_start_time = system_time_1s;       // 記錄關機開始時間
//      shutdown_process();                        // 執行一次性關機邏輯
//      shutdown_triggered = 1;
//  }

//  // 檢查是否允許跳脫關機 !!
//  if (exit_shutdown_flag) {
//      exit_shutdown_flag = 0;  // 清除 CMD 標誌      
//      shutdown_triggered = 0;       // 重置關機邏輯
//      shutdown_start_time = 0;      // 重置開始時間
//      f_shutdown_in_progress = 0;    // 結束關機
//      return;                      // 跳脫關機行為
//  }

//  // 計算已過時間
//  elapsed_time = system_time_1s - shutdown_start_time;

//  // 判斷是否完成關機條件
//  if (elapsed_time >= SHUTDOWN_DURATION_SECONDS &&
//      IGBT_TEMP_C < SHUTDOWN_TEMP_THRESHOLD &&
//      TOP_TEMP_C < SHUTDOWN_TEMP_THRESHOLD) {
//      finalize_shutdown();             // 完成關機操作
//      system_state = STANDBY;          // 切換系統狀態為 STANDBY
//      shutdown_triggered = 0;         // 重置關機邏輯
//      f_shutdown_in_progress = 0;        // 清除關機標誌
//  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 初始化倒數計時模組
void CNTdown_Timer_Init() 
{
  uint8_t id;
    for (id = 0; id < MAX_CNTDOWN_TIMERS; id++) 
    {
        cntdown_timers[id] = 0; // 初始化所有計時器為 0
    }
}

// 開始倒數計時
void cntdown_timer_start(uint8_t timer_id, uint16_t duration) {
        cntdown_timers[timer_id] = duration; // 設置計時器初始值
}

// 檢查倒數計時器是否已到時間
uint8_t cntdown_timer_expired(uint8_t timer_id) {
    return (cntdown_timers[timer_id] == 0) ? 1 : 0;
}

// 更新所有倒數計時器（每 1ms 調用一次）
void cntdown_timer_update() {
  uint8_t i;
    for (i = 0; i < MAX_CNTDOWN_TIMERS; i++) {
        if (cntdown_timers[i] > 0) {
            cntdown_timers[i]--; // 遞減計時器
        }
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Update_System_Time() {
    static uint8_t last_1ms_ticks = 0;    // 上次更新 1ms 的 ticks 值
    static uint16_t last_1s_time_1ms = 0;  // 上次 1 秒更新的 1 ms 記錄值

    // 利用 system_ticks 減去已記錄值來計算 1 ms
    if ((system_ticks - last_1ms_ticks) >= SYSTEM_TICKS_PER_1MS) {
        system_time_1ms++;                   // 增加 1ms 計數
        last_1ms_ticks += SYSTEM_TICKS_PER_1MS; // 更新已記錄的 ticks

        // 更新所有倒數計時器
        cntdown_timer_update(); 
    }

    // 利用 system_time_1ms 減去已記錄值來計算 1 秒
    if ((system_time_1ms - last_1s_time_1ms) >= SYSTEM_1MS_PER_SECOND) {
        system_time_1s++;                     // 增加 1 秒計數
        last_1s_time_1ms += SYSTEM_1MS_PER_SECOND; // 更新已記錄的 1ms 時間
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define VOLTAGE_THRESHOLD 220          // 電壓門檻值 (220V)

#define HIGH_POWER_LEVEL 1500000       // 高功率標準 (1500mW)
#define PWM_ADJUST_QUICK_CHANGE 3      // 電壓快速變化時減少的 PWM 寬度

uint32_t current_power = 0;           // 目前功率 mW
uint16_t target_current = 0;          // 目標電流 mA
//uint16_t voltage_IIR_new = 0;         // 目前濾波後電壓
//uint16_t current_IIR_new = 0;         // 目前濾波後電流

uint32_t current_sum = 0;
uint32_t voltage_sum = 0;
uint16_t current_avg = 0;
uint16_t voltage_avg = 0;

// 檢查是否可以啟用電流變化檢查
void check_enable_current_change() {
    if (cntdown_timer_expired(CNTDOWN_TIMER_CURRENT_CHANGE)) {
        f_En_check_current_change = 1; // 啟用檢查電流變化
    }
}

// 增加 PWM 寬度
void Increase_PWM_Width(uint8_t val) {
//    // 保存 CM2SF 狀態
//    uint8_t cm2sf_status = CM2M & mskCM2SF;

//    // 關閉 CM2SF 功能
//    CM2M &= ~mskCM2SF;
 
    // 增加 PWM 寬度
    if ((PW0D + val) <= PWM_MAX_WIDTH) { // 確保不超過最大值
        PW0D += val;
    } else {
        PW0D = PWM_MAX_WIDTH; // 避免超過最大寬度
    }
    
//    // 恢復 CM2SF 狀態
//    CM2M |= cm2sf_status;
}

// 減少 PWM 寬度
void Decrease_PWM_Width(uint8_t val) {
//    // 保存 CM2SF 狀態
//    uint8_t cm2sf_status = CM2M & mskCM2SF;

//    // 關閉 CM2SF 功能
//    CM2M &= ~mskCM2SF;

  // 減少 PWM 寬度
  if ((PW0D >= (val + PWM_MIN_WIDTH))) { // 確保不小於最小值
      PW0D -= val;
  } else {
      PW0D = PWM_MIN_WIDTH; // 避免低於最小寬度
  }
  
//    // 恢復 CM2SF 狀態
//    CM2M |= cm2sf_status;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 功率控制函式
void Power_Control(void)
{
    uint8_t pwm_adjust_value;
  
    // 檢查 5ms 倒數計時器是否已到
    if (!cntdown_timer_expired(CNTDOWN_TIMER_POWER_CONTROL)) {
        return; // 若未到 5ms，直接返回
    }

    // 重啟倒數計時器
    cntdown_timer_start(CNTDOWN_TIMER_POWER_CONTROL, 5);
  
    // 僅在系統狀態為 HEATING 時執行
    if (system_state != HEATING) {
        return;
    }
    
    // 初始化加熱功能
    if (!f_heating_initialized) {
        init_heating(NORMAL);
    }

    // 檢查是否可以啟用電流變化檢查
    if (f_En_check_current_change) {
        check_enable_current_change();
    }

    // 當功率大於高功率標準時處理
    if (current_power > HIGH_POWER_LEVEL) {
        if (error_flags.f.Voltage_quick_change) {
            // 電壓快速變化，減少 PWM 寬度
            Decrease_PWM_Width(PWM_ADJUST_QUICK_CHANGE);
            return; // 直接返回，避免進一步處理
        }
    }
    
    // 判斷控制模式
    if (voltage_avg >= VOLTAGE_THRESHOLD) {
        // 恆電流控制模式
        target_current = target_power / VOLTAGE_THRESHOLD;
    } else {
        // 恆功率控制模式
        target_current = target_power / voltage_avg;
    }
    

    #if TUNE_MODE == 1
//    target_current = 6;
//    current_avg = 2;
    
//    target_current = 4;
//    current_avg = 8;
    #endif
     

    #if TUNE_MODE == 1
//    tune_cnt++;
//    if(current_power >= tune_record)
//    {
//      tune_record = current_power;
//    }
//    
//    if(tune_cnt >= 600)
//    {
//      PW0M = 0;
//      _nop_();
//      //while(1);
//    }
    #endif
    
    // Patch for PW0D shrink to 0 by CM2SF
    if(PW0D < PWM_MIN_WIDTH) 
    {
      PW0M &= ~mskPW0EN;
      PW0D = PWM_MIN_WIDTH;
      PW0M |= mskPW0EN;
    }
    

    // 比較目標電流與目前電流
    if (target_current > current_avg) {
      // 增加 PWM 寬度  
      Increase_PWM_Width(1); 
    } else {
      // 減少 PWM 寬度
      pwm_adjust_value = error_flags.f.Current_quick_large ? PWM_ADJUST_QUICK_CHANGE : 1; 
      Decrease_PWM_Width(pwm_adjust_value);
    }
    
    #if TUNE_MODE == 1
//    if(tune_cnt >= 600)
//    {
//      while(1);
//    }
    #endif
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define POT_CONFIRM_INTERVAL  2   // 鍋具確認倒數計時值 (2ms)
#define POT_CHECK_INTERVAL    300 // 鍋具檢測間隔倒數計時值 (300ms)
#define POT_PULSE_THRESHOLD   30  // 鍋具不存在的脈衝數門檻

volatile uint8_t pot_pulse_cnt = 0;   // 鍋具脈衝計數器

void Pot_Detection() {
  static enum {POT_IDLE, POT_CHECKING, POT_ANALYZING} pot_detection_state = POT_IDLE;
  
  switch (pot_detection_state) {
    case POT_IDLE:
      if (!cntdown_timer_expired(CNTDOWN_TIMER_POT_DETECT)) {
        return;  // 檢鍋間隔時間未到，直接返回
      }
      
      // **開始檢鍋**
      // 啟動鍋具確認倒數計時器
      cntdown_timer_start(CNTDOWN_TIMER_POT_DETECT, POT_CONFIRM_INTERVAL);
      pot_pulse_cnt = 0;  // 清零脈衝計數器
      pot_detection_state = POT_CHECKING;
      
      PW0M &= (~mskPW0EN);
      CM0M &= (~mskCM0SF);             // 關閉 CM0SF 功能
      PW0D = POT_DETECT_PULSE_TIME;    // 設定 PW0D 數值為檢鍋用值 (6us)
      PW0Y = PWM_MAX_WIDTH;             // 設定 PW0Y 為最大值 patch, 防止不小心反向
      // **觸發檢鍋**
      PW0M = mskPW0EN | PW0_DIV1 | PW0_HOSC | PW0_INVERS | mskPW0PO; // 設定 PWM0 為脈衝模式
      PW0M1 |= (mskPGOUT);             // 觸發檢鍋脈衝
      CLEAR_CM0_IRQ_FLAG;
      CM0_IRQ_ENABLE;
      
      return; // 初始化後立即返回
      
    case POT_CHECKING:
      // **等待檢鍋完成**
      if (!cntdown_timer_expired(CNTDOWN_TIMER_POT_DETECT)) {
        return;
      }
      // 鍋具確認完成，結束檢鍋邏輯
      //CM0_IRQ_DISABLE;    //HCW** should not comment
      // 重啟檢鍋間隔計時器
      cntdown_timer_start(CNTDOWN_TIMER_POT_DETECT, POT_CHECK_INTERVAL);
        
      // **如果鍋具存在，轉入 `POT_ANALYZING`**
      if (pot_pulse_cnt < POT_PULSE_THRESHOLD) {
        pot_detection_state = POT_ANALYZING;
        f_pot_analyzed = 0;
        Pot_Analyze();
      } else {  // **鍋具不存在，保持在 `POT_IDLE`，下次重新檢測**
        pot_detection_state = POT_IDLE;
        f_pot_detected = 0;  // **重置鍋具檢測狀態**
      }
      break;
      
    case POT_ANALYZING:
      Pot_Analyze();

      // **若 Pot_Analyze完成
      if (f_pot_analyzed == 1) {
        pot_detection_state = POT_IDLE;
      }
      break;
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Pot_Detection_In_Heating(void)
{
    static bit f_current_pot_checking = 0; // 是否正在倒數的旗標
  
    // 僅在加熱狀態下執行檢鍋任務
    if (system_state != HEATING) {
        return; // 非 HEATING 狀態時直接返回
    }

    // 鍋具檢測邏輯
    #define POT_CURRENT_THRESHOLD 500   // 鍋具電流檢測門檻 (假設單位 mA)
    #define POT_CHECK_DELAY_MS 30       // 檢測門檻延遲時間 (30ms)

//    if (current_IIR_new < POT_CURRENT_THRESHOLD) {
//        // 電流低於門檻
//        if (!f_current_pot_checking) {
//            // 若未開始倒數，啟動倒數
//            cntdown_timer_start(CNTDOWN_TIMER_POT_CHECK, POT_CHECK_DELAY_MS);
//            f_current_pot_checking = 1; // 設置為正在倒數中
//        } else if (cntdown_timer_expired(CNTDOWN_TIMER_POT_CHECK)) {
//            // 若倒數完成，判定鍋具不存在
//            f_current_pot_checking = 0; // 清除倒數旗標
//            f_pot_detected = 0;        // 鍋具不存在
//            system_state = STANDBY;    // 切換系統狀態為待機
//        }
//    } else {
//        // 電流恢復正常
//        f_current_pot_checking = 0; // 清除倒數旗標
//        cntdown_timer_start(CNTDOWN_TIMER_POT_CHECK, 0); // 停止計時器
//    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define POT_ANALYZE_POWER   1000000   // 1000 000mW
#define DEFAULT_1000W_PW0D_VAL  320   // PWM 寬度 320cnt @32MHz = 10us
#define POWER_STABILITY_THRESHOLD  50000  // 50 000mW  
#define POWER_STABLE_TIME       1000   // 1000ms
#define POWER_SAMPLE_INTERVAL   100    // 100ms  

void Pot_Analyze(void) {
    static uint8_t xdata record_count = 0;
    static uint16_t xdata PW0D_val[4];  // 記錄 4 次 PWM 寬度
    static enum {PWR_UP, WAIT_STABILIZATION, RECORDING} xdata state = PWR_UP;
    uint16_t sum;
    uint8_t i;
    
//    // **如果已完成分析，則不再執行**
//    if (f_pot_analyzed == 1) {
//        return;
//    }
    
    switch (state) {
      case PWR_UP:
        // **第一步：開始加熱並進入穩定等待狀態**
        target_power = POT_ANALYZE_POWER;
        record_count = 0;  // 確保計數器歸零
        system_state = HEATING;

        // **啟動 1 秒倒數計時**
        cntdown_timer_start(CNTDOWN_TIMER_POT_DETECT, POWER_STABLE_TIME);
        state = WAIT_STABILIZATION;
        break;

      case WAIT_STABILIZATION:        
        // **等待 1 秒穩定**
        if (!cntdown_timer_expired(CNTDOWN_TIMER_POT_DETECT)) {
            return; // 等待時間未到，保持當前狀態
        }

        // **1 秒結束後，開始記錄**
        cntdown_timer_start(CNTDOWN_TIMER_POT_DETECT, POWER_SAMPLE_INTERVAL); // 啟動 100ms 記錄計時
        state = RECORDING;
        break;

      case RECORDING:
        // **等待 100ms 取樣間隔**
        if (!cntdown_timer_expired(CNTDOWN_TIMER_POT_DETECT)) {
            return;
        }

        // **重新啟動 100ms 記錄計時**
        cntdown_timer_start(CNTDOWN_TIMER_POT_DETECT, POWER_SAMPLE_INTERVAL);

        // **檢查功率是否穩定**
        if((current_power >= (POT_ANALYZE_POWER-POWER_STABILITY_THRESHOLD)) &&  \
           (current_power <= (POT_ANALYZE_POWER+POWER_STABILITY_THRESHOLD))     )
        { PW0D_val[record_count] = PW0D; }
        else
        { PW0D_val[record_count] = DEFAULT_1000W_PW0D_VAL; }
        record_count++;
        
        // **當 4 次記錄完成後，計算平均**
        if (record_count >= 4) {
          // 停止加熱
          pause_heating();
          
          sum = 0;
          for (i = 0; i < 4; i++) {
              sum += PW0D_val[i];
          }
          recorded_1000W_PW0D = (sum>>2);  // 記錄平均值 = sum/4
          if(recorded_1000W_PW0D > 640)
          {
            recorded_1000W_PW0D  = 640; //HCW**
          }
          // **分析完成設置 `f_pot_detected = 1`**
          f_pot_detected = 1;
          f_pot_analyzed = 1;
          // **重置狀態，以便下次測量**
          state = PWR_UP;
        }
        break;
    }
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define AC_PERIOD_COUNT 4  // 取 4 次週期平均
#define DEBOUNCE_TICKS 2   // 軟體 debounce 時間 (2 * 125us = 250us)
#define PRE_CLEAR_DEBOUNCE_COUNT 3  // 清除中斷前需要 3 次連續確認 HIGH
#define PRE_CLEAR_DEBOUNCE_INTERVAL 2  // 每 1 個 `system_ticks` 確認一次 (250us)

uint8_t ac_low_periods[AC_PERIOD_COUNT] = {0};  // 記錄最近 4 次 AC 週期時間

uint8_t ac_half_low_ticks_avg;

void Measure_AC_Low_Time(void) {
    uint8_t i;
    uint8_t sum = 0;
    uint8_t start_ticks;
//  uint8_t low_count;

//    // **等待中斷發生**           // It’s not necessary because, after applying the Warmup_Delay and the 4ms debounce in the CM1_ISR, 
//    while (f_CM1_AC_sync == 0);   // we can ensure the f_CM1_AC_sync is only set on the AC’s rising edge.
  
    // 等待 AC 訊號穩定，收集 4 次數據
    for (i = 0; i < AC_PERIOD_COUNT; i++) {
      
//      // 軟體 debounce：確保 CM1 真的回到 LOW，才清除f_CM1_AC_sync
//      low_count = 0;
//      while (low_count < PRE_CLEAR_DEBOUNCE_COUNT) {
//        debounce_start = system_ticks;
//        while ((system_ticks - debounce_start) < PRE_CLEAR_DEBOUNCE_INTERVAL); // 等待 `250us`
//        
//        if ((CMOUT & mskCM1OUT) == 0) {
//            low_count++;
//        } else {
//            low_count = 0;  // 若 `CM1` 變 LOW，則重新計算
//        }
//      }
      
      f_CM1_AC_sync = 0;  // **清除中斷標誌**

      // **等待中斷發生**
      while (f_CM1_AC_sync == 0);
      
      
      // **開始計時**
      start_ticks = system_ticks;

      // 軟體 debounce：等待至少 `DEBOUNCE_TICKS` 再開始偵測**
      while ((system_ticks - start_ticks) < DEBOUNCE_TICKS);
      
      // **等待 CM1 輸出轉為 LOW**
      while (CMOUT & mskCM1OUT);
      
      // **記錄時間**
      ac_low_periods[i]  = system_ticks - start_ticks;
    }

    // 計算 4 次週期的平均值
    for (i = 0; i < AC_PERIOD_COUNT; i++) {
        sum += ac_low_periods[i];
    }
    ac_half_low_ticks_avg = sum / (AC_PERIOD_COUNT*2);  // 平均值 = sum / (AC_PERIOD_COUNT*2)
    
    if ((ac_half_low_ticks_avg>16) || (ac_half_low_ticks_avg<4))
    {ac_half_low_ticks_avg = 6;}
    
    ac_half_low_ticks_avg += 3;   // special modify for not symmetry divider circuit.
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define AC_FREQUENCY_SAMPLES 4   // 取 4 次測量平均
#define AC_60HZ_THRESHOLD  73    // 60Hz 判斷門檻（73 system_ticks）

bit f_AC_50Hz = 0; // **AC 頻率標誌，1 = 50Hz，0 = 60Hz**
uint8_t xdata ac_ticks[AC_FREQUENCY_SAMPLES];

void Detect_AC_Frequency(void) {
    uint8_t i;
    uint16_t sum = 0;
    uint8_t start_tick, elapsed_ticks;

    f_CM1_AC_sync = 0;
  
    for (i = 0; i < AC_FREQUENCY_SAMPLES; i++) {
        // **等待 f_CM1_AC_sync 立起**
        while (!f_CM1_AC_sync);
        f_CM1_AC_sync = 0;  // **清除標誌**
        
        // **記錄開始時間**
        start_tick = system_ticks;

        // **等待下一次 f_CM1_AC_sync 立起**
        while (!f_CM1_AC_sync);
        f_CM1_AC_sync = 0;  // **清除標誌**
        
        // **計算經過的 ticks**
        elapsed_ticks = system_ticks - start_tick;
        ac_ticks[i] = elapsed_ticks;  // **存入 xdata 陣列**
    }

    // **計算 4 次測量的平均值**
    for (i = 0; i < AC_FREQUENCY_SAMPLES; i++) {
        sum += ac_ticks[i];
    }
    sum /= AC_FREQUENCY_SAMPLES;

    // **判斷 AC 頻率 (50Hz / 60Hz)**
    f_AC_50Hz = (sum >= AC_60HZ_THRESHOLD) ? 1 : 0;
    
    measure_per_AC_cycle = (f_AC_50Hz) ? MEASUREMENTS_PER_50HZ : MEASUREMENTS_PER_60HZ;
    
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ErrorFlags error_flags;

void Error_Process(void)
{
  if (system_state == ERROR) {
      // 檢查是否需要重置系統狀態
      if (error_flags.all_flags == 0) {
          system_state = STANDBY; // 所有錯誤標誌清除，切換為 STANDBY
      }
  }

  // 若必要，可添加其他錯誤處理邏輯
}

