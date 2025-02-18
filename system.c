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

uint32_t power_setting = 0;
uint32_t target_power = 0;            // 目標功率

volatile uint8_t system_ticks = 0;    // 系統計數 (125 μs 為單位)
uint16_t system_time_1ms = 0;        // 系統時間（1 ms 為單位）
uint16_t system_time_1s = 0;          // 系統時間（1 秒為單位

ErrorFlags error_flags;

SystemState system_state = STANDBY;  // 系統初始狀態為待機
  
  #if TUNE_MODE == 1
  uint16_t tune_cnt = 0;
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
  static uint32_t shutdown_start_time = 0;        // 關機開始時間（以秒為單位）
  uint8_t elapsed_time;

  // 檢查 system_state 是否為 SHUTTING_DOWN
  if (system_state != SHUTTING_DOWN) {
      return;  // 如果不在 SHUTTING_DOWN 狀態，直接退出
  }

  // 初始化關機邏輯
  if (!shutdown_triggered) {
      shutdown_start_time = system_time_1s;       // 記錄關機開始時間
      shutdown_process();                        // 執行一次性關機邏輯
      shutdown_triggered = 1;
  }

  // 檢查是否允許跳脫關機 !!
  if (exit_shutdown_flag) {
      exit_shutdown_flag = 0;  // 清除 CMD 標誌      
      shutdown_triggered = 0;       // 重置關機邏輯
      shutdown_start_time = 0;      // 重置開始時間
      f_shutdown_in_progress = 0;    // 結束關機
      return;                      // 跳脫關機行為
  }

  // 計算已過時間
  elapsed_time = system_time_1s - shutdown_start_time;

  // 判斷是否完成關機條件
  if (elapsed_time >= SHUTDOWN_DURATION_SECONDS &&
      IGBT_TEMP_C < SHUTDOWN_TEMP_THRESHOLD &&
      TOP_TEMP_C < SHUTDOWN_TEMP_THRESHOLD) {
      finalize_shutdown();             // 完成關機操作
      system_state = STANDBY;          // 切換系統狀態為 STANDBY
      shutdown_triggered = 0;         // 重置關機邏輯
      f_shutdown_in_progress = 0;        // 清除關機標誌
  }
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

volatile uint8_t f_voltage_quick_change = 0;    // 電壓快速變化標誌

uint32_t current_power = 0;           // 目前功率 mW
uint32_t target_current = 0;          // 目標電流 mA
//uint16_t voltage_IIR_new = 0;         // 目前濾波後電壓
//uint16_t current_IIR_new = 0;         // 目前濾波後電流

uint32_t current_sum = 0;
uint32_t voltage_sum = 0;
uint16_t current_avg = 0;
uint16_t voltage_avg = 0;
      

// 更新目前功率
void update_current_power() {
//    current_power = (uint32_t)voltage_IIR_new * current_IIR_new / 1000; // 瓦特值轉換
}


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
        init_heating();
    }

    // 檢查是否可以啟用電流變化檢查
    if (f_En_check_current_change) {
        check_enable_current_change();
    }

    // 當功率大於高功率標準時處理
    if (current_power > HIGH_POWER_LEVEL) {
        if (f_voltage_quick_change) {
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
    if(current_power >= tune_record)
    {
      tune_record = current_power;
    }
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
#define POT_CONFIRM_DURATION  2   // 鍋具確認倒數計時值 (2ms)
#define POT_CHECK_INTERVAL    300 // 鍋具檢測間隔倒數計時值 (300ms)
#define POT_PULSE_THRESHOLD   30  // 鍋具不存在的脈衝數門檻

bit pot_detecting = 0;    // 檢鍋進行中標誌
volatile uint8_t pot_pulse_cnt = 0;   // 鍋具脈衝計數器

void Pot_Detection() {
    if (pot_detecting == 0) {
        // 檢查檢鍋間隔是否已到
        if (!cntdown_timer_expired(CNTDOWN_TIMER_POT_CHECK)) {
            return; // 檢鍋間隔時間未到，直接返回
        }
        //P00 =1; //HCW**

        // 啟動鍋具確認倒數計時器
        cntdown_timer_start(CNTDOWN_TIMER_POT_CONFIRM, POT_CONFIRM_DURATION);

        // 啟動檢鍋操作
        pot_pulse_cnt = 0; // 清零脈衝計數器
        pot_detecting = 1;

        // 增加硬體相關設定
        CM0M &= (~mskCM0SF);             // 關閉 CM0SF 功能
        PW0D = POT_DETECT_PULSE_TIME;    // 設定 PW0D 數值為檢鍋用值 (6us)
        PW0M &= (~mskPW0EN);
        PW0Y = 0xFFFF;                   // 設定 PW0Y 為最大值
        PW0M |= (mskPW0EN | mskPW0PO);   // 設定 PWM0 為脈衝模式
        PW0M1 |= (mskPGOUT);             // 觸發檢鍋脈衝
        CLEAR_CM0_IRQ_FLAG;
        CM0_IRQ_ENABLE;
        return; // 初始化後立即返回
    }

    if (pot_detecting == 1) {
        // 如果鍋具確認計時器尚未到，直接返回
        if (!cntdown_timer_expired(CNTDOWN_TIMER_POT_CONFIRM)) {
            return; // 鍋具確認尚未完成
        }

        // 鍋具確認完成，結束檢鍋邏輯
        pot_detecting = 0;
        
        // 確認鍋具結果
        f_pot_detected = (pot_pulse_cnt < POT_PULSE_THRESHOLD) ? 1 : 0;
        CM0_IRQ_DISABLE;
        
        //P00 =0; //HCW**
        
        // 重啟檢鍋間隔計時器
        cntdown_timer_start(CNTDOWN_TIMER_POT_CHECK, POT_CHECK_INTERVAL);
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

