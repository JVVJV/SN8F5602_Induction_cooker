#ifndef __SYSTEM_H__
#define __SYSTEM_H__


/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>


/*_____ D E F I N I T I O N S ______________________________________________*/
// CLKSEL
#define SYS_CLK_DIV1    (0<<0)
#define SYS_CLK_DIV2    (1<<0)
#define SYS_CLK_DIV4    (2<<0)
#define SYS_CLK_DIV8    (3<<0)
#define SYS_CLK_DIV16   (4<<0)
#define SYS_CLK_DIV32   (5<<0)
#define SYS_CLK_DIV64   (6<<0)
#define SYS_CLK_DIV128  (7<<0)

#define MAX_CNTDOWN_TIMERS 5 // 支援最多 5 個倒數計時器

typedef union {
    struct {
      uint8_t IGBT_overheat : 1;
      uint8_t TOP_overheat : 1;
      uint8_t IGBT_sensor_fault : 1;
      uint8_t TOP_sensor_fault : 1;
      uint8_t Over_voltage : 1;
      uint8_t Low_voltage : 1;
      uint8_t Voltage_quick_change : 1;
      uint8_t Current_quick_large : 1;
      uint8_t Voltage_overshoot : 1;
      uint8_t Voltage_undershoot : 1;
    } f;
    uint16_t all_flags; // 用於快速檢查所有標誌
} ErrorFlags;


//typedef union {
//  struct {
//      uint8_t f_IGBT_overheat : 1;
//      uint8_t f_TOP_overheat : 1;
//      uint8_t f_IGBT_sensor_fault : 1;
//      uint8_t f_TOP_sensor_fault : 1;
//      uint8_t f_over_voltage : 1;
//      uint8_t f_low_voltage : 1;
//      uint8_t f_voltage_quick_change : 1;
//      uint8_t f_Current_quick_large : 1;
//  } bits;
//  uint8_t all_flags; // 用於快速檢查所有標誌
//} ErrorFlags0;

//typedef union {
//  struct {
//      uint8_t f_voltage_overshoot : 1;
//      uint8_t f_voltage_undershoot : 1;
//      //uint8_t f_IGBT_sensor_fault : 1;
//      //uint8_t f_TOP_sensor_fault : 1;
//      //uint8_t f_over_voltage : 1;
//      //uint8_t f_low_voltage : 1;
//      //uint8_t f_voltage_quick_change : 1;
//      //uint8_t f_Current_quick_large : 1;
//  } bits;
//  uint8_t all_flags; // 用於快速檢查所有標誌
//} ErrorFlags1;

typedef enum {
  TASK_HEAT_CONTROL,            // 加熱控制任務  
  TASK_POWER_CONTROL,           // 功率控制任務
  TASK_QUICK_CHANGE_DETECT,     // 快速變化檢測任務
  
  TASK_TEMP_MEASURE,        // 溫度測量任務
  TASK_TEMP_PROCESS,            // 溫度處理任務
    
  TASK_CURRENT_POT_CHECK,       // 電流檢鍋任務
  TASK_SHUTDOWN,                // 關機任務
  TASK_ERROR_PROCESS            // 錯誤處理任務
} TaskType;

typedef enum {
    STANDBY = 0,          // 待機狀態
    HEATING,              // 加熱中
    PERIODIC_HEATING,     // 間歇加熱中
    SHUTTING_DOWN,        // 關機狀態
    PAUSE,
    ERROR,
} SystemState;

#define CNTDOWN_TIMER_POT_DETECT      0 // 鍋具檢測間隔計時器id
#define CNTDOWN_TIMER_CURRENT_CHANGE  1 // 控制 3000ms 電流變化檢查倒數計時器 ID
#define CNTDOWN_TIMER_POWER_CONTROL   2 // 控制 5ms POWER_CONTROL
#define CNTDOWN_TIMER_I2C             3 // I2C 計時器 ID

/*_____ D E C L A R A T I O N S ____________________________________________*/
extern volatile bit f_125us;
extern volatile uint8_t system_ticks;
extern volatile uint8_t pot_pulse_cnt;   // 鍋具脈衝計數器
extern ErrorFlags error_flags;
extern SystemState system_state;
extern uint32_t power_setting;
extern uint32_t target_power;
extern TaskType current_task; // 當前任務
extern uint16_t voltage_IIR_new;         // 目前濾波後電壓
extern uint16_t current_IIR_new;         // 目前濾波後電流
extern uint32_t current_power;           // 目前功率
extern uint16_t current_RMS_mA;
extern uint16_t voltage_RMS_V;
extern uint16_t current_base;  // 存儲基準電流 ADC 值

extern uint16_t recorded_1000W_PW0D;
extern uint8_t ac_half_low_ticks_avg;

extern bit f_pot_detected;
extern bit f_AC_50Hz; // **AC 頻率標誌，1 = 50Hz，0 = 60Hz**
extern uint8_t measure_per_AC_cycle;


//DEBUG
extern  uint16_t tune_cnt;
extern uint32_t tune_record;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void SystemCLK_Init(void);
void CNTdown_Timer_Init(void);
uint8_t cntdown_timer_expired(uint8_t timer_id);
void cntdown_timer_start(uint8_t timer_id, uint16_t duration);

void Update_System_Time(void);
void shutdown_process(void);

void Quick_Change_Detect(void);
void Heat_Control(void);
void pause_heating(void);
void Periodic_Power_Control(void);
void Power_Control(void);
void Pot_Detection(void);
void Pot_Analyze(void);
void Pot_Detection_In_Heating(void);
void Shutdown_Task(void);
void Measure_AC_Low_Time(void);
void Detect_AC_Frequency(void);
void Error_Process(void);

#endif  // __SYSTEM_H__
