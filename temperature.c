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
#include "temperature.h"
#include "config.h"
#include "ADC.h"
#include "system.h"
#include "power.h"
#include "buzzer.h"

/*_____ D E F I N I T I O N S ______________________________________________*/
#define TEMP_ACCUMULATE_COUNT 8
//#define TEMP_LOG_SIZE 50

/*_____ D E C L A R A T I O N S ____________________________________________*/
uint16_t idata IGBT_TEMP_code = 0;        // 目前IGBT溫度 AD_code
uint16_t idata TOP_TEMP_code = 0;         // 目前表面溫度 AD_code
int idata IGBT_TEMP_C = 0;                // 目前IGBT溫度 度C
int idata TOP_TEMP_C = 0;                 // 目前表面溫度 度C
static bit f_temp_updated = 0;            // 溫度變數更新旗標

//int xdata igbt_temp_log[TEMP_LOG_SIZE];
//unsigned char idata temp_log_index = 0;
static uint16_t last_log_time_s = 0;  // 每秒 log 時間戳

// === NTC 監控用變數 ===
static bit f_NTC_monitoring;
static uint8_t ntc_change_count;
static idata int last_IGBT_temp = 0;
static uint32_t last_power_setting = 0;

// 每秒取樣用變數
uint8_t ntc_last_sample_time_s = 0;


/**
* The NTC table has 129 interpolation points.
* Unit: 1°C
* Description: 	NTC with pull up resistor
*				NTC resistance at 25°C 	  :100K
*				Pullup-resistance 	      :20K
*				BETA of NTC 			        :3950
*				ADC Resolution 			      :12bit
* Table Generator: https://www.sebulli.com/ntc/
*/
const int code IGBT_NTC_table[129] = {
	362, 308, 254, 226, 208, 195, 185, 176, 169, 
  163, 158, 153, 148, 144, 141, 137, 134, 131, 
  129, 126, 124, 121, 119, 117, 115, 113, 111, 
  110, 108, 106, 105, 103, 102, 100, 99, 97, 
  96, 95, 93, 92, 91, 90, 88, 87, 86, 85, 84, 
  83, 82, 81, 80, 79, 78, 77, 76, 75, 74, 73, 
  72, 71, 70, 69, 68, 67, 66, 65, 64, 64, 63, 
  62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 
  52, 51, 50, 49, 48, 47, 47, 46, 45, 44, 43, 
  42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 
  31, 29, 28, 27, 26, 25, 23, 22, 21, 19, 18, 
  16, 14, 13, 11, 9, 7, 5, 2, -1, -4, -7, -11, 
  -16, -23, -34, -45
};

/**
* The NTC table has 129 interpolation points.
* Unit: 1°C
* Description: 	NTC with pull down resistor
*				NTC resistance at 25°C 	  :100K
*				Pulldown-resistance 	    :3K
*				BETA of NTC 			        :3950
*				ADC Resolution 			      :12bit
* Table Generator: https://www.sebulli.com/ntc/
*/
const int code TOP_NTC_table[129] = {
	-16, -2, 12, 20, 27, 32, 37, 41, 44, 47, 
  50, 53, 56, 58, 61, 63, 65, 67, 69, 71, 73, 
  74, 76, 78, 79, 81, 82, 84, 86, 87, 88, 90, 
  91, 93, 94, 95, 97, 98, 99, 101, 102, 103, 
  105, 106, 107, 108, 110, 111, 112, 113, 115, 
  116, 117, 118, 120, 121, 122, 123, 125, 126, 
  127, 128, 130, 131, 132, 134, 135, 136, 138, 
  139, 140, 142, 143, 144, 146, 147, 149, 150, 
  152, 153, 155, 156, 158, 160, 161, 163, 165, 
  166, 168, 170, 172, 174, 176, 178, 180, 182, 
  184, 186, 188, 191, 193, 196, 199, 201, 204, 
  207, 210, 214, 217, 221, 225, 229, 234, 239, 
  244, 249, 256, 262, 270, 279, 289, 300, 314, 
  332, 354, 385, 434, 539, 644
};

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void Temp_Measure(void)
{
  uint16_t temp_raw;
  static uint16_t idata IGBT_TEMP_sum = 0;      // 累加 IGBT 溫度的變數
  static uint16_t idata TOP_TEMP_sum = 0;       // 累加表面溫度的變數
  static uint8_t idata Temp_Measure_cnt = 0;    // 量測次數計數器  
  
  // 呼叫通用 ADC 量測函式，量測 IGBT 溫度
  ADC_measure_4_avg(IGBT_TEMP_ADC_CHANNEL, &temp_raw);
  IGBT_TEMP_sum += temp_raw;

  // 呼叫通用 ADC 量測函式，量測表面溫度
  ADC_measure_4_avg(TOP_TEMP_ADC_CHANNEL, &temp_raw);
  
  #if ICE_DEBUG_MODE == 1
  temp_raw = 124;
  #endif
  TOP_TEMP_sum += temp_raw;

  // 增加量測次數
  Temp_Measure_cnt++;

  // 每 8 次執行一次平均計算並更新變數
  if (Temp_Measure_cnt >= TEMP_ACCUMULATE_COUNT) {
    IGBT_TEMP_code = IGBT_TEMP_sum>>3;    // 更新IGBT溫度 sum/8
    TOP_TEMP_code = TOP_TEMP_sum>>3;      // 更新表面溫度 sum/8

    // 設置旗標，表示溫度變數已更新
    f_temp_updated = 1;

    // 重置累加器與計數器
    IGBT_TEMP_sum = 0;
    TOP_TEMP_sum = 0;
    Temp_Measure_cnt = 0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// IGBT temperature error threshold
#define IGBT_TEMP_UPPER_LIMIT  110  // IGBT 溫度上限 (攝氏度) 110
#define IGBT_TEMP_RECOVERY     80   // IGBT Overheat recovery threshold 80
#define IGBT_TEMP_LOWER_LIMIT -20   // IGBT 溫度下限 (攝氏度)

// Warning temperature threshold before reaching overheat
#define IGBT_TEMP_WARNING_LIMIT      55  // Start of heat warning zone        HCW***
#define IGBT_TEMP_WARNING_RECOVERY   50  // Heat warning recovery threshold   HCW***

// IGBT temperature error threshold
#define TOP_TEMP_UPPER_LIMIT  180   // TOP 溫度上限 (單位：適當的測量單位，例如攝氏度)
#define TOP_TEMP_LOWER_LIMIT  -20   // TOP 溫度下限 (單位：適當的測量單位，例如攝氏度)
#define TOP_TEMP_RECOVERY     100   // TOP Overheat release temperature


void Temp_Process(void)
{
  int p1,p2;
  static uint8_t elapsed;
  
  // 檢查是否有更新的溫度數據
  if (f_temp_updated) {
    f_temp_updated = 0;  // 清除溫度更新旗標
    
    //IGBT_TEMP_code += TMP_CAL_Offset;
  
    // IGBT temperature process
    /* Estimate the interpolating point before and after the ADC value. */
    p1 = IGBT_NTC_table[ (IGBT_TEMP_code >> 5)  ];
    p2 = IGBT_NTC_table[ (IGBT_TEMP_code >> 5)+1];
    
    /* Interpolate between both points. */
    IGBT_TEMP_C = p1 - (((p1-p2)*(IGBT_TEMP_code & 0x001F)) >> 5);  // = p1 + ( (p2-p1) * (IGBT_TEMP_code & 0x001F) ) / 32
  
    // Start NTC monitoring on power change
    if (power_setting != last_power_setting && f_heating_initialized) 
    {
        last_power_setting = power_setting;
      
        f_NTC_monitoring = 1;
        ntc_change_count = 0;
        last_IGBT_temp = IGBT_TEMP_C;
        ntc_last_sample_time_s = (uint8_t)system_time_1s;
    }
    
//	  // 每1秒記錄1次IGBT_TEMP_C
//		if (system_time_1s != last_log_time_s) {
//				last_log_time_s = system_time_1s;
//				igbt_temp_log[temp_log_index] = IGBT_TEMP_C;
//				temp_log_index = (temp_log_index + 1) % TEMP_LOG_SIZE;
//		}

    // Overheat protection
    if (IGBT_TEMP_C > IGBT_TEMP_UPPER_LIMIT) {
      error_flags.f.IGBT_overheat = 1;
    } else if (IGBT_TEMP_C < IGBT_TEMP_RECOVERY) {
      error_flags.f.IGBT_overheat = 0;
    }

    // Sensor fault1 detection, Abnormal value
    if (IGBT_TEMP_C < IGBT_TEMP_LOWER_LIMIT) {
      error_flags.f.IGBT_sensor_fault1 = 1;
    } else {
      error_flags.f.IGBT_sensor_fault1 = 0;
    }
    
    // Sensor fault2 detection, Temperature not changed
    if (f_NTC_monitoring) 
    {
      elapsed = (uint8_t)system_time_1s - ntc_last_sample_time_s;
      if (elapsed < 30) 
      {
        // 只有在時間進位時（1秒鐘後）才記錄
        if ((uint8_t)system_time_1s != ntc_last_sample_time_s) 
        {
          ntc_last_sample_time_s = system_time_1s; // 更新上次樣本時間
          if (IGBT_TEMP_C != last_IGBT_temp) 
              ntc_change_count++;
          
          last_IGBT_temp = IGBT_TEMP_C; // 更新上次溫度
        }
      }
      else 
      {
        f_NTC_monitoring = 0;
        if (ntc_change_count <= 5)
          error_flags.f.IGBT_sensor_fault2 = 1;
      }
    }
    
    // Heat warning zone and fan control
    if (IGBT_TEMP_C > IGBT_TEMP_WARNING_LIMIT) {
      if (!warning_flags.f.IGBT_heat_warning) { 
        Fan_SetFullSpeed(); 
      }
      warning_flags.f.IGBT_heat_warning = 1;
    } else if (IGBT_TEMP_C < IGBT_TEMP_WARNING_RECOVERY) {
      if (warning_flags.f.IGBT_heat_warning) { 
        Fan_SetNormalSpeed(); 
      }
      warning_flags.f.IGBT_heat_warning = 0;
    }
    
    // TOP temperature process
    /* Estimate the interpolating point before and after the ADC value. */
    p1 = TOP_NTC_table[ (TOP_TEMP_code >> 5)  ];
    p2 = TOP_NTC_table[ (TOP_TEMP_code >> 5)+1];
    
    /* Interpolate between both points. */
    TOP_TEMP_C = p1 + (((p2-p1)*(TOP_TEMP_code & 0x001F)) >> 5);  // = p1 + ( (p2-p1) * (IGBT_TEMP_code & 0x001F) ) / 32
    
    // 檢查溫度是否異常 TOP_Temp_Process
    if (TOP_TEMP_C > TOP_TEMP_UPPER_LIMIT) {
      error_flags.f.TOP_overheat = 1;
    } else if (TOP_TEMP_C < TOP_TEMP_RECOVERY) {
      error_flags.f.TOP_overheat = 0;
    }

    if (TOP_TEMP_C < TOP_TEMP_LOWER_LIMIT) {
      error_flags.f.TOP_sensor_fault = 1;
    } else {
      error_flags.f.TOP_sensor_fault = 0;
    }
  }
}


//void Print_All_IGBT_Temps(void)
//{
//	uint8_t i;
//    // idx=0 最舊、idx=49 最新
//    for (i = 0; i < TEMP_LOG_SIZE; i++) {
//        // 計算環狀緩衝裡位置
//        int latest_pos = (temp_log_index + TEMP_LOG_SIZE - 1) % TEMP_LOG_SIZE;
//        int pos = (latest_pos + TEMP_LOG_SIZE - i) % TEMP_LOG_SIZE;
//        int temp = igbt_temp_log[pos];

//        // 這裡用 printf，或改成你的顯示方式
//        //printf("IGBT_TEMP_LOG[%2d] = %d\n", i, temp);
//    }
//}