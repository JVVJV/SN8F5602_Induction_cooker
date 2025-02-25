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


#define SYSTEM_TICKS_PER_10MS 80  // �C 10 ms ���p�� (125 �gs * 80 = 10 ms)
#define SYSTEM_10MS_PER_SECOND 100  // �C��]�t�� 10 ms �p��

TaskType current_task = TASK_HEAT_CONTROL; // ��e����

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
  
  // ��l�ƼҶ�
  SystemCLK_Init();       // ��l�� �t���W�v
  GPIO_Init();            // ��l�� GPIO �t�m
  Timer0_Init();          // ��l�� Timer0�A�Ω󲣥� 125 �L���_
  Comparator_Init();      // ��l�ƩҦ������ (CM0, CM1, CM2)
  OP_Amp_Init();          // ��l�ƹB���j���A�Ω�q�y�q��
  PWM_Init();             // ��l��PWM for IGBT driving
  Buzzer_Init();          // ��l�ƭ����X��
  I2C_Init();           // ��l�� I2C ���f
  
  CNTdown_Timer_Init();   // ��l�ƭ˼ƭp�ɼҲ�
  
  EAL = 1;                // �ҥΥ��줤�_
  
  Warmup_Delay();         // 30ms
  ADC_Init();             // ��l�� ADC�A�Ω�h�q�D�q��
  
  Measure_AC_Low_Time();  // �q��AC low tume, �Ω�IGBT C�ů�q����ɶ� & �Ұʶ����[���ɶ��I
  Detect_AC_Frequency();
  
  // �]�m��l�t�Ϊ��A
  system_state = STANDBY;  // �t���q�{�i�J�ݾ��Ҧ�

  #if TUNE_MODE == 1
//  power_setting = 800000;
  #endif
  
  // system_ticks reset to zero
  system_ticks = 0;
  
  // �i�J�D�{���`��
  while (1) {
    I2C_Communication();
    
    // 125 �gs �w���޿�
    if (f_125us) {
      f_125us = 0;  // �M�� 125 �L��X��
            
      // ��s�t�ήɶ�
      Update_System_Time();  
      
      // �W�߰������
      Power_read();    
          
      // ���ȴ`��
      switch (current_task) {
        case TASK_HEAT_CONTROL:
            Heat_Control();
            current_task = TASK_POWER_CONTROL; // ������U�@�ӥ���
            break;
        
        case TASK_POWER_CONTROL:
            Power_Control();
            current_task = TASK_QUICK_CHANGE_DETECT; // ������U�@�ӥ���
            break;  
        
        case TASK_QUICK_CHANGE_DETECT:
            //Quick_Change_Detect();
            current_task = TASK_TEMP_MEASURE; // ������U�@�ӥ���
            break;

        case TASK_TEMP_MEASURE:
            Temp_Measure();
            current_task = TASK_TEMP_PROCESS; // ������U�@�ӥ���
            break;
          
        case TASK_TEMP_PROCESS:
            Temp_Process();
            current_task = TASK_CURRENT_POT_CHECK; // ������U�@�ӥ���
            break;

        case TASK_CURRENT_POT_CHECK:
            //Pot_Detection_In_Heating();  // ����q�y�������
            current_task = TASK_SHUTDOWN; // ������U�@�ӥ���
            break;

        case TASK_SHUTDOWN:
            //Shutdown_Task();  // ������������
            current_task = TASK_ERROR_PROCESS; // ������U�@�ӥ���
            break;

        case TASK_ERROR_PROCESS:
            //Error_Process();  // �B�z���~����
            current_task = TASK_HEAT_CONTROL;  // �`���^��Ĥ@�ӥ���
            break;

        default:  break;
      }
    } //(f_125us) end
    
    // **�����[���Ҧ��G�Y���ˬd AC �T���ܤ�**
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
