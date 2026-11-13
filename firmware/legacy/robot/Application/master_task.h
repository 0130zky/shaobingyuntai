/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2024-04-20 20:49:48
 * @LastEditTime : 2024-04-21 00:27:36
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\robot_template\Application\master_task.h
 * @Description  : 
 */

#ifndef __MASTER_TASK_H
#define __MASTER_TASK_H

#include "main.h"

#include "app_link.h"

typedef struct{
	uint8_t		MiniGimbal_Enable;		
	uint8_t 	Telescope_Enable;
	uint8_t 	Supercap_Enable;
  uint8_t   Vision_Enable;
} HID_HERO_t;

typedef struct
{
  UP_Board_Set_t  UP_Board_SetData;
  UP_Board_Get_t  UP_Board_GetData;
  HID_HERO_t	  Hid_Hero;
} master_link_t;

extern master_link_t MasterLink;

void MasterLink_Task(void);

#endif
