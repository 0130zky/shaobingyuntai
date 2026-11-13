/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2024-04-19 20:31:45
 * @LastEditTime : 2024-04-21 01:44:42
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\robot_template\Application\master_task.c
 * @Description  : 
 */

#include "master_task.h"

master_link_t MasterLink;

/**
 * @description  : 
 * @return        {*}
 */
void MasterLink_Task(void)
{ 
  Set_MasterLink_Data(&MasterLinkBus);
}


