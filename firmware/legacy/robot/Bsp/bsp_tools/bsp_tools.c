/**
 * @brief :tools函数
 * @author cherishQA
 * @date : 2025-10-4
 * @version : 0.1
*/
#include "bsp_tools.h"

/**
 * @brief ：绝对值函数
 * @param input :输入值
*/
float absf( float input )
{
  if(input >= 0.0f)
   {
    return input ;
   }
   else return (-input);
}



