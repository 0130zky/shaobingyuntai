/**
 *  ███╗   ██╗███████╗██╗  ██╗████████╗  ███████╗
 *  ████╗  ██║██╔════╝╚██╗██╔╝╚══██╔══╝  ██╔════╝
 *  ██╔██╗ ██║█████╗   ╚███╔╝    ██║     █████╗  
 *  ██║╚██╗██║██╔══╝   ██╔██╗    ██║     ██╔══╝  
 *  ██║ ╚████║███████╗██╔╝ ██╗   ██║     ███████╗
 *  ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝   ╚═╝     ╚══════╝
 *
 *  @Author : Kaiyue Zhang
 *  @Email  : kaiyue.link@gmail.com
 *  @date   : 2026-10-30
 *  @description ：CAN 底层驱动：过滤器配置、接收中断与帧类型/长度检查。
 */
#include "bsp_can.h"
#include "can.h"
/* Bindings are installed by application startup, before enabling reception. */
static bsp_can_rx_handler_t s_can1_receive, s_can2_receive;

void CAN_SetRxHandlers(bsp_can_rx_handler_t can1, bsp_can_rx_handler_t can2)
{
  s_can1_receive = can1;
  s_can2_receive = can2;
}

static void Receive(CAN_HandleTypeDef *hcan, uint32_t fifo, bsp_can_rx_handler_t handler)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
  if (HAL_CAN_GetRxMessage(hcan, fifo, &header, data) != HAL_OK) return;
  if (!handler || header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA ||
      header.DLC > sizeof(data)) return;
  handler((uint16_t)header.StdId, data, (uint8_t)header.DLC);
}

/**
 * @description  : CAN过滤器配置
 * @return       : 无
 */
void CAN_Filter_Init(void)
{
  CAN_FilterTypeDef CAN_FilterInitStructure;

  CAN_FilterInitStructure.FilterActivation     = CAN_FILTER_ENABLE;     //过滤器使能
  CAN_FilterInitStructure.FilterBank           = CAN1_FILTER_BANK;      //CAN1起始过滤器组编号
  CAN_FilterInitStructure.SlaveStartFilterBank = CAN2_FILTER_BANK;
  CAN_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO0;          //过滤器组关联FIFO0
  CAN_FilterInitStructure.FilterIdHigh         = 0x0000;
  CAN_FilterInitStructure.FilterIdLow          = 0x0000;
  CAN_FilterInitStructure.FilterMaskIdHigh     = 0x0000;
  CAN_FilterInitStructure.FilterMaskIdLow      = 0x0000;
  CAN_FilterInitStructure.FilterMode           = CAN_FILTERMODE_IDMASK; //掩码模式
  CAN_FilterInitStructure.FilterScale          = CAN_FILTERSCALE_32BIT; //32位

  HAL_CAN_ConfigFilter(&hcan1,&CAN_FilterInitStructure);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);     //开启FIFO0的接收中断


}

/**
 * @description  : 接收FIFO0中断回调函数，处理CAN1接收到的报文
 * @param         {CAN_HandleTypeDef} *hcan
 * @return        {*}
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1) Receive(hcan, CAN_RX_FIFO0, s_can1_receive);
}

/**
 * @description  : 接收FIFO1中断回调函数，处理CAN2接收到的报文
 * @param         {CAN_HandleTypeDef} *hcan
 * @return        {*}
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN2) Receive(hcan, CAN_RX_FIFO1, s_can2_receive);
}

/**
 * @description  : CAN错误中断回调函数，在这里重新打开接收中断
 * @param         {CAN_HandleTypeDef} *hcan
 * @return        {*}
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  }
  if (hcan->Instance == CAN2)
  {
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);
  }
}
