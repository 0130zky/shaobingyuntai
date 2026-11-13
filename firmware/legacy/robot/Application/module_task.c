/*
 * @File         :
 * @Author       : JL HUANG
 * @Date         : 2023-11-26 16:10:36
 * @LastEditTime : 2023-11-26 16:43:37
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\balance\Application\module_task.c
 * @Description  :
 */

#include "cmsis_os.h"

#include "module_task.h"
#include "tim.h"
#include "booster_task.h"
#include "gimbal_task.h"
#include "ins_task.h"
#include "app_DM4310.h"
#include "master_task.h"
#include "referee.h"
#include "FreeRTOS.h"
#include "task.h"

/* 独立校准帧 A6 5A 01 int16_le(0.01deg) xor(前5字节)，不含发弹指令。 */
void FrictionScope_RxByte(uint8_t byte)
{
  static uint8_t frame[6], used;
  static uint32_t last;
  uint32_t now = HAL_GetTick();
  if (now-last > 50) used=0;
  last=now;
  if (used==0 && byte!=0xa6) return;
  if (used==1 && byte!=0x5a) { used=(byte==0xa6); return; }
  frame[used++]=byte;
  if (used==6) {
    used=0;
    if (frame[2]==1 && (uint8_t)(frame[0]^frame[1]^frame[2]^frame[3]^frame[4])==frame[5])
      Booster_RequestDialPhase((int16_t)(frame[3] | ((uint16_t)frame[4]<<8)));
  }
}

/* USART1 主帧仍为25 float；每20ms插入24 float诊断帧，帧尾为负无穷。
   两个时间通道均以 65536ms 回绕，保证 float 可以精确表示毫秒整数。 */
static void FrictionScope_Send(void)
{
  enum { CHANNELS = 25, PAYLOAD_BYTES = CHANNELS * sizeof(float) };
  static union {
    float channel[CHANNELS + 1];
    uint8_t bytes[PAYLOAD_BYTES + 4];
  } packet;
  referee_debug_t judge;
  if (huart1.gState != HAL_UART_STATE_READY) return;
  static uint8_t reply[6];
  int16_t phase;
  uint8_t type;
  if (Booster_TakeDialPhaseReply(&phase, &type)) {
    reply[0]=0xa6; reply[1]=0x5a; reply[2]=type;
    reply[3]=(uint8_t)phase; reply[4]=(uint8_t)((uint16_t)phase>>8);
    reply[5]=reply[0]^reply[1]^reply[2]^reply[3]^reply[4];
    HAL_UART_Transmit_DMA(&huart1, reply, sizeof(reply));
    return;
  }
  /* 单一发送者、静态DMA缓冲区，诊断不挤大每一帧主遥测。 */
  static uint32_t diag_sent_ms;
  static union { float value[FRI_DIAG_CHANNELS+1]; uint8_t bytes[(FRI_DIAG_CHANNELS+1)*4]; } diag;
  if ((uint32_t)(HAL_GetTick()-diag_sent_ms)>=20U) {
    diag_sent_ms=HAL_GetTick();
    Booster_TakeFrictionDiagnostics(diag.value);
    const uint32_t n=FRI_DIAG_CHANNELS*4U;
    diag.bytes[n]=0; diag.bytes[n+1]=0; diag.bytes[n+2]=0x80; diag.bytes[n+3]=0xff;
    HAL_UART_Transmit_DMA(&huart1, diag.bytes, sizeof(diag.bytes));
    return;
  }
  Referee_GetDebug(&judge);
  const uint32_t now = HAL_GetTick();
  uint32_t age = now - judge.last_frame_ms;
  if (!judge.valid_frame_count || age > 65535U) age = 65535U;
  const shooter_referee_status_t referee = {
    .bullet_speed_mps = judge.shot.initial_speed,
    .shot_count = judge.shot_count,
    .shot_received_ms = judge.shot_rx_ms,
    .frame_age_ms = age,
    .shooter_id = judge.shot.shooter_number,
  };
  /* Encoder copies one completed control cycle; DMA keeps its own buffer. */
  if (!Booster_BuildScopeTelemetry(packet.channel, &referee)) return;
  packet.bytes[PAYLOAD_BYTES + 0] = 0x00;
  packet.bytes[PAYLOAD_BYTES + 1] = 0x00;
  packet.bytes[PAYLOAD_BYTES + 2] = 0x80;
  packet.bytes[PAYLOAD_BYTES + 3] = 0x7f;
  HAL_UART_Transmit_DMA(&huart1, packet.bytes, sizeof(packet.bytes));
}

Derivative derivative;
VisionConnect vision1;
union VofaDATA Vofa;
extern float tim;
void Module_Init(void)
{
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);


}
Derivative DM_delta;
extern volatile uint32_t pack_time;
extern uint32_t *mail_p;
extern uint32_t test_t;
/**
 * @description  : 常规辅助任务接口
 * @return        {*}
 */
void Module_Task(void)
{
    static uint8_t const_i;
    if(const_i++ >= 200)
    {
        const_i = 0;
        if(HAL_GetTick() - pack_time >= 500)//遥控状态看门狗
        {
            RC_Data.Error_code |= 0x01;
        }
        else
        {
            RC_Data.Error_code &= 0xfe;
        }

        if(RC_Data.Error_code || Booster_DialFault())
        {
            LED_Red();      // 遥控丢失：只亮红灯提示，不鸣笛
        }
        else{
            LED_Green();
        }
    }


   static uint8_t vision_count = 0;
   static uint8_t telem_count  = 0;
   uint8_t usb_tx_this_cycle = 0;
   if(MasterLink.UP_Board_GetData.Color == 1)
   VisionConnectUpdateTX(&vision1, 'B', INS.Yaw + 180.0f, GIMBAL.DM4310.RefAngle, MasterLink.UP_Board_GetData.gunSpeed1_f); //更新视觉发送缓存
   if(MasterLink.UP_Board_GetData.Color == 0)
   VisionConnectUpdateTX(&vision1, 'R', INS.Yaw + 180.0f, GIMBAL.DM4310.RefAngle, MasterLink.UP_Board_GetData.gunSpeed1_f); //更新视觉发送缓存
   if (vision_count++ >= 10) {
       VisionConnectSend(&vision1);  // 向视觉发送数据10ms@100hz
       vision_count = 0;
       usb_tx_this_cycle = 1;
   }
   //摩擦轮状态回传上位机。注意：CDC_Transmit_FS 不可重入，
   //USB 发送端只能有这一个调用者，且每次迭代最多发一帧
   if (!usb_tx_this_cycle && ++telem_count >= SCOPE_DIV) {
       telem_count = 0;
       uint8_t telem[PC_TELEM_LEN];
       if (Booster_BuildTelemetry(telem))
         CDC_Transmit_FS(telem, PC_TELEM_LEN);  // 返回 BUSY 则丢这一帧，上位机容忍丢帧
   }

   FrictionScope_Send();
}
/**
 * @brief 离散求导
 * @param Derivative 导数结构体指针
 * @param x 当前自变量值，单位：us
 * @param y 当前因变量值
 * @return 导数估计值
 */
float derivation(float x, float y, Derivative *delta)
{
    if(!delta -> flag)
    {
        delta -> flag = 1;
        delta -> x_last = x;
        delta -> y_last = y;
        return 0.0f;
    }

    //向前差分计算导数
    float dx = (x - delta -> x_last) / 168000000;
    float dy =  y - delta -> y_last;

    delta -> x_last = x;
    delta -> y_last = y;

    //if(fabs(dx) < 1e-11) return 0.0f;

    return dy/dx;
}

// 初始化窗口滤波
void window_filter_init(winter_variance_t* variable, char width) {
    // 设置窗口宽度
    variable->winter_width = width;
    // 初始化窗口中的数据为0
    for (char i = 0; i < width; i++) {
      variable->x[i] = 0;
    }
    // 初始化窗口数据的和为0
    variable->x_sum = 0;
    // 初始化窗口数据的平均值
    variable->x_avr = 0;
  }

  // 计算窗口滤波
  int32_t window_filter(winter_variance_t* variable, int32_t input) {
    // 将输入数据存入窗口
    variable->x[variable->ptr] = input;
    // 初始化窗口数据的和为0
    variable->x_sum = 0;
    // 计算窗口数据的和
    for (char i = 0; i < variable->winter_width; i++) {
      variable->x_sum += variable->x[i];
    }
    // 计算窗口数据的平均值
    variable->x_avr = variable->x_sum / variable->winter_width;
    // 更新指针
    variable->ptr++;
    // 如果指针到达窗口末尾，则重置为0
    if (variable->ptr == variable->winter_width)
      variable->ptr = 0;
    return variable->x_avr;
  }

/**
 * @brief Vofa+函数justfloat协议发送函数，使用DMA发送(cubemx内开启对应串口的DMA
 *
 * @param set 下列四个fp32(float)格式变量名称和个数可以自行更改，对应更改ch数组大小即可
 * @param feedback
 * @param target
 * @param speed
 * @param huart 所使用的串口号地址
 */

 void JustFloat(float set, float feedback, float target, float speed, UART_HandleTypeDef *huart) {
    /* DMA 尚未完成时丢弃本次采样，不能改写它正在读取的全局缓冲区。
       此发送口仅由 Module_Task 调用。 */
    if (huart->gState != HAL_UART_STATE_READY) return;
    // uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};  协议规定的帧尾
    Vofa.VofaD.set = set;
    Vofa.VofaD.feedback = feedback;
    Vofa.VofaD.TargetSpeed1 = target;
    Vofa.VofaD.Speed1 = speed;
    Vofa.VofaD.tail1 = 0x00;
    Vofa.VofaD.tail2 = 0x00;
    Vofa.VofaD.tail3 = 0x80;
    Vofa.VofaD.tail4 = 0x7f;
    HAL_UART_Transmit_DMA(huart, Vofa.ch, 20);
  }

  VisionState visionState;
  union nav_rxd_u nav_rxd;
  /**
   * @brief
   * @param  connect:
   * @param  TargetColor:
   */
  void VisionConnectINIT(VisionConnect *connect, uint8_t OurColor) {
    connect->RXData.VisionRxData.PitchAngleTarget = 0.0f;
    connect->RXData.VisionRxData.YawAngleTarget = 0.0f;
    connect->RXData.VisionRxData.fireControl = HOLD_FIRE;
    connect->RXData.VisionRxData.header = 0;
    // connect->TXData.targetType = 0;
    connect->TXData.PitchAngle = 0.0f;
    connect->TXData.YawAngle = 0.0f;
    connect->TXData.OurColor = OurColor;
    connect->TXData.ShootSpeed = 0.0f;
  }

  /**
   * @brief  更新视觉发送缓存
   * @param  connect:
   * @param  TargetColor:
   * @param  yawAngle:
   * @param  pitchAngle:
   * @param  shootSpeed:
   */
  void VisionConnectUpdateTX(VisionConnect *connect, uint8_t OurColor, fp32 yawAngle,
                             fp32 pitchAngle, fp32 shootSpeed) {
    // connect->TXData.targetType = 0;
    connect->TXData.OurColor = OurColor;

    connect->TXData.PitchAngle = pitchAngle;	// 反转pitch轴
    connect->TXData.YawAngle = yawAngle;	// 将yawAngle的范围由-180~180改为0~360
    connect->TXData.ShootSpeed = shootSpeed;
  }

  /**
   * @brief  向视觉上位机发送数据
   * @param  connnect:
   */
  void VisionConnectSend(VisionConnect *connnect) {

    connnect->TXData.packHead = 0x00a5;  // 包头
    // connnect->TXData.targetType = 0x00;
    // 我方颜色
    if (connnect->TXData.OurColor == RED) {
      connnect->TXData.OurColor = RED;
    }
    else if (connnect->TXData.OurColor == BLUE) {
      connnect->TXData.OurColor = BLUE;
    }

    // 发送
    CDC_Transmit_FS((uint8_t *)&(connnect->TXData), sizeof(connnect->TXData));
  }

  // 解包state
  void VisionGetState(VisionConnect* connect, VisionState* visionState)
  {
      uint8_t state = connect->RXData.VisionRxData.state;
      visionState->aoto_aim_state = state & 0x01;
      visionState->our_color = (state & 0x02) >> 1;
      visionState->tracking = (state & 0x04) >> 2;
      visionState->aim_type = (state & 0x18) >> 3;
  }
