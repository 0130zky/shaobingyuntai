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
 *  @description ：裁判系统帧解析、校验与数据发布。
 */
#include "referee.h"
#include "FreeRTOS.h"
#include "task.h"

static referee_debug_t s_debug;

void Referee_GetDebug(referee_debug_t *out) {
  taskENTER_CRITICAL();
  *out = s_debug;
  taskEXIT_CRITICAL();
}

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

uint8_t usart_buf[2][USART_RX_BUF_LENGHT];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];  

unpack_data_t referee_unpack_obj;
frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

game_state_t game_state;                              // 0x0001
game_result_t game_result;                            // 0x0002
game_robot_HP_t game_robot_HP;                        // 0x0003
event_data_t event_data;                              // 0x0101         
referee_warning_t referee_warning;                    // 0x0104
dart_remaining_time_t dart_remaining_time;            // 0x0105
robot_state_t robot_state;                            // 0x0201
power_heat_data_t power_heat_data;                    // 0x0202
game_robot_pos_t game_robot_pos;                      // 0x0203
robot_buff_t robot_buff;                              // 0x0204
hurt_data_t hurt_data;                                // 0x0206
shoot_data_t shoot_data;                              // 0x0207
projectile_allowance_t projectile_allowance;          // 0x0208
rfid_status_t rfid_status;                            // 0x0209
dart_client_cmd_t dart_client_cmd;                    // 0x020A
ground_robot_position_t ground_robot_position;        // 0x020B
radar_mark_data_t radar_mark_data;                    // 0x020C
robot_interaction_data_t robot_interaction_data;      // 0x0301
referee_remote_control_t referee_remote_control;      // 0x0304
robot_custom_data_t robot_custom_data;			          // 0x0309

/**
 * @brief :初始化uart串口基于DMA，FIFO的双缓冲区域的接收
 * @param huart :串口是谁
 * @param hdmarx : 接受
 * @param hdmatx : 发送
 * @param rx1_buf rx2_buf : 双缓冲区
 * @param dma_buf_num : 单个缓冲区的大小
*/
void referee_usart_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdmarx,
                        DMA_HandleTypeDef *hdmatx, uint8_t *rx1_buf, uint8_t *rx2_buf,
                        uint16_t dma_buf_num) {
  // enable the DMA transfer for the receiver and tramsmit request
  // 启用发送和接收请求
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAT);
  // enalbe idle interrupt
  // 开启空闲中断
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);      
  // disable DMA                                   
  // 停止DMA ，清0 flag                                       
  __HAL_DMA_DISABLE(hdmarx);                           
  // 等待flag被清除                                                       
  while (hdmarx->Instance->CR & DMA_SxCR_EN) {           
    __HAL_DMA_DISABLE(hdmarx);                           
  }                                                      
  //清除所有的中断标志                                                       
  __HAL_DMA_CLEAR_FLAG(hdmarx, DMA_LISR_TCIF1);               
                                                               
  hdmarx->Instance->PAR = (uint32_t) & (USART6->DR);

  // memory buffer 1
  hdmarx->Instance->M0AR = (uint32_t)(rx1_buf);
  // memory buffer 2
  hdmarx->Instance->M1AR = (uint32_t)(rx2_buf);
  // data length
  __HAL_DMA_SET_COUNTER(hdmarx, dma_buf_num);

  // enable double memory buffer
  SET_BIT(hdmarx->Instance->CR, DMA_SxCR_DBM);

  // enable DMA
  __HAL_DMA_ENABLE(hdmarx);

  // disable DMA
  __HAL_DMA_DISABLE(hdmatx);

  while (hdmatx->Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(hdmatx);
  }

  hdmatx->Instance->PAR = (uint32_t) & (USART6->DR);
}

void init_referee_struct_data(void) {
  memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
  memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

  memset(&game_state, 0, sizeof(game_state_t));
  memset(&game_result, 0, sizeof(game_result_t));
  memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));

  memset(&event_data, 0, sizeof(event_data_t));
  memset(&referee_warning, 0, sizeof(referee_warning_t));

  memset(&dart_remaining_time, 0, sizeof(dart_remaining_time_t));
  memset(&robot_state, 0, sizeof(robot_state_t));
  memset(&power_heat_data, 0, sizeof(power_heat_data_t));
  memset(&game_robot_pos, 0, sizeof(game_robot_pos_t));
  memset(&robot_buff, 0, sizeof(robot_buff_t));
  memset(&hurt_data, 0, sizeof(hurt_data_t));
  memset(&shoot_data, 0, sizeof(shoot_data_t));
  memset(&projectile_allowance, 0, sizeof(projectile_allowance_t));
  memset(&rfid_status, 0, sizeof(rfid_status_t));
  memset(&dart_client_cmd, 0, sizeof(dart_client_cmd_t));
  memset(&ground_robot_position, 0, sizeof(ground_robot_position_t));
  memset(&radar_mark_data, 0, sizeof(radar_mark_data_t));
  memset(&robot_interaction_data, 0, sizeof(robot_interaction_data_t));
  memset(&referee_remote_control, 0, sizeof(referee_remote_control_t));
}

void referee_data_solve(uint8_t *frame) {
  uint16_t cmd_id = 0;

  uint8_t index = 0;

  memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

  index += sizeof(frame_header_struct_t);

  memcpy(&cmd_id, frame + index, sizeof(uint16_t));
  index += sizeof(uint16_t);

  taskENTER_CRITICAL();
  s_debug.valid_frame_count++;
  s_debug.last_frame_ms = HAL_GetTick();
  taskEXIT_CRITICAL();

  switch (cmd_id) {
    case GAME_STATE_CMD_ID: {
      memcpy(&game_state, frame + index, sizeof(game_state_t));
    } break;
    case GAME_RESULT_CMD_ID: {
      memcpy(&game_result, frame + index, sizeof(game_result_t));
    } break;
    case GAME_ROBOT_HP_CMD_ID: {
      memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP_t));
    } break;
    case EVENTS_CMD_ID: {
      memcpy(&event_data, frame + index, sizeof(event_data_t));
    } break;
    case REFEREE_WARNING_CMD_ID: {
      memcpy(&referee_warning, frame + index, sizeof(referee_warning_t));
    } break;
    case DART_REMAINING_TIME_CMD_ID: {
      memcpy(&dart_remaining_time, frame + index, sizeof(dart_remaining_time_t));
    } break;
    case ROBOT_STATE_CMD_ID: {
      memcpy(&robot_state, frame + index, sizeof(robot_state_t));
    } break;
    case POWER_HEAT_DATA_CMD_ID: {
      memcpy(&power_heat_data, frame + index, sizeof(power_heat_data_t));
    } break;
    case GAME_ROBOT_POS_CMD_ID: {
      memcpy(&game_robot_pos, frame + index, sizeof(game_robot_pos_t));
    } break;
    case ROBOT_BUFF_CMD_ID: {
      memcpy(&robot_buff, frame + index, sizeof(robot_buff_t));
    } break;
    case HURT_DATA_CMD_ID: {
      memcpy(&hurt_data, frame + index, sizeof(hurt_data_t));
    } break;
    case SHOOT_DATA_CMD_ID: {
      /* 旧协议或异常长度不能被当作新的测速事件。 */
      if (referee_receive_header.data_length != sizeof(shoot_data_t)) break;
      memcpy(&shoot_data, frame + index, sizeof(shoot_data_t));
      taskENTER_CRITICAL();
      s_debug.shot = shoot_data;
      s_debug.shot_rx_ms = HAL_GetTick();
      s_debug.shot_count++;
      taskEXIT_CRITICAL();
    } break;
    case PROJECTILE_ALLOWANCE_CMD_ID: {
      memcpy(&projectile_allowance, frame + index, sizeof(projectile_allowance_t));
    } break;
    case RFID_STATUS_CMD_ID: {
      memcpy(&rfid_status, frame + index, sizeof(rfid_status_t));
    } break;
    case DART_CLIENT_CMD_CMD_ID: {
      memcpy(&dart_client_cmd, frame + index, sizeof(dart_client_cmd_t));
    } break;
    case GROUND_ROBOT_POSITION_CMD_ID: {
      memcpy(&ground_robot_position, frame + index, sizeof(ground_robot_position_t));
    } break;
    case RADAR_MARK_DATA_CMD_ID: {
      memcpy(&radar_mark_data, frame + index, sizeof(radar_mark_data_t));
    } break;
    case ROBOT_INTERACTION_DATA_CMD_ID: {
      memcpy(&robot_interaction_data, frame + index, sizeof(robot_interaction_data_t));
    } break;
    case REFEREE_REMOTE_CONTROL_CMD_ID: {
      memcpy(&referee_remote_control, frame + index, sizeof(referee_remote_control_t));
    } break;
    default: {
      break;
    }
  }
}


void refereeINIT() {
  memset(&s_debug, 0, sizeof(s_debug));
  init_referee_struct_data();  //分配空间

  fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);

  referee_usart_init(&huart6, &hdma_usart6_rx, &hdma_usart6_tx, usart_buf[0], usart_buf[1],
                     USART_RX_BUF_LENGHT);
}


// ÿ10ms����һ�Σ��������
void referee_unpack_fifo_data(void) {
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while (fifo_s_used(&referee_fifo)) {
    byte = fifo_s_get(&referee_fifo);
    switch (p_obj->unpack_step) {
        /*********�������ǶԴ���������һ֡���ݵ�֡ͷframe_header����У��**********/
      case STEP_HEADER_SOF: {
        if (byte == sof) {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else {
          p_obj->index = 0;
        }
      } break;

      case STEP_LENGTH_LOW: {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      } break;

      case STEP_LENGTH_HIGH: {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN)) {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      } break;

      case STEP_FRAME_SEQ: {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      } break;

      case STEP_HEADER_CRC8: {
        p_obj->protocol_packet[p_obj->index++] = byte;
        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE) {
          if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE)) {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      } break;
        /*********�����ǶԴ���������һ֡���ݵ�֡ͷframe_header����У��**********/
      case STEP_DATA_CRC16: {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if (verify_CRC16_check_sum(p_obj->protocol_packet,
                                     REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
            referee_data_solve(p_obj->protocol_packet);  // �����������Ķ����ݽ��н��
          }
        }
      } break;

      default: {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      } break;
    }
  }
}

/**
 * @brief  �жϴ������������ж��е���
 */
void refereeReceiveHandler(void) {
  static volatile uint8_t res;
  if (USART6->SR & UART_FLAG_IDLE) {
    __HAL_UART_CLEAR_PEFLAG(&huart6);

    static uint16_t this_time_rx_len = 0;

    if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT)
        == RESET) { /* Current memory buffer used is Memory 0 */

      // disable DMA
      // �����жϽ���������һ�����������ݴ��꣬Ȼ��ʧЧDMA��cpuȥ������Բ��ԣ�
      __HAL_DMA_DISABLE(huart6.hdmarx);

      // get receive data length, length = set_data_length - remain_length
      // ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
      this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);

      // cpu���·���dma����
      __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);

      // cpu��û�д�����ո��������������������dma��׼���ְ����Ļ���ŵ�Memory1
      huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;

      __HAL_DMA_ENABLE(huart6.hdmarx);

      // cpu���ڴ���dma��������ݣ���������Memory 0��
      fifo_s_puts(&referee_fifo, (char *)usart_buf[0], this_time_rx_len);
      //  detect_hook(REFEREE_TOE);
    }
    else {
      /* Current memory buffer used is Memory 1 */
      __HAL_DMA_DISABLE(huart6.hdmarx);
      this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
      __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
      huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
      __HAL_DMA_ENABLE(huart6.hdmarx);
      fifo_s_puts(&referee_fifo, (char *)usart_buf[1], this_time_rx_len);
      //  detect_hook(REFEREE_TOE);
    }
  }
}
