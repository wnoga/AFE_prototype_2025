/*
 * can_functions.c
 *
 *  Created on: Dec 19, 2024
 *      Author: blondier94
 */

#include "can_functions.h"
#include "AFE_functions.h"
#include "slcan/slcan.h"
#include <stm32f0xx_hal_can.h>
#include <stm32f0xx_hal_gpio.h>
#include <string.h>

extern CAN_HandleTypeDef hcan;
static CanTxMsgTypeDef CanTxBuffer;
static CanRxMsgTypeDef CanRxBuffer;
s_can_msg_recieved can_msg_received;
volatile int8_t canRxFlag = 0;

void
_delay (size_t ms)
{
  for (size_t i0 = 0; i0 < (48 / 4) * 1000 * ms; ++i0)
    {
      __asm__("nop");
    }
}

void
blink1 (void)
{
//  while(1)
//  for(uint8_t i0=0;i0<3;++i0)
//    {
//      HAL_Delay(250);
  HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_9);
//      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,GPIO_PIN_SET);
//    }
}

GPIO_TypeDef * GetGPIOPortByEnumerator(uint8_t enumerator)
{
  GPIO_TypeDef *port=NULL;
#if USE_SHIFTING_IN_SetPinFromArray
    // Calculate port address by shifting from GPIOA base
    port = (GPIO_TypeDef *)((uint32_t)GPIOA + (enumerator * ((uint32_t)GPIOB - (uint32_t)GPIOA)));
#else
  switch (enumerator)
    {
      case 0:port=GPIOA;break;
      case 1:port=GPIOB;break;
      case 2:port=GPIOC;break;
      case 3:port=GPIOD;break;
      case 4:port=GPIOE;break;
      case 5:port=GPIOF;break;
      default:break;
    }
#endif
  return port;
}



/**
 * @brief Return 1 if ID are equal
 * @param rxCanMsg Pointer to the CAN Rx
 * @param own_id The device's own ID (8-bit receiver ID)
 * @return 1 if ID are equal
 */
inline static int __attribute__ ((always_inline, optimize("-O3")))
is_this_msg_for_me (CanRxMsgTypeDef *rxCanMsg, uint8_t own_id)
{
  if (rxCanMsg->IDE != CAN_ID_STD)
    {
      return 0;
    }
  else if (own_id == (0xFF & (rxCanMsg->StdId >> 2)))
    {
      return 1;
    }
  else
    {
      return 0;
    }
}

/**
 * @brief Configure CAN filters to accept messages only from master and matching this device's ID
 * @param hcan Pointer to the CAN handle
 * @param own_id The device's own ID (8-bit receiver ID)
 * @return HAL_StatusTypeDef status (HAL_OK if successful)
 */
HAL_StatusTypeDef __attribute__ ((cold, optimize("-Os")))
configure_can_filter (CAN_HandleTypeDef *hcan, uint8_t own_id)
{
  CAN_FilterConfTypeDef sFilterConfig;
//  CAN_FilterTypeDef sFilterConfig;

  // Validate receiver ID
  if (own_id > 0xFF)
    {
      return HAL_ERROR;  // Invalid receiver ID
    }

  // Build Filter ID to match messages from Master to this device
  uint32_t filter_id = ((0 << 10) | (own_id << 2));  // Master bit and own receiver ID
  // Master = 0, Slave = 1

  // Build Filter Mask to filter by Master bit and Receiver ID
  uint32_t filter_mask = ((1 << 10) | (0xFF << 2));  // Mask for Master bit and receiver ID bits

  /* Master bit should be checked if is 0, otherwise message should be rejected */

  // Configure filter parameters
  sFilterConfig.BankNumber = 14;
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // Identifier mask mode
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32-bit filter
  sFilterConfig.FilterIdHigh = (filter_id << 5);    // Filter ID in high bits
//  sFilterConfig.FilterIdHigh = 0x0000;    // Filter ID in high bits
  sFilterConfig.FilterIdLow = 0x0000;                        // Not used for standard IDs
  sFilterConfig.FilterMaskIdHigh = (filter_mask << 5); // Mask in high bits≥
//  sFilterConfig.FilterMaskIdHigh = 0x0000; // Mask in high bits
  sFilterConfig.FilterMaskIdLow = 0x0000;                     // Not used for standard IDs
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;  // Assign to FIFO 0
  sFilterConfig.FilterActivation = ENABLE;                // Enable the filter

  // Apply the filter configuration
  return HAL_CAN_ConfigFilter (hcan, &sFilterConfig);
}

// CAN transmission state machine states
typedef enum
{
  e_CANMachineState_IDLE,
  e_CANMachineState_SENDING
} e_CANMachineState;

// Global variables
CANCircularBuffer_t canTxBuffer;
e_CANMachineState canState = e_CANMachineState_IDLE;
uint32_t canTxLast_ms = 0;

void
HAL_CAN_TxCpltCallback (CAN_HandleTypeDef *hcan)
{
  CAN_TransmitCallback (hcan);
}

/***
 * TODO Change behavior when CAN have an error
 */
void
HAL_CAN_ErrorCallback (CAN_HandleTypeDef *hcan)
{
  CAN_TransmitCallback (hcan);
  HAL_CAN_Receive_IT (hcan, CAN_FIFO0);
  NVIC_SystemReset();
}

// Initialize circular buffer
void __attribute__ ((cold, optimize("-Os")))
CANCircularBuffer_Init (CANCircularBuffer_t *cb)
{
  cb->head = 0;
  cb->tail = 0;
  cb->count = 0;
}

// Check if buffer is full
inline int8_t __attribute__ ((always_inline, optimize("-O3")))
CANCircularBuffer_isFull (CANCircularBuffer_t *cb)
{
  return cb->count == CAN_BUFFER_SIZE;
}

// Check if buffer is empty
inline int8_t __attribute__ ((always_inline, optimize("-O3")))
CANCircularBuffer_isEmpty (CANCircularBuffer_t *cb)
{
  return cb->count == 0;
}

// Add message to buffer
int8_t __attribute__ ((optimize("-O3")))
CANCircularBuffer_enqueueMessage (CANCircularBuffer_t *cb, CAN_Message_t *msg)
{
  if (CANCircularBuffer_isFull (cb))
    {
      return 0; // Buffer is full
    }
  else
    {
      memcpy (&cb->buffer[cb->head], msg, sizeof(*msg));
      cb->head = (cb->head + 1) % CAN_BUFFER_SIZE;
      ++cb->count;
      return 1;
    }
}

static CAN_Message_t*
CANCircularBuffer_getMessage (CANCircularBuffer_t *cb)
{
  return CANCircularBuffer_isEmpty (cb) ? NULL : &cb->buffer[cb->tail];
}

int8_t __attribute__ ((optimize("-O3")))
CANCircularBuffer_deleteMessage (CANCircularBuffer_t *cb)
{
  if (CANCircularBuffer_isEmpty (cb))
    {
      return 0; // Buffer is empty
    }
  else
    {
      cb->tail = (cb->tail + 1) % CAN_BUFFER_SIZE;
      --cb->count;
      return 1;
    }
}

// Remove message from buffer
CAN_Message_t*
CANCircularBuffer_dequeueMessage (CANCircularBuffer_t *cb)
{
  if (CANCircularBuffer_isEmpty (cb))
    {
      return 0; // Buffer is empty
    }
  CAN_Message_t *msg = &cb->buffer[cb->tail];
  cb->tail = (cb->tail + 1) % CAN_BUFFER_SIZE;
  --cb->count;
  return msg;
}

// Transmit handler (called periodically or in main loop)
void __attribute__ ((optimize("-O3")))
CAN_TransmitHandler (CAN_HandleTypeDef *hcan)
{
  CAN_Message_t *msg;
  while ((msg = CANCircularBuffer_getMessage (&canTxBuffer)))
    {
      if ((HAL_GetTick () - msg->timestamp) > CAN_MSG_LIFETIME_MS)
	{
	  CANCircularBuffer_deleteMessage (&canTxBuffer);
	  continue;
	}
      if (canState == e_CANMachineState_IDLE)
	{
#if CAN_MSG_BURST_DELAY_MS
	  if ((HAL_GetTick () - canTxLast_ms) > CAN_MSG_BURST_DELAY_MS)
	    {
#endif // CAN_MSG_BURST_DELAY_MS
	      canTxLast_ms = HAL_GetTick ();
	      hcan->pTxMsg->StdId = msg->id;
	      hcan->pTxMsg->DLC = msg->dlc;
	      hcan->pTxMsg->IDE = CAN_ID_STD;
	      hcan->pTxMsg->RTR = CAN_RTR_DATA;
	      memcpy (&hcan->pTxMsg->Data[0], &msg->data[0], msg->dlc);
	      if (HAL_CAN_Transmit_IT (hcan) == HAL_OK)
		{
		  canState = e_CANMachineState_SENDING; // Set state to sending
		  return;
		}
#if CAN_MSG_BURST_DELAY_MS
	    }
	  else
	    {
	      return;
	    }
#endif // CAN_MSG_BURST_DELAY_MS
	}
    }
  if (CANCircularBuffer_isEmpty (&canTxBuffer))
    {
      canState = e_CANMachineState_IDLE;
    }
}

// Callback for transmission complete (called by interrupt)
void __attribute__ ((optimize("-O3")))
CAN_TransmitCallback (CAN_HandleTypeDef *hcan)
{
  CANCircularBuffer_deleteMessage(&canTxBuffer);
  canState = e_CANMachineState_IDLE; // Set state back to idle
}

void
can_send_msg (CAN_HandleTypeDef *hcan, uint8_t *msg, size_t len, uint8_t own_id, uint8_t msg_code)
{
  if (len > 8)
    {
      return;
    }
  hcan->pTxMsg->DLC = len > 8 ? 8 : len;
  hcan->pTxMsg->StdId = (1 << 10) | (own_id << 2); // Master bit and own receiver ID
  hcan->pTxMsg->StdId |= msg_code & 0b11;
  memcpy (&hcan->pTxMsg->Data[0], msg, len);
  hcan->pTxMsg->RTR = CAN_RTR_DATA;
}

void modify_aurt_as_test_led(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  // GPIO Ports Clock Enable
  __HAL_RCC_GPIOA_CLK_ENABLE();
  // Configure PIN A9: UART TX to GPIO
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

typedef enum
{
  e_can_machine_state_init = 0,
  e_can_machine_state_idle
} e_can_machine_state;

static e_can_machine_state can_machine_state = e_can_machine_state_init;

void __attribute__ ((cold, optimize("-Os")))
can_machine_init_0 (void)
{
  can_machine_state = e_can_machine_state_init;
  canState = e_CANMachineState_IDLE;
  canTxLast_ms = HAL_GetTick ();

  hcan.pTxMsg = &CanTxBuffer;
  hcan.pRxMsg = &CanRxBuffer;
}

void
can_machine (void)
{
  switch (can_machine_state)
    {
    case e_can_machine_state_init:
      {
	can_machine_init_0();
	can_machine_state = e_can_machine_state_idle;
	canState = e_CANMachineState_IDLE;
	canTxLast_ms = HAL_GetTick ();

	/* CAN interrupt Init */

	HAL_NVIC_SetPriority (CEC_CAN_IRQn, 2, 2);
	HAL_NVIC_EnableIRQ (CEC_CAN_IRQn);
	HAL_CAN_Receive_IT (&hcan, CAN_FIFO0);

	StartCan ();
	CAN_Message_t tmp;
	  uint8_t command = AFECommand_resetAll;
	  uint32_t msg_id = CAN_ID_IN_MSG;
	  tmp.timestamp = HAL_GetTick ();
	  tmp.id = msg_id;
	  tmp.data[0] = command; // Standard reply [function]
	  tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	  tmp.dlc = 2;
	CANCircularBuffer_enqueueMessage(&canTxBuffer, &tmp);

	break;
      }
    case e_can_machine_state_idle:
      {
	CAN_TransmitHandler (&hcan);
	break;
      }
    default:
      {
	NVIC_SystemReset ();
	break;
      }
    }
}

inline uint8_t __attribute__ ((always_inline, optimize("-O3")))
get_byte_of_message_number (uint8_t msg_index, uint8_t total_msg_count)
{
  return
      ((msg_index > 15) | (total_msg_count > 15)) ?
	  0 : (0xF0 & (total_msg_count << 4)) | ((msg_index + 1) & 0x0F); // 0xF0 - total nr of msgs, 0x0F msg nr
}

void
CANCircularBuffer_enqueueMessage_data (CANCircularBuffer_t *cb, CAN_Message_t *tmp,
				       uint8_t msg_index, uint8_t total_msg_count, uint8_t channel,
				       void *value, uint8_t size)
{
  tmp->data[1] = get_byte_of_message_number (msg_index, total_msg_count);
  tmp->data[2] = 1 << channel; // create mask of the channel
  tmp->dlc = 2 + 1 + size;
  memcpy (&tmp->data[3], value, size);
  CANCircularBuffer_enqueueMessage (cb, tmp);
}

void
CANCircularBuffer_enqueueMessage_data_float (CANCircularBuffer_t *cb, CAN_Message_t *tmp,
					     uint8_t msg_index, uint8_t total_msg_count,
					     uint8_t channel, float *value)
{
  CANCircularBuffer_enqueueMessage_data (cb, tmp, msg_index, total_msg_count, channel, (void*)value,
					 sizeof(float));
}

void
CANCircularBuffer_enqueueMessage_timestamp_ms (CANCircularBuffer_t *cb, CAN_Message_t *tmp,
					       uint8_t msg_index, uint8_t total_msg_count,
					       uint32_t timestamp_ms)
{
  tmp->data[1] = get_byte_of_message_number (msg_index, total_msg_count);
  tmp->data[2] = 0;
  tmp->dlc = 2 + 1 + sizeof(uint32_t);
  memcpy (&tmp->data[3], &timestamp_ms, sizeof(uint32_t));
  CANCircularBuffer_enqueueMessage (cb, tmp);
}

void __attribute__ ((optimize("-O3")))
HAL_CAN_RxCpltCallback (CAN_HandleTypeDef *hcan)
{
  if (is_this_msg_for_me(hcan->pRxMsg, CAN_ID) && (hcan->pRxMsg->DLC <= 8))
    {
      can_msg_received.DLC = hcan->pRxMsg->DLC;
      can_msg_received.timestamp = HAL_GetTick ();
      memcpy (&can_msg_received.Data[0], &hcan->pRxMsg->Data[0], hcan->pRxMsg->DLC);
      canRxFlag = 1;
    }
  HAL_CAN_Receive_IT (hcan, CAN_FIFO0);
}
