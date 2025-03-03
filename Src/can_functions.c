/*
 * can_functions.c
 *
 *  Created on: Dec 19, 2024
 *      Author: blondier94
 */

#include "can_functions.h"
#include "slcan/slcan.h"
#include <stm32f0xx_hal_can.h>
#include <stm32f0xx_hal_gpio.h>
#include <string.h>

extern CAN_HandleTypeDef hcan;
s_can_msg_recieved can_msg_received;

void
_delay (size_t ms)
{
  for (size_t i0 = 0; i0 < (48 / 4) * 1000 * ms; ++i0)
    {
      __asm__("nop");
    }
}

void blink1(void)
{
//  while(1)
//  for(uint8_t i0=0;i0<3;++i0)
//    {
//      HAL_Delay(250);
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
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

/***
 * array[0] -> port
 * array[1] -> pin
 * array[2] -> status
 */
void SetPinFromArray(uint8_t *array) {
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState state;

    port = GetGPIOPortByEnumerator(array[0]);

    pin = (1 << array[1]); // Convert pin number to HAL GPIO pin format
    state = (array[2] != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    // Set the pin state
    HAL_GPIO_WritePin(port, pin, state);
}

GPIO_PinState ReadPinFromArray(uint8_t *array) {
    GPIO_TypeDef *port;
    uint16_t pin;

    // Calculate port address by shifting from GPIOA base
    port = (GPIO_TypeDef *)((uint32_t)GPIOA + (array[0] * ((uint32_t)GPIOB - (uint32_t)GPIOA)));

    pin = (1 << array[1]); // Convert pin number to HAL GPIO pin format

    // Read and return the pin state
    return HAL_GPIO_ReadPin(port, pin);
}


/**
 * @brief Return 1 if ID are equal
 * @param rxCanMsg Pointer to the CAN Rx
 * @param own_id The device's own ID (8-bit receiver ID)
 * @return 1 if ID are equal
 */
static int
is_this_msg_for_me (CanRxMsgTypeDef *rxCanMsg, uint8_t own_id)
{
  if (rxCanMsg->IDE != CAN_ID_STD)
    {
      return 0;
    }
//  if (rxCanMsg->StdId == CAN_ID) /* FIXME Delete this IF on production */
//    {
//      return 1;
//    }
  if (own_id == (0xFF & (rxCanMsg->StdId >> 2)))
    {
      return 1;
    }
  return 0;
}

void can_machine_new_message_received(CanRxMsgTypeDef *new_msg)
{
  is_this_msg_for_me(new_msg, CAN_ID);
  can_msg_received.timestamp = HAL_GetTick();
  can_msg_received.DLC = new_msg->DLC;
//  can_msg_recieved.StdId = new_msg->StdId;
  memcpy(&can_msg_received.Data[0],&new_msg->Data[0],new_msg->DLC);
}

/**
 * @brief Configure CAN filters to accept messages only from master and matching this device's ID
 * @param hcan Pointer to the CAN handle
 * @param own_id The device's own ID (8-bit receiver ID)
 * @return HAL_StatusTypeDef status (HAL_OK if successful)
 */
HAL_StatusTypeDef
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
//  filter_mask &=~(0x0003); // clear last two bits from mask (used for communication purposes)

//  // Master bit should be checked if is 0, otherwise message should be rejected
//  filter_id = 0;
//  filter_mask = 0;

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
typedef enum {
    CAN_IDLE,
    CAN_SENDING
} CAN_State_t;

// Global variables
CircularBuffer_t canTxBuffer;
CAN_State_t canState = CAN_IDLE;
uint32_t canTxLast_ms = 0;
#define CAN_MSG_BURST_DELAY_MS 100

// Function prototypes
void CAN_InitBuffer(CircularBuffer_t *cb);
int8_t CAN_IsBufferFull(CircularBuffer_t *cb);
int8_t CAN_IsBufferEmpty(CircularBuffer_t *cb);
int8_t CAN_EnqueueMessage(CircularBuffer_t *cb, CAN_Message_t *msg);
int8_t CAN_DequeueMessage(CircularBuffer_t *cb, CAN_Message_t *msg);
void CAN_TransmitHandler(CAN_HandleTypeDef *hcan);
void CAN_TransmitCallback(CAN_HandleTypeDef *hcan);

void
HAL_CAN_TxCpltCallback (CAN_HandleTypeDef *hcan)
{
  CAN_TransmitCallback (hcan);
}
void
HAL_CAN_ErrorCallback (CAN_HandleTypeDef *hcan)
{
  CAN_TransmitCallback (hcan);
  HAL_CAN_Receive_IT (hcan, CAN_FIFO0);
  NVIC_SystemReset();
}

// Initialize circular buffer
void
CAN_InitBuffer (CircularBuffer_t *cb)
{
  cb->head = 0;
  cb->tail = 0;
  cb->count = 0;
}

// Check if buffer is full
int8_t
CAN_IsBufferFull (CircularBuffer_t *cb)
{
  return cb->count == CAN_BUFFER_SIZE;
}

// Check if buffer is empty
int8_t
CAN_IsBufferEmpty (CircularBuffer_t *cb)
{
  return cb->count == 0;
}

// Add message to buffer
int8_t
CAN_EnqueueMessage (CircularBuffer_t *cb, CAN_Message_t *msg)
{
  if (CAN_IsBufferFull (cb))
    {
      return 0; // Buffer is full
    }
  cb->buffer[cb->head] = *msg;
  cb->head = (cb->head + 1) % CAN_BUFFER_SIZE;
  cb->count++;
  return 1;
}

int8_t
CAN_GetMessage (CircularBuffer_t *cb, CAN_Message_t *msg)
{
  if (CAN_IsBufferEmpty (cb))
    {
      return 0; // Buffer is empty
    }
  *msg = cb->buffer[cb->tail];
  return 1;
}

int8_t
CAN_DeleteMessage (CircularBuffer_t *cb)
{
  if (CAN_IsBufferEmpty (cb))
    {
      return 0; // Buffer is empty
    }
  cb->tail = (cb->tail + 1) % CAN_BUFFER_SIZE;
  cb->count--;
  return 1;
}

// Remove message from buffer
int8_t
CAN_DequeueMessage (CircularBuffer_t *cb, CAN_Message_t *msg)
{
  if (CAN_IsBufferEmpty (cb))
    {
      return 0; // Buffer is empty
    }

  *msg = cb->buffer[cb->tail];
  cb->tail = (cb->tail + 1) % CAN_BUFFER_SIZE;
  cb->count--;
  return 1;
}

// Transmit handler (called periodically or in main loop)
void
CAN_TransmitHandler (CAN_HandleTypeDef *hcan)
{
  CAN_Message_t msg;
  uint32_t tmstmp = HAL_GetTick ();
  for (; (CAN_IsBufferEmpty (&canTxBuffer) == 0)
//	  && ((HAL_GetTick () - tmstmp) < CAN_TRANSMITHANDLER_LIFETIME_MS)
      ;)
    {
      if (CAN_GetMessage (&canTxBuffer, &msg))
	{
	  if ((HAL_GetTick () - msg.timestamp) > CAN_MSG_LIFETIME_MS)
	    {
	      CAN_DeleteMessage (&canTxBuffer);
	      continue;
	    }
	  if (canState == CAN_IDLE)
	    {
#if CAN_MSG_BURST_DELAY_MS
	      if ((HAL_GetTick () - canTxLast_ms) > CAN_MSG_BURST_DELAY_MS)
		{
#endif // CAN_MSG_BURST_DELAY_MS
	      canTxLast_ms = HAL_GetTick ();
	      hcan->pTxMsg->StdId = msg.id;
	      hcan->pTxMsg->DLC = msg.dlc;
	      hcan->pTxMsg->IDE = CAN_ID_STD;
	      hcan->pTxMsg->RTR = CAN_RTR_DATA;
	      memcpy (&hcan->pTxMsg->Data[0], &msg.data[0], msg.dlc);
	      if (HAL_CAN_Transmit_IT (hcan) == HAL_OK)
		{
		  canState = CAN_SENDING; // Set state to sending
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
    }
  if (CAN_IsBufferEmpty (&canTxBuffer))
    {
      canState = CAN_IDLE;
    }
}

// Callback for transmission complete (called by interrupt)
void
CAN_TransmitCallback (CAN_HandleTypeDef *hcan)
{
  CAN_DeleteMessage(&canTxBuffer);
  canState = CAN_IDLE; // Set state back to idle
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
//  while(1)
//    {
//      HAL_Delay(250);
//      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
//    }
}

static uint8_t canRxMsg[8];
volatile int8_t canRxFlag = 0;

typedef enum
{
  e_can_machine_state_init = 0,
  e_can_machine_state_idle
} e_can_machine_state;

e_can_machine_state can_machine_state = e_can_machine_state_init;
void
can_machine (void)
{
  switch (can_machine_state)
    {
    case e_can_machine_state_init:
      {
	can_machine_state = e_can_machine_state_idle;
	canState = CAN_IDLE;
	canTxLast_ms = HAL_GetTick ();

	/* CAN interrupt Init */

	HAL_NVIC_SetPriority (CEC_CAN_IRQn, 2, 2);
	HAL_NVIC_EnableIRQ (CEC_CAN_IRQn);
	HAL_CAN_Receive_IT (&hcan, CAN_FIFO0);

	StartCan ();

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

void
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
