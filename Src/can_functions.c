/*
 * can_functions.c
 *
 *  Created on: Dec 19, 2024
 *      Author: blondier94
 */

#include "can_functions.h"
#include "AFE_functions.h"
#include <string.h>
#include "slcan/slcan.h"
#include <stm32f0xx_hal_can.h>
#include <stm32f0xx_hal_gpio.h>

#if WATCHDOG_FOR_CAN_RECIEVER_ENABLED
uint32_t afe_can_watchdog_timestamp_ms = 0;
uint32_t afe_can_watchdog_timeout_ms = 5*60*1000; // default 5
#endif // WATCHDOG_FOR_CAN_RECIEVER_ENABLED

#if USE_CAN_MSG_BURST_DELAY_MS
uint32_t canMsgBurstDelay_ms = 100;
#endif

extern CAN_HandleTypeDef hcan;
static CanTxMsgTypeDef CanTxBuffer;
static CanRxMsgTypeDef CanRxBuffer;

s_can_msg_recieved can_msg_received;
volatile int8_t canRxFlag = 0;

// CAN transmission state machine states
typedef enum
{
  e_CANMachineState_IDLE,
  e_CANMachineState_SENDING,
  e_CANMachineState_ERROR // New state for CAN bus error
} e_CANMachineState;

// Global variables
CANCircularBuffer_t canTxBuffer;
e_CANMachineState canState = e_CANMachineState_IDLE;
uint32_t canTxLast_ms = 0;

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
  HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_9);
}

GPIO_TypeDef*
GetGPIOPortByEnumerator (uint8_t enumerator)
{
  GPIO_TypeDef *port = NULL;
#if USE_SHIFTING_IN_SetPinFromArray
    // Calculate port address by shifting from GPIOA base
    port = (GPIO_TypeDef *)((uint32_t)GPIOA + (enumerator * ((uint32_t)GPIOB - (uint32_t)GPIOA)));
#else
  switch (enumerator)
    {
    case 0:
      port = (GPIO_TypeDef *)GPIOA;
      break;
    case 1:
      port = (GPIO_TypeDef *)GPIOB;
      break;
    case 2:
      port = (GPIO_TypeDef *)GPIOC;
      break;
    case 3:
      port = (GPIO_TypeDef *)GPIOD;
      break;
    case 4:
      port = (GPIO_TypeDef *)GPIOE;
      break;
    case 5:
      port = (GPIO_TypeDef *)GPIOF;
      break;
    default:
      break;
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
  return (rxCanMsg->IDE == CAN_ID_STD) && (own_id == ((rxCanMsg->StdId >> 2) & 0xFF));
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
  // uint32_t error_code = hcan->ErrorCode; // For debugging
  // Example: if (error_code & HAL_CAN_ERROR_BOF) { /* Bus-off occurred */ }

  canState = e_CANMachineState_ERROR;

  // Always attempt to re-enable CAN message reception interrupt.
  // If the bus recovers and a message is received, HAL_CAN_RxCpltCallback
  // will be called, which can then reset canState to IDLE.
  // If the peripheral is in a state like Bus-Off, this call might not immediately
  // lead to reception, but it readies the interrupt for when recovery happens.
  if (HAL_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK)
    {
      // Optionally handle failure to re-arm reception IT
    }
}

// Initialize circular buffer
void __attribute__ ((cold, optimize("-Os")))
CANCircularBuffer_Init (CANCircularBuffer_t *cb)
{
  cb->head = 0;
  cb->tail = 0;
}

// Check if buffer is empty
inline int8_t __attribute__ ((always_inline, optimize("-O3")))
CANCircularBuffer_isEmpty (CANCircularBuffer_t *cb)
{
  return cb->head == cb->tail;
}

// Add message to buffer
int8_t __attribute__ ((optimize("-O3")))
CANCircularBuffer_enqueueMessage (CANCircularBuffer_t *cb, CAN_Message_t *msg)
{
  memcpy (&cb->buffer[cb->head], msg, sizeof(*msg));
  ++cb->head;
  if (cb->head >= CAN_BUFFER_SIZE)
    {
      cb->head = 0;
    }
  if (cb->tail == cb->head)
    {
      ++cb->tail;
      if (cb->tail >= CAN_BUFFER_SIZE)
	{
	  cb->tail = 0;
	}
    }
  return 1;
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
      ++cb->tail;
      if (cb->tail >= CAN_BUFFER_SIZE)
	{
	  cb->tail = 0;
	}
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
  ++cb->tail;
  if (cb->tail >= CAN_BUFFER_SIZE)
    {
      cb->tail = 0;
    }
  return msg;
}


// Transmit handler (called periodically or in main loop)
void __attribute__ ((optimize("-O3")))
CAN_TransmitHandler (CAN_HandleTypeDef *hcan)
{
  // If an error is active or a transmission is already in progress, do nothing here.
  if (canState == e_CANMachineState_ERROR || canState == e_CANMachineState_SENDING)
    {
      return;
    }

  // At this point, canState == e_CANMachineState_IDLE

  CAN_Message_t *msg;

  msg = CANCircularBuffer_getMessage(&canTxBuffer); // Peek at the oldest message

  // Check for message timeout
  if (msg && (HAL_GetTick() - msg->timestamp) > CAN_MSG_LIFETIME_MS)
    {
      CANCircularBuffer_deleteMessage(&canTxBuffer); // Remove timed-out message
      // Let the next call to CAN_TransmitHandler process the new buffer state
      return;
    }

  // If message is valid (not timed out and exists)
  if (msg)
    {
#if USE_CAN_MSG_BURST_DELAY_MS
      if ((HAL_GetTick() - canTxLast_ms) <= canMsgBurstDelay_ms)
        {
          // Burst delay not yet elapsed
          return;
        }
#endif // CAN_MSG_BURST_DELAY_MS

      canTxLast_ms = HAL_GetTick();
      hcan->pTxMsg->StdId = msg->id;
      hcan->pTxMsg->DLC = msg->dlc;
      hcan->pTxMsg->IDE = CAN_ID_STD;
      hcan->pTxMsg->RTR = CAN_RTR_DATA;
      memcpy(&hcan->pTxMsg->Data[0], &msg->data[0], msg->dlc);

      if (HAL_CAN_Transmit_IT(hcan) == HAL_OK)
        {
          canState = e_CANMachineState_SENDING; // Set state to sending
        }
      else
        {
          // Transmission request failed (e.g., bus off or other hardware issue)
          canState = e_CANMachineState_ERROR;
        }
    }
}

// Callback for transmission complete (called by interrupt)
void __attribute__ ((optimize("-O3")))
CAN_TransmitCallback (CAN_HandleTypeDef *hcan)
{
  // If a transmission completed successfully, the bus might be recovering or fine.
  if (canState == e_CANMachineState_ERROR) {
      canState = e_CANMachineState_IDLE;
  }
  CANCircularBuffer_deleteMessage (&canTxBuffer);
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

void
modify_aurt_as_test_led (void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  // GPIO Ports Clock Enable
  __HAL_RCC_GPIOA_CLK_ENABLE();
  // Configure PIN A9: UART TX to GPIO
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
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

uint8_t can_machine_inited_0 = 0;
void
can_machine (void)
{
  switch (can_machine_state)
    {
    case e_can_machine_state_init:
      {
	can_machine_init_0 ();
	can_machine_state = e_can_machine_state_idle;
	canState = e_CANMachineState_IDLE;
	canTxLast_ms = HAL_GetTick ();

	/* CAN interrupt Init */
	HAL_NVIC_SetPriority (CEC_CAN_IRQn, 2, 2);
	HAL_NVIC_EnableIRQ (CEC_CAN_IRQn);
	HAL_CAN_Receive_IT (&hcan, CAN_FIFO0);

	StartCan ();
	CAN_Message_t tmp;
	// uint8_t command = AFECommand_resetAll;
	// uint32_t msg_id = CAN_ID_IN_MSG;
	typedef enum
	{
	  RESET_UNKNOWN = 0,
	  RESET_POWER_ON,
	  RESET_PIN,
	  RESET_BROWN_OUT,
	  RESET_SOFTWARE,
	  RESET_WATCHDOG,
	  RESET_WINDOW_WATCHDOG,
	  RESET_LOW_POWER
	} ResetReason_t;

	uint32_t timestamp_ms = HAL_GetTick ();
	tmp.timestamp = timestamp_ms;
	tmp.id = CAN_ID_IN_MSG;
	if (can_machine_inited_0)
	  {
	    tmp.data[0] = AFECommand_resetCAN;
	    CANCircularBuffer_enqueueMessage_data (&canTxBuffer, &tmp, 0, 1, 0x00,
						   (uint8_t*) &timestamp_ms, sizeof(uint32_t));
	  }
	else
	  {
	    tmp.data[0] = AFECommand_resetAll; // Standard reply [function]
	    tmp.data[1] = get_byte_of_message_number (0, 1); // Standard number of messages 1/1
	    tmp.data[2] = RCC->CSR;
	    RCC->CSR |= RCC_CSR_RMVF;  // Set the RMVF bit to clear all reset flags
	    __DSB ();  // Ensure the flag is cleared before continuing execution
	    tmp.dlc = 3;
	    CANCircularBuffer_enqueueMessage (&canTxBuffer, &tmp);
	    can_machine_inited_0 = 1;
	  }
#if WATCHDOG_FOR_CAN_RECIEVER_ENABLED
	afe_can_watchdog_timestamp_ms = HAL_GetTick ();
#endif
	break;
      }
    case e_can_machine_state_idle:
      {
#if WATCHDOG_FOR_CAN_RECIEVER_ENABLED
	if ((HAL_GetTick () - afe_can_watchdog_timestamp_ms) > afe_can_watchdog_timeout_ms)
	  {
	    NVIC_SystemReset (); // Reset if no respond
	  }
#endif // WATCHDOG_FOR_CAN_RECIEVER_ENABLED
	if (canState == e_CANMachineState_ERROR)
	  {
	    // NVIC_SystemReset (); // Reset if CAN error
	    can_machine_state = e_can_machine_state_init;
	  }

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
  return (0xF0 &
      ((total_msg_count - 1U) << 4U)) |
      ((msg_index) & 0x0F); // 0xY0 - max index, 0x0Z msg index
}

void
CANCircularBuffer_enqueueMessage_data (CANCircularBuffer_t *cb, CAN_Message_t *tmp,
				       uint8_t msg_index, uint8_t total_msg_count, uint8_t channel_mask,
				       uint8_t *value, uint8_t size)
{
  tmp->data[1] = get_byte_of_message_number (msg_index, total_msg_count);
  tmp->data[2] = channel_mask;
  tmp->dlc = 2 + 1 + size;
  memcpy (&tmp->data[3], value, size);
  CANCircularBuffer_enqueueMessage (cb, tmp);
}

void
CANCircularBuffer_enqueueMessage_data_float (CANCircularBuffer_t *cb, CAN_Message_t *tmp,
					     uint8_t msg_index, uint8_t total_msg_count,
					     uint8_t channel_mask, float *value)
{
  CANCircularBuffer_enqueueMessage_data (cb, tmp, msg_index, total_msg_count, channel_mask,
					 (uint8_t*) value, sizeof(float));
}

void
CANCircularBuffer_enqueueMessage_timestamp_ms (CANCircularBuffer_t *cb, CAN_Message_t *tmp,
					       uint8_t msg_index, uint8_t total_msg_count,
					       uint8_t channel_mask,
					       uint32_t timestamp_ms)
{
  tmp->data[1] = get_byte_of_message_number (msg_index, total_msg_count);
  tmp->data[2] = channel_mask;
  tmp->dlc = 2 + 1 + sizeof(uint32_t);
  memcpy (&tmp->data[3], &timestamp_ms, sizeof(uint32_t));
  CANCircularBuffer_enqueueMessage (cb, tmp);
}

void __attribute__ ((optimize("-O3")))
HAL_CAN_RxCpltCallback (CAN_HandleTypeDef *hcan)
{
  // If a message is successfully received, the bus is likely working.
  // Clear error state if it was set.
  if (canState == e_CANMachineState_ERROR) {
      canState = e_CANMachineState_IDLE;
  }

  if (is_this_msg_for_me (hcan->pRxMsg, AFE_CAN_ID) && (hcan->pRxMsg->DLC <= 8))
    {
      can_msg_received.DLC = (uint8_t)hcan->pRxMsg->DLC;
      can_msg_received.timestamp = HAL_GetTick ();
#if WATCHDOG_FOR_CAN_RECIEVER_ENABLED
      afe_can_watchdog_timestamp_ms = can_msg_received.timestamp;
#endif // WATCHDOG_FOR_CAN_RECIEVER_ENABLED
      memcpy (&can_msg_received.Data[0], &hcan->pRxMsg->Data[0], hcan->pRxMsg->DLC);
      canRxFlag = 1;
    }
  // Re-enable CAN message reception interrupt
  if (HAL_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK)
    {
        // If re-arming fails, could set error state again or log
        // canState = e_CANMachineState_ERROR;
    }
}
