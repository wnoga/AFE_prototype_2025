/*
 * can_functions.h
 *
 *  Created on: Dec 19, 2024
 *      Author: blondier94
 */

#ifndef CAN_FUNCTIONS_H_
#define CAN_FUNCTIONS_H_

#include <stm32f0xx_hal.h>

extern ADC_HandleTypeDef hadc;
extern DAC_HandleTypeDef hdac;
extern CAN_HandleTypeDef hcan;
extern IWDG_HandleTypeDef hiwdg;
extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_adc;

#define CAN_ID 35
#define CAN_ID_IN_MSG ((1<<10) | (CAN_ID << 2))
extern const uint8_t verArr[];
extern const size_t verArrLen;
extern uint32_t UID[];

#define CAN_TRANSMITHANDLER_LIFETIME_MS 10
#define CAN_MSG_LIFETIME_MS 5000
#define CAN_BUFFER_SIZE 128
#define CAN_MSG_RECIEVED_TIMEOUT_MS 3000
#if CAN_BUFFER_SIZE <= 0xFF
typedef uint8_t can_bs_t;
#elif CAN_BUFFER_SIZE <= 0xFFFF
typedef uint16_t can_bs_t;
#elif CAN_BUFFER_SIZE <= 0xFFFFFFFF
typedef size_t can_bs_t;
#endif

// CAN Message structure
typedef struct {
    uint32_t id;       // CAN ID
    uint8_t data[8];   // CAN Data payload
    uint8_t dlc;       // Data Length Code
    uint32_t timestamp;// Msg timestamp, for timeout
} CAN_Message_t;

// Circular buffer structure
typedef struct {
    CAN_Message_t buffer[CAN_BUFFER_SIZE];
    can_bs_t head;      // Index for writing new data
    can_bs_t tail;      // Index for reading data
    can_bs_t count;     // Number of elements in the buffer
} CircularBuffer_t;

extern CircularBuffer_t canTxBuffer;

typedef enum
{
  e_CANIdFunctionCode_singleSet = 0b00, // Single set
  e_CANIdFunctionCode_singleRead = 0b01, // Single read
  e_CANIdFunctionCode_2 = 0b10, // Reserved
  e_CANIdFunctionCode_multipleRead = 0b11, // Reserved
} e_CANIdFunctionCode;

typedef struct
{
  uint32_t timestamp;
  uint32_t DLC;
  uint8_t Data[8];
} s_can_msg_recieved;

extern volatile int8_t canRxFlag;
extern s_can_msg_recieved can_msg_received;

GPIO_TypeDef * GetGPIOPortByEnumerator(uint8_t enumerator);
void SetPinFromArray(uint8_t *array);
GPIO_PinState ReadPinFromArray(uint8_t *array);

// Add message to buffer
int8_t CAN_EnqueueMessage (CircularBuffer_t *cb, CAN_Message_t *msg);

void can_machine_new_message_received(CanRxMsgTypeDef *new_msg);

HAL_StatusTypeDef configure_can_filter (CAN_HandleTypeDef *hcan, uint8_t own_id);
void modify_aurt_as_test_led(void);
void can_machine (void);

void _delay (size_t ms);
void blink1(void);

#endif /* CAN_FUNCTIONS_H_ */
