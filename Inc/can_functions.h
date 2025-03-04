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

#define CAN_ID 8
#define CAN_ID_IN_MSG (CAN_ID << 2)
extern const uint8_t verArr[];
extern const size_t verArrLen;
extern uint32_t UID[];

#define CAN_TRANSMITHANDLER_LIFETIME_MS 10
#define CAN_MSG_LIFETIME_MS 5000
#define CAN_BUFFER_SIZE 128
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

#define CAN_MSG_RECIEVED_TIMEOUT_MS 3000
typedef struct
{
  uint32_t timestamp;
//  uint32_t StdId;
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

//self.get_sensor_data_si = 0x30
//self.averaging_mode = 0xD0
//self.bin_duration_ms = 0xD1
//self.time_interval_ms = 0xD3
//self.scaling_factor = 0xD5
//self.get_timestamp = 0xE0

typedef enum
{
  AFECommand_getSerialNumber = 0x00,
  AFECommand_getVersion = 0x01,
  AFECommand_resetAll = 0x03,

  AFECommand_getSensorDataSi_last = 0x30,
  AFECommand_getSensorDataSi_average = 0x31,
  AFECommand_getSensorDataSi_all_last = 0x32,
  AFECommand_getSensorDataSi_all_average = 0x33,
  AFECommand_setSensorDataSi_all_periodic_average = 0x34,

  AFECommand_getSensorDataSiAndTimestamp_average = 0x3B,
  AFECommand_getSensorDataSi_all_periodic_average = 0x3F,

  AFECOMMAND_setSensorDataSi_periodic_last = 0x40,
  AFECOMMAND_setSensorDataSiAndTimestamp_periodic_last = 0x41,
  AFECOMMAND_setSensorDataSi_periodic_average = 0x42,
  AFECOMMAND_setSensorDataSiAndTimestamp_periodic_average = 0x43,

  AFECommand_transmitSPIData = 0xA0,
  AFECommand_writeGPIO = 0xA2,

  AFECommand_setTemperatureLoopForChannelState_bySubdevice = 0xC0,
  AFECommand_setTemperatureLoopForChannelState_byMask = 0xC1,
  AFECommand_setDACValueRaw_bySubdevice = 0xC2,
  AFECommand_setDACValueSi_bySubdevice = 0xC3,
  AFECommand_stopTemperatureLoopForAllChannels = 0xC4,
  AFECommand_setDAC_bySubdevice = 0xC5,

  AFECommand_setAveragingMode = 0xD0,
  AFECommand_setAveragingAlpha = 0xD1,
  AFECommand_setAveragingBufferSize = 0xD2,
  AFECommand_setAveragingDt_ms = 0xD3,
  AFECommand_setAveragingMaxDt_ms = 0xD4,
  AFECommand_setAveragingMultiplicator = 0xD5,
  AFECommand_setAveragingSubdevice = 0xD6,
} AFECommand;

typedef enum
{
  AFECommandChannel_0 = 0b00000001,
  AFECommandChannel_1 = 0b00000010,
  AFECommandChannel_2 = 0b00000100,
  AFECommandChannel_3 = 0b00001000,
  AFECommandChannel_4 = 0b00010000,
  AFECommandChannel_5 = 0b00100000,
  AFECommandChannel_6 = 0b01000000,
  AFECommandChannel_7 = 0b10000000,
} AFECommandChannel;

typedef enum
{
  AFECommandSubdevice_master 	= 0b00000001,
  AFECommandSubdevice_slave 	= 0b00000010,
  AFECommandSubdevice_both 	= 0b00000011,
} AFECommandSubdevice;

#endif /* CAN_FUNCTIONS_H_ */
