/*
 * BufferADC.h
 *
 *  Created on: Oct 24, 2024
 *      Author: Wojciech Noga
 */

#ifndef BUFFERADC_H_
#define BUFFERADC_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

uint32_t __attribute__ ((weak))
HAL_GetTick(void)
{
  return 0;
}

// Structure to store ADC measurement and timestamp
typedef struct __attribute__((packed))
{
  uint32_t timestamp_ms; // Timestamp in milliseconds since MCU start
  uint16_t adc_value; // Raw ADC value (assuming 10-bit ADC, adjust based on resolution)
} s_ADC_Measurement;

// Circular buffer structure
typedef struct __attribute__((packed))
{
  s_ADC_Measurement *buffer;
  size_t head;  // Points to the next insertion position
  size_t tail;  // Points to the next element to be read
  size_t buffer_size;
  uint32_t dt_ms; // Minimum time in milliseconds between measurements
  uint32_t last_data_ms;
} s_BufferADC;

void init_buffer (s_BufferADC *cb, s_ADC_Measurement *buffer, size_t buffer_size, uint32_t dt_ms);
size_t CircularBuffer_GetItemCount (s_BufferADC *cb);
int check_time_diff_is_more_than (uint32_t measurement_timestamp_ms, uint32_t timestamp_ms, uint32_t max_dt_ms, uint32_t dt_ms);
size_t get_n_latest_from_buffer_max_dt_ms (s_BufferADC *cb, size_t N,s_ADC_Measurement *here, uint32_t timestamp_ms, uint32_t max_dt_ms);
size_t get_n_latest_from_buffer (s_BufferADC *cb, size_t N, s_ADC_Measurement *here);
void add_to_buffer (s_BufferADC *cb, s_ADC_Measurement *measurement);


#endif /* BUFFERADC_H_ */
