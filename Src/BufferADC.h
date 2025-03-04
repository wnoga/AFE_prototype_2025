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


#define MAX_BUFFER_ADC_SIZE 128

// Structure to store ADC measurement and timestamp
typedef struct
{
  uint16_t adc_value; // Raw ADC value (assuming 10-bit ADC, adjust based on resolution)
  uint32_t timestamp_ms; // Timestamp in milliseconds since MCU start
} s_ADC_Measurement;

// Circular buffer structure
typedef struct
{
//    ADC_Measurement buffer[MAX_BUFFER_SIZE]; // Array of ADC measurements
  s_ADC_Measurement *buffer;
  size_t head;  // Points to the next insertion position
  size_t tail;  // Points to the next element to be read
//    size_t count; // Current number of elements in the buffer
  size_t buffer_size;
  uint32_t dt_ms; // Minimum time in milliseconds between measurements
//  size_t id; // Buffer ID
} s_BufferADC;

void init_buffer (s_BufferADC *cb, s_ADC_Measurement *buffer,
	     size_t buffer_size, uint32_t dt_ms);

size_t CircularBuffer_GetItemCount (const s_BufferADC *cb);

int check_time_diff_is_more_than (uint32_t measurement_timestamp_ms,
			      uint32_t timestamp_ms, uint32_t max_dt_ms,
			      uint32_t dt_ms);

size_t get_n_latest_from_buffer_max_dt_ms (s_BufferADC *cb, size_t N,
				    s_ADC_Measurement *here,
				    uint32_t timestamp_ms, uint32_t max_dt_ms);

size_t get_n_latest_from_buffer (s_BufferADC *cb, size_t N, s_ADC_Measurement *here);

void add_to_buffer (s_BufferADC *cb, s_ADC_Measurement *measurement);


#endif /* BUFFERADC_H_ */
