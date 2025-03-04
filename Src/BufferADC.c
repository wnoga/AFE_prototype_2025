/*
 * BufferADC.c
 *
 *  Created on: Oct 24, 2024
 *      Author: Wojciech Noga
 */

#include "BufferADC.h"
//#include <stdint.h>
//#include <stdlib.h>
//#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <time.h>

void
init_buffer (s_BufferADC *cb, s_ADC_Measurement *buffer, size_t buffer_size, uint32_t dt_ms)
{
//  cb->id = id;
  cb->head = 0;
  cb->tail = 0;
//  cb->count = 0;
  cb->buffer = buffer;
  if (buffer_size > MAX_BUFFER_ADC_SIZE)
    {
      printf("ERROR! File: %s line: %u\n",__FILE__,__LINE__);
      exit(-1);
    }
  else
    {
      cb->buffer_size = buffer_size;
    }
  cb->dt_ms = dt_ms;
  cb->buffer[0].timestamp_ms = 0;
  cb->buffer[0].adc_value = 0;
}

size_t CircularBuffer_GetItemCount(const s_BufferADC *cb) {
    // Check if the head has wrapped around or not
    if (cb->head >= cb->tail) {
        // Buffer is either partially filled or full without wrapping
        return cb->head - cb->tail;
    } else {
        // Buffer has wrapped around
        return cb->buffer_size - (cb->tail - cb->head);
    }
}

int check_time_diff_is_more_than(uint32_t measurement_timestamp_ms, uint32_t timestamp_ms,uint32_t max_dt_ms,uint32_t dt_ms)
{
  float tmp = (float)max_dt_ms + ((float)dt_ms / 10.0);
  tmp = roundf(tmp);

  if((timestamp_ms - measurement_timestamp_ms) > (uint32_t)tmp)
    {
      return 1;
    }
  return 0;
}

size_t cnt=0;

size_t
get_n_latest_from_buffer_max_dt_ms (s_BufferADC *cb, size_t N,
				    s_ADC_Measurement *here,
				    uint32_t timestamp_ms, uint32_t max_dt_ms)
{
//  if(cb->count == 0)
  if(cb->head == cb->tail)
    {
      return 0;
    }
  // Ensure N does not exceed the number of available measurements or buffer size
  size_t cb_count = CircularBuffer_GetItemCount(cb);
  if (N > cb_count)
    {
      N = cb_count; // Limit N to the current buffer count
    }
  size_t start_index = (cb->head == 0) ? (cb->buffer_size - 1) : (cb->head - 1);

  // Read N elements backwards in the buffer
  for (size_t i = 0; i < N; i++)
    {
//      if ((timestamp_ms - cb->buffer[start_index].timestamp_ms) > (max_dt_ms))
      if (check_time_diff_is_more_than (cb->buffer[start_index].timestamp_ms,
					timestamp_ms, max_dt_ms, cb->dt_ms)
	  && (max_dt_ms != 0))
	{
	  return i;
	}
      else
	{
	  here[i] = cb->buffer[start_index]; // Copy to output array
	  start_index =
	      (start_index == 0) ? (cb->buffer_size - 1) : (start_index - 1);
	}
    }
  return N;
}

size_t
get_n_latest_from_buffer (s_BufferADC *cb, size_t N, s_ADC_Measurement *here)
{
  return get_n_latest_from_buffer_max_dt_ms (cb, N, here, 0, 0);
}

void
add_to_buffer (s_BufferADC *cb, s_ADC_Measurement *measurement)
{
  s_ADC_Measurement tmp;
  if (get_n_latest_from_buffer (cb, 1, &tmp))
    {
      if (((measurement->timestamp_ms - tmp.timestamp_ms) < cb->dt_ms))
	{
	  return;
	}
    }
  cb->buffer[cb->head] = *measurement;
  cb->head = (cb->head + 1) % cb->buffer_size;
  if (cb->head == cb->tail) // Buffer is full
    {
      cb->tail = (cb->tail + 1) % cb->buffer_size; // Move the tail to discard the oldest entry
    }
  return;
}



