/*
 * BufferADC.c
 *
 *  Created on: Oct 24, 2024
 *      Author: Wojciech Noga
 */

#include "BufferADC.h"
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <time.h>

#include <stdint.h>

void __attribute__ ((cold, optimize("-Os")))
init_buffer (s_BufferADC *cb, s_ADC_Measurement *buffer, size_t buffer_size, uint32_t dt_ms)
{
  cb->head = 0;
  cb->tail = 0;
  cb->buffer = buffer;
  cb->buffer_size = buffer_size;
  cb->dt_ms = dt_ms;
  cb->buffer[0].timestamp_ms = 0;
  cb->buffer[0].adc_value = 0;
#if USE_STACK_FOR_BUFFER
  free(cb->buffer);
  cb->buffer = malloc(buffer_size * sizeof(s_ADC_Measurement));
#else
#endif
  memset (cb->buffer, 0, buffer_size * sizeof(s_ADC_Measurement));
}

inline size_t __attribute__ ((always_inline, optimize("-O3")))
CircularBuffer_GetItemCount (s_BufferADC *cb)
{
  return cb->head >= cb->tail ? cb->head - cb->tail : cb->buffer_size - (cb->tail - cb->head);
}

size_t __attribute__ ((optimize("-O3")))
get_n_latest_from_buffer_max_dt_ms (s_BufferADC *cb, size_t N, s_ADC_Measurement *here,
				    uint32_t timestamp_ms, uint32_t max_dt_ms)
{
  if (cb->head == cb->tail)
    {
      return 0;
    }
  // Ensure N does not exceed the number of available measurements or buffer size
  size_t cb_count = CircularBuffer_GetItemCount (cb);
  if (N > cb_count)
    {
      N = cb_count; // Limit N to the current buffer count
    }
  size_t start_index = (cb->head == 0) ? (cb->buffer_size - 1) : (cb->head - 1);
  size_t index = start_index;
  // Read N elements backwards in the buffer
  for (size_t i = 0; i < N; i++)
    {
      if ((timestamp_ms - cb->buffer[index].timestamp_ms) > max_dt_ms)
	{
	  return i;
	}
      else
	{
	  here[i] = cb->buffer[index]; // Copy to output array
	  index = (index != 0) ? (index - 1) : (cb->buffer_size - 1); // Decrement index
	}
    }
  return N;
}

size_t
get_n_latest_from_buffer (s_BufferADC *cb, size_t N, s_ADC_Measurement *here)
{
  return get_n_latest_from_buffer_max_dt_ms (cb, N, here, HAL_GetTick (), UINT32_MAX);
}

inline void __attribute__ ((always_inline, optimize("-O3")))
add_to_buffer (s_BufferADC *cb, s_ADC_Measurement *measurement)
{
  // Check if the buffer is not empty
  if (cb->tail != cb->head)
    {
      // Calculate the index of the last element added to the buffer
      size_t last_index = (cb->head != 0) ? cb->head - 1 : cb->buffer_size - 1;
      // Check if the time difference between the new measurement and the last measurement is less than the minimum allowed time difference (dt_ms)
      if ((measurement->timestamp_ms - cb->buffer[last_index].timestamp_ms) < cb->dt_ms)
	{
	  // If the time difference is too small, do not add the new measurement
	  return;
	}
    }
  // Copy the new measurement to the buffer at the current head position
  cb->buffer[cb->head].adc_value = measurement->adc_value;
  cb->buffer[cb->head].timestamp_ms = measurement->timestamp_ms;
  cb->last_data_ms = measurement->timestamp_ms;

  // Increment head, handling wrap-around
  ++cb->head;
  if (cb->head >= cb->buffer_size)
    {
      cb->head = 0; // Wrap around
    }

  // Check if buffer is full (head has wrapped around and caught up with tail)
  if (cb->head == cb->tail)
    {
      // Move the tail to discard the oldest entry, handling wrap-around
      ++cb->tail;
      if (cb->tail >= cb->buffer_size)
	{
	  cb->tail = 0;
	}
    }
}
