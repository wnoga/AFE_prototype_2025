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

#if defined (STM32F072xB)
#else
uint32_t __attribute__((weak))
HAL_GetTick (void)
{
  return (uint32_t) (clock () * 1000 / CLOCKS_PER_SEC);
}
#endif

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
}

inline size_t __attribute__ ((always_inline, optimize("-O3")))
CircularBuffer_GetItemCount (const s_BufferADC *cb)
{
  return cb->head >= cb->tail ? cb->head - cb->tail : cb->buffer_size - (cb->tail - cb->head);
}

int __attribute__ ((optimize("-O3")))
check_time_diff_is_more_than (uint32_t measurement_timestamp_ms, uint32_t timestamp_ms,
			      uint32_t max_dt_ms, uint32_t dt_ms)
{
  float tmp = (float) max_dt_ms + ((float) dt_ms / 10.0);
  tmp = roundf (tmp);

  if ((timestamp_ms - measurement_timestamp_ms) > (uint32_t) tmp)
    {
      return 1;
    }
  else
    {
      return 0;
    }
}

size_t __attribute__ ((optimize("-O3")))
get_n_latest_from_buffer_max_dt_ms (s_BufferADC *cb, size_t N,
				    s_ADC_Measurement *here,
				    uint32_t timestamp_ms, uint32_t max_dt_ms)
{
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
  return get_n_latest_from_buffer_max_dt_ms (cb, N, here, HAL_GetTick(), UINT32_MAX);
}

inline void __attribute__ ((always_inline, optimize("-O3")))
add_to_buffer (s_BufferADC *cb, const s_ADC_Measurement *measurement)
{
  if (cb->tail != cb->head) // Is not empty
    {
      if ((measurement->timestamp_ms
	  - cb->buffer[(cb->head != 0) ? cb->head - 1 : cb->buffer_size - 1].timestamp_ms)
	  < cb->dt_ms) // Check if this should be pushed to the buffer
	{
	  return;
	}
    }
  memcpy (&cb->buffer[cb->head], measurement, sizeof(s_ADC_Measurement));
  cb->head = (cb->head + 1) % cb->buffer_size;
  if (cb->head == cb->tail) // Buffer is full
    {
      cb->tail = (cb->tail + 1) % cb->buffer_size; // Move the tail to discard the oldest entry
    }
  return;
}



