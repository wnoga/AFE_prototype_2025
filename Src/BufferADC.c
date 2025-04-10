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
#include <stm32f0xx_hal.h>
#else
#include "mock_hal.h"
#endif

/**
 * @brief Initializes the circular buffer for ADC measurements.
 *
 * This function sets up the circular buffer by initializing its head and tail pointers,
 * assigning the provided buffer, setting the buffer size, and specifying the minimum
 * time difference between measurements. It also initializes the first element of the buffer.
 *
 * @param cb Pointer to the circular buffer structure.
 * @param buffer Pointer to the array where ADC measurements will be stored.
 */
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
  memset (cb->buffer, 0, buffer_size * sizeof(s_ADC_Measurement));
}

/**
 * @brief Gets the number of items currently stored in the circular buffer.
 *
 * This function calculates the number of items in the circular buffer by
 * comparing the head and tail pointers. It handles the wrap-around case
 * where the tail is ahead of the head.
 *
 * @param cb Pointer to the circular buffer structure.
 * @return The number of items in the buffer.
 */
inline size_t __attribute__ ((always_inline, optimize("-O3")))
CircularBuffer_GetItemCount (const s_BufferADC *cb)
{
  return cb->head >= cb->tail ? cb->head - cb->tail : cb->buffer_size - (cb->tail - cb->head);
}

/**
 * @brief Retrieves the N latest measurements from the buffer, considering time constraints.
 *
 * This function retrieves up to N of the most recent measurements from the circular buffer,
 * ensuring that each measurement's timestamp is within the specified time window.
 *
 * @param cb Pointer to the circular buffer structure.
 * @param N The maximum number of measurements to retrieve.
 * @param here Pointer to the array where the retrieved measurements will be stored.
 * @param timestamp_ms The current timestamp, used to check the time window.
 * @param max_dt_ms The maximum allowed time difference between the current timestamp and a measurement's timestamp.
 * @return The number of measurements actually retrieved.
 */
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
	  break;
	}
      else
	{
	  here[i] = cb->buffer[index]; // Copy to output array
	  index = (index == 0) ? (cb->buffer_size - 1) : (index - 1); // Decrement index
	}
    }
  return N;
}

/**
 * @brief Retrieves the N latest measurements from the buffer without time constraints.
 *
 * This function retrieves the N most recent measurements from the circular buffer
 * without considering any time window. It uses the HAL_GetTick() function to get
 * the current timestamp and sets the maximum time difference to the maximum value of uint32_t.
 *
 * @param cb Pointer to the circular buffer structure.
 * @param N The maximum number of measurements to retrieve.
 * @param here Pointer to the array where the retrieved measurements will be stored.
 * @return The number of measurements actually retrieved.
 */
size_t
get_n_latest_from_buffer (s_BufferADC *cb, size_t N, s_ADC_Measurement *here)
{
  return get_n_latest_from_buffer_max_dt_ms (cb, N, here, HAL_GetTick (), UINT32_MAX);
}

/**
 * @brief Adds a new measurement to the circular buffer.
 */
inline void __attribute__ ((always_inline, optimize("-O3")))
add_to_buffer (s_BufferADC *cb, const s_ADC_Measurement *measurement)
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
  memcpy (&cb->buffer[cb->head], measurement, sizeof(s_ADC_Measurement));

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
