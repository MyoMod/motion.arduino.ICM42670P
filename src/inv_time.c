/*
 *
 * Copyright (c) [2022] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */
 
#include "fsl_common.h"
#include "fsl_qtmr.h"
#include "peripherals.h"

void inv_imu_sleep_us(uint32_t us)
{
    SDK_DelayAtLeastUs(us, CLOCK_GetFreq(kCLOCK_CoreSysClk));
}

uint64_t inv_imu_get_time_us(void)
{
    // TODO:Maybe get this with a accuracy more than 1ms
    return ((uint64_t)QTMR_GetCurrentTimerCount(TMR1_PERIPHERAL, TMR1_MS_COUNTER_CHANNEL)) * 1000;
}