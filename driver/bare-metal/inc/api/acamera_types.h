//----------------------------------------------------------------------------
//   The confidential and proprietary information contained in this file may
//   only be used by a person authorised under and to the extent permitted
//   by a subsisting licensing agreement from ARM Limited or its affiliates.
//
//          (C) COPYRIGHT [2019] ARM Limited or its affiliates.
//              ALL RIGHTS RESERVED
//
//   This entire notice must be reproduced on all copies of this file
//   and copies of this file may only be made by a person if such person is
//   permitted to do so under the terms of a subsisting license agreement
//   from ARM Limited or its affiliates.
//----------------------------------------------------------------------------

#ifndef __ACAMERA__TYPES_H__
#define __ACAMERA__TYPES_H__

#include "acamera_firmware_config.h"

#if KERNEL_MODULE == 1
#include "linux/types.h"
#else
#include <stdint.h>
#endif

typedef enum af_state {
    AF_STATE_INACTIVE,
    AF_STATE_SCAN,
    AF_STATE_FOCUSED,
    AF_STATE_UNFOCUSED
} af_state_t;

typedef enum ae_state {
    AE_STATE_INACTIVE,
    AE_STATE_SEARCHING,
    AE_STATE_CONVERGED,
} ae_state_t;

typedef enum awb_state {
    AWB_STATE_INACTIVE,
    AWB_STATE_SEARCHING,
    AWB_STATE_CONVERGED,
} awb_state_t;

typedef enum acamera_stream_type {
    ACAMERA_STREAM_META,
    ACAMERA_STREAM_RAW,
    ACAMERA_STREAM_FR,
    ACAMERA_STREAM_DS1,
    ACAMERA_STREAM_DS2,
    ACAMERA_STREAM_NUM_STREAM_TYPES
} acamera_stream_type_t;

typedef enum dma_buf_state {
    dma_buf_empty = 0,
    dma_buf_busy,
    dma_buf_ready,
    dma_buf_purge
} dma_buf_state;

/**
 *   supported pipe types
 *   please note that dma_max is used
 *   only as a counter for supported pipes
 */
typedef enum dma_type {
    dma_fr = 0,
    dma_ds1,
    dma_max
} dma_type;

typedef struct LookupTable {
    void *ptr;
    uint16_t rows;
    uint16_t cols;
    uint16_t width;
} LookupTable;

#endif /* __ACAMERA__TYPES_H__ */
