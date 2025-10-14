/******************************************************************************
 * \file mtb_dvp_camera_ov7675.h
 *
 * \brief
 *     This file is the public interface of the OV7675 camera sensor.
 *
 ********************************************************************************
 * \copyright
 * Copyright 2025 Cypress Semiconductor Corporation
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif


#include "mtb_dvp_camera_ov7675_def.h"
#include "cy_pdl.h"
#include "vg_lite.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define OV7675_I2C_ADDR             (0x21U)
#define OV7675_BYTES_PER_PIXEL      (2u)
#define OV7675_FRAME_WIDTH          (320u)
#define OV7675_FRAME_HEIGHT         (240u)
#define LINE_SIZE                   (OV7675_FRAME_WIDTH * OV7675_BYTES_PER_PIXEL)

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

/** Structure containing resolution parameters of the camera */
typedef struct ov7675_resolution
{
    uint16_t width;
    uint16_t height;
} ov7675_resolution_t;

/** Output format configuration structure */
typedef struct ov7675_output_format_config
{
    uint8_t com7;
    uint8_t com15;
} ov7675_output_format_config_t;

/** Resolution configuration structure */
typedef struct ov7675_resolution_config
{
    uint8_t com7;
} ov7675_resolution_config_t;

/** Windowing configuration structure */
typedef struct ov7675_windowing_config
{
    uint8_t href;
    uint8_t hstart;
    uint8_t hstop;
    uint8_t vref;
    uint8_t vstart;
    uint8_t vstop;
} ov7675_windowing_config_t;

/** Frame rate configuration structure */
typedef struct ov7675_frame_rate_config
{
    uint8_t clkrc;
    uint8_t dblv;
} ov7675_frame_rate_config_t;

/** OV7675 return status. */
typedef enum _ov7675_status
{
    kStatus_OV7675_Success = 0x0, /* success */
    kStatus_OV7675_I2CFail = 0x1, /* I2C failure */
    kStatus_OV7675_Fail = 0x2    /* fail */
} ov7675_status_t;

/** OV7675 handler configuration structure */
typedef struct ov7675_handler
{
    /* I2C related definition. */
    CySCB_Type* i2cBase;     /* I2C instance. */
    uint8_t i2cDeviceAddr; /* I2C device address */
} ov7675_handler_t;

/** Initialization structure of OV7675 */
typedef struct ov7675_config
{
    ov7675_output_format_config_t* outputFormat;
    ov7675_resolution_t resolution;
    ov7675_frame_rate_config_t* frameRate;
} ov7675_config_t;

/******************************************************************************/


/******************************************************************************
* Function Name: mtb_dvp_cam_ov7675_init
*******************************************************************************
* Summary:
*  This function initializes the OV7675 DVP camera (with a fixed configuration)
*  and the MCU hardware resources that are required for interfacing the camera.
*  It sets the frame_ready_flag flag that indicates whether the image frame is ready,
*  and the active_frame_flag flag that indicates which frame buffer is active/ready.
*  The "active_frame_flag == true" indicates that the index 1 of the double buffer is active.
*
* Parameters:
*  buffer               Pointer to the image buffer
*  i2c_instance         Pointer to an initialized I2C object context
*  frame_ready_flag     Pointer to the flag that indicates whether the image frame is ready
*  active_frame_flag    Pointer to the flag that indicates which frame buffer is active/ready
*                       in the double buffer
*
* Return: cy_rslt_t -> Status of the execution
*
******************************************************************************/
cy_rslt_t mtb_dvp_cam_ov7675_init(vg_lite_buffer_t* buffer, cy_stc_scb_i2c_context_t* i2c_instance,
                                  bool* frame_ready_flag, bool* active_frame_flag);


#if defined(__cplusplus)
}
#endif
