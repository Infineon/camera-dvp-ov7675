/******************************************************************************
 * \file mtb_dvp_camera_ov7675.c
 *
 * \brief
 *     This file contains the functions for interacting with the
 *     OV7675 camera using DVP interface.
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

#include "mtb_dvp_camera_ov7675.h"
#include "cybsp.h"


/*******************************************************************************
 * Macros
 ******************************************************************************/
#define I2C_CMD_DELAY_US    (2000)
#define BUFFER_COUNT        (2)
#define NUM_BYTES           (1)
#define I2C_TIMEOUT         (100)


/*******************************************************************************
* Global variables
*******************************************************************************/
__attribute__((section(".cy_sharedmem")))
__attribute((used))    uint8_t line_buffer[BUFFER_COUNT][LINE_SIZE];
static bool row_buffer_flag = false;
static bool frame_buffer_flag = false;
static vg_lite_buffer_t* image_frames = NULL;
static bool* _frame_ready = NULL;
static bool* _active_frame = NULL;
static cy_stc_scb_i2c_context_t* camera_i2c_context = NULL;


/*******************************************************************************
 * Camera settings
 ******************************************************************************/
/* OV7675 resolution options */
const ov7675_resolution_t OV7675_VGA = { 640, 480 };
const ov7675_resolution_t OV7675_QVGA = { 320, 240 };

/* Night mode initialization structure data */
const ov7675_output_format_config_t OV7675_FORMAT_RGB565 = { 0x04, 0xd0 };
const ov7675_output_format_config_t OV7675_FORMAT_RGB555 = { 0x04, 0xf0 };

/* resolution initialization structure data */
const ov7675_resolution_config_t OV7675_RESOLUTION_VGA = { 0x00 };    /*!< 640 x 480 */

const ov7675_resolution_config_t OV7675_RESOLUTION_QVGA = { 0x10 };    /*!< 320 x 240 */

/* Special effects configuration initialization structure data */
const ov7675_windowing_config_t OV7675_WINDOW_VGA = { 0x36, 0x13, 0x01, 0x0a, 0x02, 0x7a };
const ov7675_windowing_config_t OV7675_WINDOW_QVGA = { 0x80, 0x15, 0x03, 0x00, 0x03, 0x7b };

/* Frame rate initialization structure data */

const ov7675_frame_rate_config_t OV7675_30FPS_24MHZ_XCLK = { 0x00, 0x4a };
const ov7675_frame_rate_config_t OV7675_15FPS_24MHZ_XCLK = { 0x01, 0x4a };
const ov7675_frame_rate_config_t OV7675_05FPS_24MHZ_XCLK = { 0x04, 0x4a };

ov7675_handler_t s_Ov7675CameraHandler =
{
    .i2cBase       = CYBSP_I2C_CAM_CONTROLLER_HW,
    .i2cDeviceAddr = OV7675_I2C_ADDR
};

ov7675_config_t s_Ov7675CameraConfig =
{
    .outputFormat   = (ov7675_output_format_config_t*)&OV7675_FORMAT_RGB565,
    .resolution     =
    {
        .width      = OV7675_FRAME_WIDTH,
        .height     = OV7675_FRAME_HEIGHT,
    },
    .frameRate      = (ov7675_frame_rate_config_t*)&OV7675_05FPS_24MHZ_XCLK
};

/******************************************************************************/


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
cy_rslt_t mtb_dvp_cam_configure(void);
cy_rslt_t mtb_dvp_cam_dma_init(void);
cy_rslt_t mtb_dvp_cam_start_xclk(void);
void mtb_dvp_cam_intr_init(void);
void mtb_dvp_cam_intr_callback(void);
cy_rslt_t mtb_dvp_cam_axi_dmac_init(void);

cy_rslt_t master_write(CySCB_Type* base, cy_stc_scb_i2c_context_t* context,
                       uint16_t dev_addr,
                       const uint8_t* data, uint16_t size, uint32_t timeout,
                       bool send_stop);
cy_rslt_t master_read(CySCB_Type* base, cy_stc_scb_i2c_context_t* context,
                      uint16_t dev_addr,
                      uint8_t* data, uint16_t size, uint32_t timeout, bool send_stop);
cy_rslt_t I2C_Read_OV7675_Reg(CySCB_Type* base, uint8_t device_addr, uint8_t regAddr,
                              uint8_t* rxBuff, uint32_t rxSize);
cy_rslt_t I2C_Write_OV7675_Reg(CySCB_Type* base, uint8_t device_addr, uint8_t regAddr,
                               uint8_t val);
ov7675_status_t OV7675_ModifyReg(ov7675_handler_t* handle, uint8_t reg, uint8_t clrMask,
                                 uint8_t val);
ov7675_status_t OV7675_Init(ov7675_handler_t* handle, const ov7675_config_t* config);
ov7675_status_t OV7675_Configure(ov7675_handler_t* handle, const ov7675_config_t* config);
ov7675_status_t OV7675_OutputFormat(ov7675_handler_t* handle,
                                    const ov7675_output_format_config_t* outputFormatConfig);
ov7675_status_t OV7675_Resolution(ov7675_handler_t* handle, const ov7675_resolution_t* resolution);
ov7675_status_t OV7675_SetWindow(ov7675_handler_t* handle,
                                 const ov7675_windowing_config_t* windowingConfig);
ov7675_status_t OV7675_FrameRateAdjustment(ov7675_handler_t* handle,
                                           const ov7675_frame_rate_config_t* frameRateConfig);


/*******************************************************************************
* Function Name: master_write
*******************************************************************************/
cy_rslt_t master_write(CySCB_Type* base, cy_stc_scb_i2c_context_t* context,
                       uint16_t dev_addr,
                       const uint8_t* data, uint16_t size, uint32_t timeout,
                       bool send_stop)
{
    cy_rslt_t status = ((*context).state == CY_SCB_I2C_IDLE)
        ? Cy_SCB_I2C_MasterSendStart(base, dev_addr, CY_SCB_I2C_WRITE_XFER, timeout, context)
        : Cy_SCB_I2C_MasterSendReStart(base, dev_addr, CY_SCB_I2C_WRITE_XFER, timeout, context);

    if (status == CY_SCB_I2C_SUCCESS)
    {
        while (size > 0)
        {
            status = Cy_SCB_I2C_MasterWriteByte(base, *data, timeout, context);
            if (status != CY_SCB_I2C_SUCCESS)
            {
                break;
            }
            --size;
            ++data;
        }
    }

    if (send_stop)
    {
        /* SCB in I2C mode is very time sensitive.               */
        /* In practice we have to request STOP after each block, */
        /* otherwise it may break the transmission               */
        Cy_SCB_I2C_MasterSendStop(base, timeout, context);
    }

    return status;
}


/*******************************************************************************
* Function Name: master_read
*******************************************************************************/
cy_rslt_t master_read(CySCB_Type* base, cy_stc_scb_i2c_context_t* context,
                      uint16_t dev_addr,
                      uint8_t* data, uint16_t size, uint32_t timeout, bool send_stop)
{
    cy_en_scb_i2c_command_t ack = CY_SCB_I2C_ACK;

    /* Start transaction, send dev_addr */
    cy_rslt_t status = (*context).state == CY_SCB_I2C_IDLE
        ? Cy_SCB_I2C_MasterSendStart(base, dev_addr, CY_SCB_I2C_READ_XFER, timeout, context)
        : Cy_SCB_I2C_MasterSendReStart(base, dev_addr, CY_SCB_I2C_READ_XFER, timeout, context);
    if (status == CY_SCB_I2C_SUCCESS)
    {
        while (size > 0)
        {
            if (size == 1)
            {
                ack = CY_SCB_I2C_NAK;
            }
            status = Cy_SCB_I2C_MasterReadByte(base, ack, (uint8_t*)data, timeout, context);
            if (status != CY_SCB_I2C_SUCCESS)
            {
                break;
            }
            --size;
            ++data;
        }
    }

    if (send_stop)
    {
        /* SCB in I2C mode is very time sensitive.               */
        /* In practice we have to request STOP after each block, */
        /* otherwise it may break the transmission               */
        Cy_SCB_I2C_MasterSendStop(base, timeout, context);
    }
    return status;
}


/*******************************************************************************
* Function Name: delay
*******************************************************************************/
static void delay(int microseconds)
{
    Cy_SysLib_DelayUs(microseconds);
}


/*******************************************************************************
* Function Name: I2C_Read_OV7675_Reg
*******************************************************************************/
cy_rslt_t I2C_Read_OV7675_Reg(CySCB_Type* base, uint8_t device_addr, uint8_t regAddr,
                              uint8_t* rxBuff, uint32_t rxSize)
{
    cy_rslt_t status;
    uint8_t RegBuffer[1];
    RegBuffer[0] = regAddr;
    status = master_write(base, camera_i2c_context, (uint16_t)device_addr, RegBuffer,
                          NUM_BYTES, I2C_TIMEOUT, true);
    if (CY_RSLT_SUCCESS == status)
    {
        status = master_read(base, camera_i2c_context, (uint16_t)device_addr, rxBuff,
                             (uint16_t)rxSize, I2C_TIMEOUT, true);
        if (CY_RSLT_SUCCESS != status)
        {
            return status;
        }
    }
    return status;
}


/*******************************************************************************
* Function Name: I2C_Write_OV7675_Reg
*******************************************************************************/
cy_rslt_t I2C_Write_OV7675_Reg(CySCB_Type* base, uint8_t device_addr, uint8_t regAddr,
                               uint8_t val)
{
    cy_rslt_t status;
    uint8_t abWriteBuffer[2];
    abWriteBuffer[0] = regAddr;
    abWriteBuffer[1] = val;
    status = master_write(base, camera_i2c_context, (uint16_t)device_addr,
                          abWriteBuffer, BUFFER_COUNT, I2C_TIMEOUT, true);
    return status;
}


/*******************************************************************************
* Function Name: OV7675_ModifyReg
*******************************************************************************/
ov7675_status_t OV7675_ModifyReg(ov7675_handler_t* handle, uint8_t reg, uint8_t clrMask,
                                 uint8_t val)
{
    ov7675_status_t status = kStatus_OV7675_Success;

    cy_rslt_t ret_val = 0U;
    uint8_t reg_val = 0U;
    uint8_t tmp = 0U;
    ret_val = I2C_Read_OV7675_Reg(handle->i2cBase, handle->i2cDeviceAddr, reg, &reg_val, NUM_BYTES);
    if (ret_val != kStatus_OV7675_Success)
    {
        return kStatus_OV7675_Fail;
    }
    tmp = ~clrMask;
    reg_val &= tmp;
    reg_val |= val;
    ret_val = I2C_Write_OV7675_Reg(handle->i2cBase, handle->i2cDeviceAddr, reg, reg_val);
    if (ret_val != kStatus_OV7675_Success)
    {
        return kStatus_OV7675_Fail;
    }
    return status;
}


/*******************************************************************************
* Function Name: OV7675_Init
*******************************************************************************/
ov7675_status_t OV7675_Init(ov7675_handler_t* handle, const ov7675_config_t* config)
{
    ov7675_status_t status = kStatus_OV7675_Success;

    uint8_t u8TempVal0, u8TempVal1;

    /* Reset Device */
    I2C_Write_OV7675_Reg(handle->i2cBase, handle->i2cDeviceAddr, OV7675_COM7_REG,
                         OV7675_COM7_RESET_MASK);
    /* wait for at least 1ms */
    delay(I2C_CMD_DELAY_US);
    /* Read product ID number MSB */
    if (I2C_Read_OV7675_Reg(handle->i2cBase, handle->i2cDeviceAddr, OV7675_PID_REG, &u8TempVal0,
                            NUM_BYTES) != kStatus_OV7675_Success)
    {
        return kStatus_OV7675_Fail;
    }
    /* Read product ID number MSB */
    if (I2C_Read_OV7675_Reg(handle->i2cBase, handle->i2cDeviceAddr, OV7675_VER_REG, &u8TempVal1,
                            NUM_BYTES) != kStatus_OV7675_Success)
    {
        return kStatus_OV7675_Fail;
    }
    if ((u8TempVal0 != OV7675_PID_NUM) && (u8TempVal1 != OV7675_VER_NUM))
    {
        return kStatus_OV7675_Fail;
    }

    /* NULL pointer means default setting. */
    if (config != NULL)
    {
        I2C_Write_OV7675_Reg(handle->i2cBase, handle->i2cDeviceAddr, OV7675_COM10_REG,
                             OV7675_COM10_PCLK_HB_MASK | OV7675_COM10_HREF_REV_MASK);
        OV7675_Configure(handle, config);
    }


    return status;
}


/*******************************************************************************
* Function Name: OV7675_Configure
*******************************************************************************/
ov7675_status_t OV7675_Configure(ov7675_handler_t* handle, const ov7675_config_t* config)
{
    ov7675_status_t status = kStatus_OV7675_Success;

    ov7675_windowing_config_t* windowConfig;
    OV7675_OutputFormat(handle, config->outputFormat);
    OV7675_Resolution(handle, &config->resolution);
    uint32_t u32TempResolution = (config->resolution.width);
    u32TempResolution = (u32TempResolution << 16U);
    u32TempResolution |= config->resolution.height;

    switch (u32TempResolution)
    {
        case 0x028001e0:
            windowConfig = (ov7675_windowing_config_t*)&OV7675_WINDOW_VGA;
            break;

        case 0x014000f0:
            windowConfig = (ov7675_windowing_config_t*)&OV7675_WINDOW_QVGA;
            break;

        default:
            return kStatus_OV7675_Fail; /* not supported resolution */
    }
    OV7675_SetWindow(handle, windowConfig);
    OV7675_FrameRateAdjustment(handle, config->frameRate);

    return status;
}


/*******************************************************************************
* Function Name: OV7675_OutputFormat
*******************************************************************************/
ov7675_status_t OV7675_OutputFormat(ov7675_handler_t* handle,
                                    const ov7675_output_format_config_t* outputFormatConfig)
{
    ov7675_status_t status = kStatus_OV7675_Success;

    OV7675_ModifyReg(handle, OV7675_COM7_REG, OV7675_COM7_OUT_FMT_BITS, outputFormatConfig->com7);
    OV7675_ModifyReg(handle, OV7675_COM15_REG, OV7675_COM15_RGB_FMT_BITS,
                     outputFormatConfig->com15);

    /* Mirror the image */
    OV7675_ModifyReg(handle, OV7675_MVFP_REG, OV7675_MVFP_MIRROR_MASK, OV7675_MVFP_MIRROR_MASK);

    return status;
}


/*******************************************************************************
 * Function Name: OV7675_Resolution
 ********************************************************************************/
ov7675_status_t OV7675_Resolution(ov7675_handler_t* handle, const ov7675_resolution_t* resolution)
{
    ov7675_status_t status = kStatus_OV7675_Success;

    ov7675_resolution_config_t* resolution_config;
    uint32_t u32TempResolution;
    u32TempResolution = resolution->width;
    u32TempResolution = u32TempResolution << 16;
    u32TempResolution |= resolution->height;
    switch (u32TempResolution)
    {
        case 0x028001e0:
            resolution_config = (ov7675_resolution_config_t*)&OV7675_RESOLUTION_VGA;
            break;

        case 0x014000f0:
            resolution_config = (ov7675_resolution_config_t*)&OV7675_RESOLUTION_QVGA;
            break;

        default:
            return kStatus_OV7675_Fail; /*!< not supported resolution */
    }

    OV7675_ModifyReg(handle, OV7675_COM7_REG, OV7675_COM7_FMT_QVGA_MASK, resolution_config->com7);

    /* Automatically set output window after resolution change */
    OV7675_ModifyReg(handle, OV7675_TSLB_REG, OV7675_TSLB_AUTO_WIN_MASK, BIT_SET);

    return status;
}


/*******************************************************************************
* Function Name: OV7675_SetWindow
*******************************************************************************/
ov7675_status_t OV7675_SetWindow(ov7675_handler_t* handle,
                                 const ov7675_windowing_config_t* windowingConfig)
{
    ov7675_status_t status = kStatus_OV7675_Success;

    OV7675_ModifyReg(handle, OV7675_TSLB_REG, OV7675_TSLB_AUTO_WIN_MASK, BIT_CLEAR);

    I2C_Write_OV7675_Reg(handle->i2cBase, handle->i2cDeviceAddr, OV7675_HREF_REG,
                         windowingConfig->href);
    I2C_Write_OV7675_Reg(handle->i2cBase, handle->i2cDeviceAddr, OV7675_HSTART_REG,
                         windowingConfig->hstart);
    I2C_Write_OV7675_Reg(handle->i2cBase, handle->i2cDeviceAddr, OV7675_HSTOP_REG,
                         windowingConfig->hstop);
    I2C_Write_OV7675_Reg(handle->i2cBase, handle->i2cDeviceAddr, OV7675_VREF_REG,
                         windowingConfig->vref);
    I2C_Write_OV7675_Reg(handle->i2cBase, handle->i2cDeviceAddr, OV7675_VSTART_REG,
                         windowingConfig->vstart);
    I2C_Write_OV7675_Reg(handle->i2cBase, handle->i2cDeviceAddr, OV7675_VSTOP_REG,
                         windowingConfig->vstop);

    return status;
}


/*******************************************************************************
* Function Name: OV7675_FrameRateAdjustment
*******************************************************************************/
ov7675_status_t OV7675_FrameRateAdjustment(ov7675_handler_t* handle,
                                           const ov7675_frame_rate_config_t* frameRateConfig)
{
    ov7675_status_t status = kStatus_OV7675_Success;

    I2C_Write_OV7675_Reg(handle->i2cBase, handle->i2cDeviceAddr, OV7675_CLKRC_REG,
                         frameRateConfig->clkrc);
    delay(I2C_CMD_DELAY_US);
    I2C_Write_OV7675_Reg(handle->i2cBase, handle->i2cDeviceAddr, OV7675_DBLV_REG,
                         frameRateConfig->dblv);
    delay(I2C_CMD_DELAY_US);

    return status;
}


/*****************************************************************************
* Function Name: mtb_dvp_cam_configure
*****************************************************************************/
cy_rslt_t mtb_dvp_cam_configure(void)
{
    ov7675_status_t ov7675Status;

    /* Configure the OV7675 camera */
    ov7675Status = OV7675_Init(&s_Ov7675CameraHandler, (ov7675_config_t*)&s_Ov7675CameraConfig);

    return (cy_rslt_t)ov7675Status;
}


/*****************************************************************************
* Function Name: mtb_dvp_cam_start_xclk
*****************************************************************************/
cy_rslt_t mtb_dvp_cam_start_xclk(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = Cy_TCPWM_PWM_Init(CYBSP_PWM_DVP_CAM_CTRL_HW, CYBSP_PWM_DVP_CAM_CTRL_NUM,
                               &CYBSP_PWM_DVP_CAM_CTRL_config);
    if (CY_TCPWM_SUCCESS != result)
    {
        return result;
    }

    Cy_TCPWM_PWM_Enable(CYBSP_PWM_DVP_CAM_CTRL_HW, CYBSP_PWM_DVP_CAM_CTRL_NUM);
    /* Start the TCPWM block *   */
    Cy_TCPWM_TriggerStart_Single(CYBSP_PWM_DVP_CAM_CTRL_HW, CYBSP_PWM_DVP_CAM_CTRL_NUM);

    return result;
}


/*****************************************************************************
* Function Name: mtb_dvp_cam_intr_callback
*****************************************************************************/
void mtb_dvp_cam_intr_callback(void)
{
    if (Cy_GPIO_GetInterruptStatus(CYBSP_DVP_CAM_HREF_PORT, CYBSP_DVP_CAM_HREF_NUM))
    {
        Cy_GPIO_ClearInterrupt(CYBSP_DVP_CAM_HREF_PORT, CYBSP_DVP_CAM_HREF_NUM);
        NVIC_ClearPendingIRQ(CYBSP_DVP_CAM_HREF_IRQ);

        Cy_DMA_Descriptor_SetDstAddress(&CYBSP_DMA_DVP_CAM_CONTROLLER_Descriptor_0,
                                        (uint8_t*)line_buffer[row_buffer_flag]);
        #if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT != 0)
        SCB_InvalidateDCache_by_Addr((uint32_t*)line_buffer[row_buffer_flag],
                                     sizeof(line_buffer[row_buffer_flag]));
        SCB_CleanDCache_by_Addr((uint32_t*)&CYBSP_DMA_DVP_CAM_CONTROLLER_Descriptor_0,
                                sizeof(CYBSP_DMA_DVP_CAM_CONTROLLER_Descriptor_0));
        #endif
        Cy_DMA_Channel_Enable(CYBSP_DMA_DVP_CAM_CONTROLLER_HW,
                              CYBSP_DMA_DVP_CAM_CONTROLLER_CHANNEL);

        row_buffer_flag = !row_buffer_flag;
    }

    if (Cy_GPIO_GetInterruptStatus(CYBSP_DVP_CAM_VSYNC_PORT, CYBSP_DVP_CAM_VSYNC_NUM))
    {
        Cy_GPIO_ClearInterrupt(CYBSP_DVP_CAM_VSYNC_PORT, CYBSP_DVP_CAM_VSYNC_NUM);
        NVIC_ClearPendingIRQ(CYBSP_DVP_CAM_VSYNC_IRQ);

        Cy_AXIDMAC_Descriptor_SetDstAddress(&CYBSP_AXIDMAC_DVP_CAM_CONTROLLER_Descriptor_0,
                                            (uint32_t*)image_frames[frame_buffer_flag].address);
        #if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT != 0)
        SCB_InvalidateDCache_by_Addr((uint32_t*)image_frames[frame_buffer_flag].address,
                                     sizeof(image_frames[frame_buffer_flag].address));
        SCB_CleanDCache_by_Addr((uint32_t*)&CYBSP_AXIDMAC_DVP_CAM_CONTROLLER_Descriptor_0,
                                sizeof(CYBSP_AXIDMAC_DVP_CAM_CONTROLLER_Descriptor_0));
        #endif
        Cy_AXIDMAC_Channel_Enable(CYBSP_AXIDMAC_DVP_CAM_CONTROLLER_HW,
                                  CYBSP_AXIDMAC_DVP_CAM_CONTROLLER_CHANNEL);

        frame_buffer_flag = !frame_buffer_flag;
        *_active_frame = frame_buffer_flag;
        *_frame_ready = true;
    }
}


/*****************************************************************************
* Function Name: mtb_dvp_cam_intr_init
*****************************************************************************/
void mtb_dvp_cam_intr_init(void)
{
    /* Interrupt config structure */
    cy_stc_sysint_t tIntrCfg =
    {
        .intrSrc      = CYBSP_DVP_CAM_HREF_IRQ, /* Interrupt source */
        .intrPriority = 7UL /* Interrupt priority */
    };
    /* Initialize the interrupt */
    Cy_GPIO_ClearInterrupt(CYBSP_DVP_CAM_HREF_PORT, CYBSP_DVP_CAM_HREF_PIN);
    NVIC_ClearPendingIRQ(CYBSP_DVP_CAM_HREF_IRQ);
    Cy_GPIO_ClearInterrupt(CYBSP_DVP_CAM_VSYNC_PORT, CYBSP_DVP_CAM_VSYNC_PIN);
    NVIC_ClearPendingIRQ(CYBSP_DVP_CAM_VSYNC_IRQ);

    Cy_SysInt_Init(&tIntrCfg, &mtb_dvp_cam_intr_callback);

    /* Enable the interrupt. Since both HREF and VSYNC pins have the same
     * interrupt source, enabling any one is sufficient */
    NVIC_EnableIRQ(CYBSP_DVP_CAM_HREF_IRQ);
}


/*****************************************************************************
* Function Name: mtb_dvp_cam_dma_init
*****************************************************************************/
cy_rslt_t mtb_dvp_cam_dma_init(void)
{
    cy_rslt_t status;

    status = Cy_DMA_Descriptor_Init(&CYBSP_DMA_DVP_CAM_CONTROLLER_Descriptor_0,
                                    &CYBSP_DMA_DVP_CAM_CONTROLLER_Descriptor_0_config);
    if (status != CY_DMA_SUCCESS)
    {
        return status;
    }

    status = Cy_DMA_Channel_Init(CYBSP_DMA_DVP_CAM_CONTROLLER_HW,
                                 CYBSP_DMA_DVP_CAM_CONTROLLER_CHANNEL,
                                 &CYBSP_DMA_DVP_CAM_CONTROLLER_channelConfig);
    if (status != CY_DMA_SUCCESS)
    {
        return status;
    }

    memset(line_buffer[0], 0x0, LINE_SIZE);
    memset(line_buffer[1], 0x0, LINE_SIZE);

    Cy_DMA_Descriptor_SetSrcAddress(&CYBSP_DMA_DVP_CAM_CONTROLLER_Descriptor_0,
                                    (void*)&GPIO_PRT16->IN);

    Cy_DMA_Descriptor_SetDstAddress(&CYBSP_DMA_DVP_CAM_CONTROLLER_Descriptor_0,
                                    line_buffer[row_buffer_flag]);

    #if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT != 0)
    SCB_CleanDCache_by_Addr((uint32_t*)&GPIO_PRT16->IN, sizeof(GPIO_PRT16->IN));
    SCB_CleanDCache_by_Addr((uint32_t*)&CYBSP_DMA_DVP_CAM_CONTROLLER_Descriptor_0,
                            sizeof(CYBSP_DMA_DVP_CAM_CONTROLLER_Descriptor_0));
    #endif

    Cy_DMA_Enable(CYBSP_DMA_DVP_CAM_CONTROLLER_HW);
    Cy_DMA_Channel_Enable(CYBSP_DMA_DVP_CAM_CONTROLLER_HW, CYBSP_DMA_DVP_CAM_CONTROLLER_CHANNEL);

    return status;
}


/*****************************************************************************
* Function Name: mtb_dvp_cam_axi_dmac_init
*****************************************************************************/
cy_rslt_t mtb_dvp_cam_axi_dmac_init(void)
{
    cy_rslt_t status;

    status = Cy_AXIDMAC_Descriptor_Init(&CYBSP_AXIDMAC_DVP_CAM_CONTROLLER_Descriptor_0,
                                        &CYBSP_AXIDMAC_DVP_CAM_CONTROLLER_Descriptor_0_config);
    if (CY_AXIDMAC_SUCCESS != status)
    {
        return status;
    }

    status = Cy_AXIDMAC_Channel_Init(CYBSP_AXIDMAC_DVP_CAM_CONTROLLER_HW,
                                     CYBSP_AXIDMAC_DVP_CAM_CONTROLLER_CHANNEL,
                                     &CYBSP_AXIDMAC_DVP_CAM_CONTROLLER_channelConfig);
    if (CY_AXIDMAC_SUCCESS != status)
    {
        return status;
    }

    Cy_AXIDMAC_Descriptor_SetSrcAddress(&CYBSP_AXIDMAC_DVP_CAM_CONTROLLER_Descriptor_0,
                                        (uint32_t*)line_buffer);
    Cy_AXIDMAC_Descriptor_SetDstAddress(&CYBSP_AXIDMAC_DVP_CAM_CONTROLLER_Descriptor_0,
                                        (uint32_t*)image_frames[frame_buffer_flag].address);

    #if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT != 0)
    SCB_CleanDCache_by_Addr((uint32_t*)&line_buffer, sizeof(line_buffer));
    SCB_CleanDCache_by_Addr((uint32_t*)&CYBSP_AXIDMAC_DVP_CAM_CONTROLLER_Descriptor_0,
                            sizeof(CYBSP_AXIDMAC_DVP_CAM_CONTROLLER_Descriptor_0));
    #endif

    Cy_AXIDMAC_Enable(CYBSP_AXIDMAC_DVP_CAM_CONTROLLER_HW);
    Cy_AXIDMAC_Channel_Enable(CYBSP_AXIDMAC_DVP_CAM_CONTROLLER_HW,
                              CYBSP_AXIDMAC_DVP_CAM_CONTROLLER_CHANNEL);

    return status;
}


/*****************************************************************************
* Function Name: mtb_dvp_cam_ov7675_init
*****************************************************************************/
cy_rslt_t mtb_dvp_cam_ov7675_init(vg_lite_buffer_t* buffer, cy_stc_scb_i2c_context_t* i2c_instance,
                                  bool* _frame_ready_flag, bool* _active_frame_flag)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;
    image_frames = buffer;
    _frame_ready = _frame_ready_flag;
    _active_frame = _active_frame_flag;

    camera_i2c_context = i2c_instance;
    CY_ASSERT(NULL != i2c_instance);

    /* Initialize DMA DW*/
    status = mtb_dvp_cam_dma_init();
    if (CY_RSLT_SUCCESS != status)
    {
        return status;
    }

    /* Initialize AXIDMAC*/
    status = mtb_dvp_cam_axi_dmac_init();
    if (CY_RSLT_SUCCESS != status)
    {
        return status;
    }

    /* Initialize HREF interrupt */
    mtb_dvp_cam_intr_init();

    /* Initialize and start XCLK*/
    status = mtb_dvp_cam_start_xclk();
    if (CY_RSLT_SUCCESS != status)
    {
        return status;
    }

    /* Initialize the camera with set configurations */
    status = mtb_dvp_cam_configure();
    if (CY_RSLT_SUCCESS != status)
    {
        return status;
    }

    return status;
}
