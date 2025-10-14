# OV7675 DVP Camera driver library for ModusToolbox&trade;

## Overview

This driver library provides functions for supporting [OV7675 DVP Camera module](https://www.arducam.com/640x480-0-3-mp-mega-pixel-lens-ov7675-cmos-camera-module-with-adapter-board.html) from OmniVision on PSOC&trade; Edge device.

## Quick start

Follow these steps to add the driver in an application for PSOC&trade; Edge AI Kit

1. Create a [PSOC&trade; Edge MCU: Empty application](https://github.com/Infineon/mtb-example-psoc-edge-empty-app) by following "Create a new application" section in [AN235935](https://www.infineon.com/AN235935) – Getting started with PSOC&trade; Edge E84 MCU on ModusToolbox&trade; application note

2. Add the *camera-dvp-ov7675* library to the application using Library Manager

3. Add 'COMPONENTS+=GFXSS' to the application common.mk file

4. Place the following code in the main.c file of the CM55 application of your project (proj_cm55) to read image frame

    ```cpp
    #include "cybsp.h"
    #include "cy_pdl.h"
    #include "mtb_dvp_camera_ov7675.h"
    #include "vg_lite.h"

    /* Image buffer - driver needs a double buffer buffer */
    vg_lite_buffer_t image_buffer[2];

    cy_stc_scb_i2c_context_t i2c_master_context;
    bool frame_ready = false;
    bool active_frame = false;

    int main(void)
    {
        cy_rslt_t result;

        /* Initialize the device and board peripherals */
        result = cybsp_init();

        /* Board initialization failed. Stop program execution. */
        if (CY_RSLT_SUCCESS != result)
        {
            CY_ASSERT(0);
        }

        /* Enable global interrupts */
        __enable_irq();

        /* Enable I2C Controller */
        result = Cy_SCB_I2C_Init(CYBSP_I2C_CAM_CONTROLLER_HW, &CYBSP_I2C_CAM_CONTROLLER_config,
                                 &i2c_master_context);
        if (CY_SCB_I2C_SUCCESS != result)
        {
            CY_ASSERT(0);
        }

        Cy_SCB_I2C_Enable(CYBSP_I2C_CAM_CONTROLLER_HW);

        /* Initialize the camera DVP OV7675 */
        result = mtb_dvp_cam_ov7675_init(image_buffer, &i2c_master_context,
                                         &frame_ready, &active_frame);
        if (CY_RSLT_SUCCESS != result)
        {
            CY_ASSERT(0);
        }

        for (;;)
        {
            /* Use the image buffer to show the picture on the display
            *  when the frame_ready is true. Take ready frame - image_buffer[active_frame]
            */
        }
    }
    ```

5. Build and program the application.

> **Note:** This driver configures the camera with QVGA (320*240) resolution by default.

## Steps to configure the camera with VGA (640*480) resolution

> **Note:** Ensure that you take a backup of the working driver files with QVGA (320*240) resolution before following the below steps.

1. Change macros `CAMERA_WIDTH` to `640` and `CAMERA_HEIGHT` to `480` in [mtb_dvp_camera_ov7675.h](./mtb_dvp_camera_ov7675.h) file.

2. Change the value of 'clkrc' register from `0x04` to `0x09`.

Replace
``` cpp
const ov7675_frame_rate_config_t OV7675_05FPS_24MHZ_XCLK = { 0x04, 0x4a };
```
with
``` cpp
const ov7675_frame_rate_config_t OV7675_05FPS_24MHZ_XCLK = { 0x09, 0x4a };
```
in [mtb_dvp_camera_ov7675.c](./mtb_dvp_camera_ov7675.c) file.

3. Match your DMA and AXIDMAC settings as shown below:


*Figure 1. Datawire DMA configuration for 640X480 resolution*


![](images/DMA_configuration.png)


*Figure 2. AXIDMAC configuration for 640X480 resolution*


![](images/AXIDMAC_configuration.png)


## More information

For more information, see the following documents:

* [API reference guide](./api_reference.md)
* [ModusToolbox&trade; software environment, quick start guide, documentation, and videos](https://www.infineon.com/modustoolbox)
* [AN239191](https://www.infineon.com/AN239191) – Getting started with graphics on PSOC&trade; Edge MCU
* [Infineon Technologies AG](https://www.infineon.com)


---
© 2025, Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.
