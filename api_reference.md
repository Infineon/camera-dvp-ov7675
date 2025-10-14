# OV7675 DVP Camera driver library API reference guide

## General description

Basic set of APIs for interacting with the OV7675 DVP Camera module.

For more information about the OV7675 DVP Camera module, see [OV7675 DVP Camera module](https://www.arducam.com/640x480-0-3-mp-mega-pixel-lens-ov7675-cmos-camera-module-with-adapter-board.html).

This version of the driver sets a fixed configuration for the camera: 24 MHz XCLK, 320x240 resolution, AEC/AGC enabled, default brightness/contrast, RGB565 color format.


## Code snippets

### Snippet 1: OV7675 DVP Camera initialization

The following snippet initializes the OV7675 DVP Camera with a fixed configuration and passes a buffer to store camera data.

```
/* Image buffer - driver needs a double buffer */
vg_lite_buffer_t image_buffer[2];

cy_stc_scb_i2c_context_t i2c_master_context;
bool frame_ready = false;
bool active_frame = false;

/* Initializes the OV7675 DVP camera module */
mtb_dvp_cam_ov7675_init(image_buffer, i2c_master_context, frame_ready, active_frame);

for(;;)
{
    if ( frame_ready == true )
    {
        /* Ensure to clear the flag */
        frame_ready = false;

        /* Use the active/ready image frame */
        vg_lite_blit(&render_target, &image_buffer[active_frame], &image_matrix,
                             VG_LITE_BLEND_NONE, 0, VG_LITE_FILTER_POINT);
    }
}
```

## Functions

void `mtb_dvp_cam_ov7675_init (vg_lite_buffer_t* buffer, cy_stc_scb_i2c_context_t* i2c_instance, bool* _frame_ready_flag, bool* _active_frame_flag)`
- Initializes the OV7675 camera module, copies the image data to the buffer, indicates whether the image frame is ready, and indicates which frame/index is active in the double buffer.

## Function documentation

#### mtb_dvp_cam_ov7675_init

- void mtb_dvp_cam_ov7675_init (vg_lite_buffer_t *buffer, cy_stc_scb_i2c_context_t* i2c_instance, bool* _frame_ready_flag, bool* _active_frame_flag)

  **Summary:** This function initializes the OV7675 DVP camera (with a fixed configuration) and the MCU hardware resources that are required for interfacing the camera. It sets the frame_ready_flag flag that indicates whether the image frame is ready, and the active_frame_flag flag that indicates which frame buffer is active/ready. The "active_frame_flag == true" indicates that the index 1 of the double buffer is active.

  **Table 1: Parameters**

   Parameters             |  Description
   :-------               |  :------------
   [in] buffer            |  Pointer to the image buffer
   [in] i2c_instance      |  Pointer to an initialized I2C object context
   [in] frame_ready_flag  |  Pointer to the flag that indicates whether the image frame is ready
   [in] active_frame_flag |  Pointer to the flag that indicates which frame buffer is active/ready in the double buffer

   <br>

---
© 2025, Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.
