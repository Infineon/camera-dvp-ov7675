/******************************************************************************
 * \file mtb_dvp_camera_ov7675_def.h
 *
 * \brief
 *     This file contains the address of OV7675 registers.
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


/*******************************************************************************
 * Macros
 ******************************************************************************/

#define OV7675_GAIN_REG                       0x00U         /* Gain lower 8 bits (rest in vref). */

#define OV7675_BLUE_REG                       0x01U         /* Blue gain. */

#define OV7675_RED_REG                        0x02U         /* Red gain. */

#define OV7675_VREF_REG                       0x03U         /* Pieces of GAIN, VSTART, VSTOP. */

#define OV7675_COM1_REG                       0x04U         /* Control 1. */

#define OV7675_BAVE_REG                       0x05U         /* U/B Average level. */

#define OV7675_BGAVE_REG                      0x06U         /* Y/Gb Average level. */

#define OV7675_AECHH_REG                      0x07U         /* AEC MS 5 bits. */

#define OV7675_RAVE_REG                       0x08U         /* V/R Average level. */

#define OV7675_COM2_REG                       0x09U         /* Control 2. */

#define OV7675_PID_REG                        0x0AU         /* Product ID MSB register address. */
#define OV7675_PID_NUM                        0x76U         /* Product ID. */

#define OV7675_VER_REG                        0x0BU         /* Product ID LSB register address. */
#define OV7675_VER_NUM                        0x73U         /* Product VERION. */

#define OV7675_COM3_REG                       0x0CU         /* Control 3. */
#define OV7675_COM3_DCW_BITS                  0x0CU         /* Output format, bit 3 and 2 */

#define OV7675_COM4_REG                       0x0DU         /* Control 4. */

#define OV7675_DEBUG2_REG                     0x0EU         /* Debug mode. */

#define OV7675_COM6_REG                       0x0FU         /* Control 6. */

#define OV7675_AECH_REG                       0x10U         /* More bits of AEC value. */

#define OV7675_CLKRC_REG                      0x11U         /* Clock control. */

#define OV7675_COM7_REG                       0x12U        /* Control 7. */
#define OV7675_COM7_RESET_MASK                0x80U        /* SCCB Register reset. */
#define OV7675_COM7_FMT_QVGA_MASK             0x10U        /* QVGA format. */
#define OV7675_COM7_OUT_FMT_BITS              0x05U        /* Output format, bit 2 and 0 */

#define OV7675_COM8_REG                       0x13U        /* Control 8. */

#define OV7675_COM9_REG                       0x14U        /* Control 9  - gain ceiling. */

#define OV7675_COM10_REG                      0x15U        /* Control 10. */

#define OV7675_HSTART_REG                     0x17U        /* Horizontal start high bits. */

#define OV7675_HSTOP_REG                      0x18U        /* Horizontal stop high bits. */

#define OV7675_VSTART_REG                     0x19U        /* Vertical start high bits. */

#define OV7675_VSTOP_REG                      0x1AU        /* Vertical stop high bits. */

#define OV7675_PSHFT_REG                      0x1BU        /* Pixel delay after HREF. */

#define OV7675_MIDH_REG                       0x1CU        /* Manuf. ID high. */

#define OV7675_MIDL_REG                       0x1DU        /* Manuf. ID low. */

#define OV7675_MVFP_REG                       0x1EU        /* Mirror / vflip. */
#define OV7675_MVFP_MIRROR_MASK               0x20U        /* Mirror the image */

#define OV7675_AEW_REG                        0x24U        /* AGC upper limit. */

#define OV7675_AEB_REG                        0x25U        /* AGC lower limit. */

#define OV7675_VPT_REG                        0x26U        /* AGC/AEC fast mode op region. */

#define OV7675_EXHCH_REG                      0x2AU        /* dummy pixel insert MSB. */

#define OV7675_EXHCL_REG                      0x2BU        /* dummy pixel insert LSB. */

#define OV7675_ADVFL_REG                      0x2DU        /* LSB of dummy vertical line */

#define OV7675_ADVFH_REG                      0x2EU        /* MSB of dummy vertical line */

#define OV7675_HSYST_REG                      0x30U        /* HSYST rising edge delay. */

#define OV7675_HSYEN_REG                      0x31U        /* HSYNC falling edge delay. */

#define OV7675_HREF_REG                       0x32U        /* HREF pieces. */

#define OV7675_TSLB_REG                       0x3AU        /* lots of stuff. */
#define OV7675_TSLB_AUTO_WIN_MASK             0x01U        /* Automatically set window upon
                                                              resolution change. */

#define OV7675_COM11_REG                      0x3BU        /* Control 11. */

#define OV7675_COM12_REG                      0x3CU       /* Control 12. */

#define OV7675_COM13_REG                      0x3DU       /* Control 13. */

#define OV7675_COM14_REG                      0x3EU       /* Control 14. */

#define OV7675_EDGE_REG                       0x3FU       /* Edge enhancement factor. */

#define OV7675_COM15_REG                      0x40U       /* Control 15. */

#define OV7675_COM16_REG                      0x41U       /* Control 16. */

#define OV7675_COM17_REG                      0x42U       /* Control 17. */

#define OV7675_DNSTH_REG                      0x4CU       /* De-noise strength. */

#define OV7675_MTX1_REG                       0x4FU       /* Matrix Coefficient 1. */

#define OV7675_MTX2_REG                       0x50U       /* Matrix Coefficient 2. */

#define OV7675_MTX3_REG                       0x51U       /* Matrix Coefficient 3. */

#define OV7675_MTX4_REG                       0x52U       /* Matrix Coefficient 4. */

#define OV7675_MTX5_REG                       0x53U       /* Matrix Coefficient 5. */

#define OV7675_MTX6_REG                       0x54U       /* Matrix Coefficient 6. */

#define OV7675_BRIGHT_REG                     0x55U       /* Brightness. */

#define OV7675_CONTRAS_REG                    0x56U       /* Contrast control. */

#define OV7675_CONTRAS_CENTER_REG             0x57U       /* Contrast center control. */
#define OV7675_CONTRAS_CENTER_DEFAULT         0x80U       /* Default value */

#define OV7675_MTXS_REG                       0x58U       /* Matrix Coefficient Sign. */
#define OV7675_MTXS_AUTOCONTA_MASK            0x80U       /* Enable auto contrast */

#define OV7675_LCC1_MASK                      0x62U       /* Lens correction option 1. */

#define OV7675_LCC2_MASK                      0x63U       /* Lens correction option 2. */

#define OV7675_LCC3_MASK                      0x65U       /* Lens correction option 3. */

#define OV7675_LCC4_MASK                      0x65U       /* Lens correction option 4. */

#define OV7675_LCC5_MASK                      0x66U       /* Lens correction option 6. */

#define OV7675_MANU_REG                       0x67U       /* Manual U value. */

#define OV7675_MANV_REG                       0x68U       /* Manual V value. */

#define OV7675_GFIX_REG                       0x69U       /* Fix gain control. */

#define OV7675_GREEN_REG                      0x6AU       /* G Channel AWB Gain. */

#define OV7675_DBLV_REG                       0x6BU       /* PLL control. */

#define OV7675_AWBCTR3_REG                    0x6CU       /* AWB Control 3. */

#define OV7675_AWBCTR2_REG                    0x6DU       /* AWB Control 2. */

#define OV7675_AWBCTR1_REG                    0x6EU       /* AWB Control 1. */

#define OV7675_AWBCTR0_REG                    0x6FU       /* AWB Control 0. */

#define OV7675_SCALING_XSC_REG                0x70U       /* horizontal scale factor. */

#define OV7675_SCALING_YSC_REG                0x71U       /* vertical scale factor. */

#define OV7675_REG74_REG                      0x74U       /* register 74. */

#define OV7675_REG75_REG                      0x75U       /* register 75. */

#define OV7675_REG76_REG                      0x76U       /* OV's name. */

#define OV7675_REG77_REG                      0x77U       /* register 77. */

#define OV7675_SLOP_REG                       0x7AU       /* gamma curve highest segment slop. */

#define OV7675_GAM1_REG                       0x7BU       /* gamma curve 1 segment slop. */

#define OV7675_GAM2_REG                       0x7CU       /* gamma curve 2 segment slop. */

#define OV7675_GAM3_REG                       0x7DU       /* gamma curve 3 segment slop. */

#define OV7675_GAM4_REG                       0x7EU       /* gamma curve 4 segment slop. */

#define OV7675_GAM5_REG                       0x7FU       /* gamma curve 5 segment slop. */

#define OV7675_GAM6_REG                       0x80U       /* gamma curve 6 segment slop. */

#define OV7675_GAM7_REG                       0x81U       /* gamma curve 7 segment slop. */

#define OV7675_GAM8_REG                       0x82U       /* gamma curve 8 segment slop. */

#define OV7675_GAM9_REG                       0x83U       /* gamma curve 9 segment slop. */

#define OV7675_GAM10_REG                      0x84U       /* gamma curve 10 segment slop. */
#define OV7675_COM10_PCLK_HB_MASK             0x20U       /* Suppress PCLK on horizontal blank. */
#define OV7675_COM10_HREF_REV_MASK            0x10U       /* Reverse HREF. */


#define OV7675_GAM11_REG                      0x85U       /* gamma curve 11 segment slop. */

#define OV7675_GAM12_REG                      0x86U       /* gamma curve 12 segment slop. */

#define OV7675_GAM13_REG                      0x87U       /* gamma curve 13 segment slop. */

#define OV7675_GAM14_REG                      0x88U       /* gamma curve 14 segment slop. */

#define OV7675_GAM15_REG                      0x89U       /* gamma curve 15 segment slop. */
#define OV7675_COM15_RGB_FMT_BITS             0x30U       /* RGB format section, bit 5 and 4 */

#define OV7675_RGB444_REG                     0x8CU       /* RGB 444 control. */
#define OV7675_RGB444_SET_BITS                0x01U       /* Set RGB444, bit 2 and 1 */

#define OV7675_DM_LNH_REG                     0x92U       /* dummy line LSB. */

#define OV7675_DM_LCC6_REG                    0x93U       /* dummy line MSB. */

#define OV7675_DM_LCC7_REG                    0x94U       /* Lens correction option 6. */

#define OV7675_BD50ST_REG                     0x9DU       /* 50Hz banding filter value. */

#define OV7675_BD60ST_REG                     0x9EU       /* 60Hz banding filter value. */

#define OV7675_NT_CTRL_REG                    0xA4U       /* Auto frame rate adjustment. */

#define OV7675_BD50MAX_REG                    0xA5U       /* 50Hz banding step limit. */

#define OV7675_BD60MAX_REG                    0xABU       /* 60Hz banding step limit. */

#define OV7675_SATCTR_REG                     0xC9U       /* Saturation control. */

#define OV7675_REGCA_REG                      0xCAU       /* Dummy pixel horizontal. */

#define OV7675_REFCF_REG                      0xCAU       /* Used together with COM11. */


#define BIT_SET                               0x01U
#define BIT_CLEAR                             0x00U


#if defined(__cplusplus)
}
#endif
