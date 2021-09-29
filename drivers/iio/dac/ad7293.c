// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD7293 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#define AD7293_READ				(1 << 7)
#define AD7293_WRITE				(0 << 7)
#define AD7293_ADDR(x)				((x) & 0xFF)

#define AD7293_R1B				(1 << 16)
#define AD7293_R2B				(2 << 16)
#define AD7293_TRANSF_LEN(x)			((x) >> 16)

#define AD7293_PAGE(x)				((x) << 8)
#define AD7293_PAGE_ADDR(x)			(((x) >> 8) & 0xFF)
#define AD7293_REG_ADDR(x)			((x) & 0xFF)

#define AD7293_CHIP_ID				0x18

/* AD7293 Register Map Common */
#define AD7293_REG_NO_OP			(AD7293_R1B | AD7293_PAGE(0x00) | 0x00)
#define AD7293_REG_PAGE_SELECT			(AD7293_R1B | AD7293_PAGE(0x00) | 0x01)
#define AD7293_REG_CONV_CMD			(AD7293_R2B | AD7293_PAGE(0x00) | 0x02)
#define AD7293_REG_RESULT			(AD7293_R1B | AD7293_PAGE(0x00) | 0x03)
#define AD7293_REG_DAC_EN			(AD7293_R1B | AD7293_PAGE(0x00) | 0x04)
#define AD7293_REG_DEVICE_ID			(AD7293_R2B | AD7293_PAGE(0x00) | 0x0C)
#define AD7293_REG_SOFT_RESET			(AD7293_R2B | AD7293_PAGE(0x00) | 0x0F)

/* AD7293 Register Map Page 0x00 */
#define AD7293_REG_VIN0				(AD7293_R2B | AD7293_PAGE(0x00) | 0x10)
#define AD7293_REG_VIN1				(AD7293_R2B | AD7293_PAGE(0x00) | 0x11)
#define AD7293_REG_VIN2				(AD7293_R2B | AD7293_PAGE(0x00) | 0x12)
#define AD7293_REG_VIN3				(AD7293_R2B | AD7293_PAGE(0x00) | 0x13)
#define AD7293_REG_TSENSE_INT			(AD7293_R2B | AD7293_PAGE(0x00) | 0x20)
#define AD7293_REG_TSENSE_D0			(AD7293_R2B | AD7293_PAGE(0x00) | 0x21)
#define AD7293_REG_TSENSE_D1			(AD7293_R2B | AD7293_PAGE(0x00) | 0x22)
#define AD7293_REG_ISENSE_0			(AD7293_R2B | AD7293_PAGE(0x00) | 0x28)
#define AD7293_REG_ISENSE_1			(AD7293_R2B | AD7293_PAGE(0x00) | 0x29)
#define AD7293_REG_ISENSE_2			(AD7293_R2B | AD7293_PAGE(0x00) | 0x2A)
#define AD7293_REG_ISENSE_3			(AD7293_R2B | AD7293_PAGE(0x00) | 0x2B)
#define AD7293_REG_UNI_VOUT0			(AD7293_R2B | AD7293_PAGE(0x00) | 0x30)
#define AD7293_REG_UNI_VOUT1			(AD7293_R2B | AD7293_PAGE(0x00) | 0x31)
#define AD7293_REG_UNI_VOUT2			(AD7293_R2B | AD7293_PAGE(0x00) | 0x32)
#define AD7293_REG_UNI_VOUT3			(AD7293_R2B | AD7293_PAGE(0x00) | 0x33)
#define AD7293_REG_BI_VOUT0			(AD7293_R2B | AD7293_PAGE(0x00) | 0x34)
#define AD7293_REG_BI_VOUT1			(AD7293_R2B | AD7293_PAGE(0x00) | 0x35)
#define AD7293_REG_BI_VOUT2			(AD7293_R2B | AD7293_PAGE(0x00) | 0x36)
#define AD7293_REG_BI_VOUT3			(AD7293_R2B | AD7293_PAGE(0x00) | 0x37)

/* AD7293 Register Map Page 0x01 */
#define AD7293_REG_AVDD				(AD7293_R2B | AD7293_PAGE(0x01) | 0x10)
#define AD7293_REG_DACVDD_UNI			(AD7293_R2B | AD7293_PAGE(0x01) | 0x11)
#define AD7293_REG_DACVDD_BI			(AD7293_R2B | AD7293_PAGE(0x01) | 0x12)
#define AD7293_REG_AVSS				(AD7293_R2B | AD7293_PAGE(0x01) | 0x13)
#define AD7293_REG_BI_VOUT0_MON			(AD7293_R2B | AD7293_PAGE(0x01) | 0x14)
#define AD7293_REG_BI_VIOU1_MON			(AD7293_R2B | AD7293_PAGE(0x01) | 0x15)
#define AD7293_REG_BI_VOUT2_MON			(AD7293_R2B | AD7293_PAGE(0x01) | 0x16)
#define AD7293_REG_BI_VOUT3_MON			(AD7293_R2B | AD7293_PAGE(0x01) | 0x17)
#define AD7293_REG_RS0_MON			(AD7293_R2B | AD7293_PAGE(0x01) | 0x28)
#define AD7293_REG_RS1_MON			(AD7293_R2B | AD7293_PAGE(0x01) | 0x29)
#define AD7293_REG_RS2_MON			(AD7293_R2B | AD7293_PAGE(0x01) | 0x2A)
#define AD7293_REG_RS3_MON			(AD7293_R2B | AD7293_PAGE(0x01) | 0x2B)

/* AD7293 Register Map Page 0x02 */
#define AD7293_REG_DIGITAL_OUT_EN		(AD7293_R2B | AD7293_PAGE(0x02) | 0x11)
#define AD7293_REG_DIGITAL_INOUT_FUNC		(AD7293_R2B | AD7293_PAGE(0x02) | 0x12)
#define AD7293_REG_DIGITAL_FUNC_POL		(AD7293_R2B | AD7293_PAGE(0x02) | 0x13)
#define AD7293_REG_GENERAL			(AD7293_R2B | AD7293_PAGE(0x02) | 0x14)
#define AD7293_REG_VINX_RANGE0			(AD7293_R2B | AD7293_PAGE(0x02) | 0x15)
#define AD7293_REG_VINX_RANGE1			(AD7293_R2B | AD7293_PAGE(0x02) | 0x16)
#define AD7293_REG_VINX_DIFF_SE			(AD7293_R2B | AD7293_PAGE(0x02) | 0x17)
#define AD7293_REG_VINX_FILTER			(AD7293_R2B | AD7293_PAGE(0x02) | 0x18)
#define AD7293_REG_BG_EN			(AD7293_R2B | AD7293_PAGE(0x02) | 0x19)
#define AD7293_REG_CONV_DELAY			(AD7293_R2B | AD7293_PAGE(0x02) | 0x1A)
#define AD7293_REG_TSENSE_BG_EN			(AD7293_R2B | AD7293_PAGE(0x02) | 0x1B)
#define AD7293_REG_ISENSE_BG_EN			(AD7293_R2B | AD7293_PAGE(0x02) | 0x1C)
#define AD7293_REG_ISENSE_GAIN			(AD7293_R2B | AD7293_PAGE(0x02) | 0x1D)
#define AD7293_REG_DAC_SNOOZE_O			(AD7293_R2B | AD7293_PAGE(0x02) | 0x1F)
#define AD7293_REG_DAC_SNOOZE_1			(AD7293_R2B | AD7293_PAGE(0x02) | 0x20)
#define AD7293_REG_RSX_MON_BG_EN		(AD7293_R2B | AD7293_PAGE(0x02) | 0x23)
#define AD7293_REG_INTEGR_CL			(AD7293_R2B | AD7293_PAGE(0x02) | 0x28)
#define AD7293_REG_PA_ON_CTRL			(AD7293_R2B | AD7293_PAGE(0x02) | 0x29)
#define AD7293_REG_RAMP_TIME_0			(AD7293_R2B | AD7293_PAGE(0x02) | 0x2A)
#define AD7293_REG_RAMP_TIME_1			(AD7293_R2B | AD7293_PAGE(0x02) | 0x2B)
#define AD7293_REG_RAMP_TIME_2			(AD7293_R2B | AD7293_PAGE(0x02) | 0x2C)
#define AD7293_REG_RAMP_TIME_3			(AD7293_R2B | AD7293_PAGE(0x02) | 0x2D)
#define AD7293_REG_CL_FR_IT			(AD7293_R2B | AD7293_PAGE(0x02) | 0x2E)
#define AD7293_REG_INTX_AVSS_AVDD		(AD7293_R2B | AD7293_PAGE(0x02) | 0x2F)

/* AD7293 Register Map Page 0x03 */
#define AD7293_REG_VINX_SEQ			(AD7293_R2B | AD7293_PAGE(0x03) | 0x10)
#define AD7293_REG_ISENSEX_TSENSEX_SEQ		(AD7293_R2B | AD7293_PAGE(0x03) | 0x11)
#define AD7293_REG_RSX_MON_BI_VOUTX_SEQ		(AD7293_R2B | AD7293_PAGE(0x03) | 0x12)

/* AD7293 Register Map Page 0x04 */
#define AD7293_REG_VIN0_HL			(AD7293_R2B | AD7293_PAGE(0x04) | 0x10)
#define AD7293_REG_VIN1_HL			(AD7293_R2B | AD7293_PAGE(0x04) | 0x11)
#define AD7293_REG_VIN2_HL			(AD7293_R2B | AD7293_PAGE(0x04) | 0x12)
#define AD7293_REG_VIN3_HL			(AD7293_R2B | AD7293_PAGE(0x04) | 0x13)
#define AD7293_REG_TSENSE_INT_HL		(AD7293_R2B | AD7293_PAGE(0x04) | 0x20)
#define AD7293_REG_TSENSE_D0_HL			(AD7293_R2B | AD7293_PAGE(0x04) | 0x21)
#define AD7293_REG_TSENSE_D1_HL			(AD7293_R2B | AD7293_PAGE(0x04) | 0x22)
#define AD7293_REG_ISENSE0_HL			(AD7293_R2B | AD7293_PAGE(0x04) | 0x28)
#define AD7293_REG_ISENSE1_HL			(AD7293_R2B | AD7293_PAGE(0x04) | 0x29)
#define AD7293_REG_ISENSE2_HL			(AD7293_R2B | AD7293_PAGE(0x04) | 0x2A)
#define AD7293_REG_ISENSE3_HL			(AD7293_R2B | AD7293_PAGE(0x04) | 0x2B)

/* AD7293 Register Map Page 0x05 */
#define AD7293_REG_AVDD_HL			(AD7293_R2B | AD7293_PAGE(0x05) | 0x10)
#define AD7293_REG_DACVDD_UNI_HL		(AD7293_R2B | AD7293_PAGE(0x05) | 0x11)
#define AD7293_REG_DACVDD_BI_HL			(AD7293_R2B | AD7293_PAGE(0x05) | 0x12)
#define AD7293_REG_AVSS_HL			(AD7293_R2B | AD7293_PAGE(0x05) | 0x13)
#define AD7293_REG_BI_VOUT0_MON_HL		(AD7293_R2B | AD7293_PAGE(0x05) | 0x14)
#define AD7293_REG_BI_VOUT1_MON_HL		(AD7293_R2B | AD7293_PAGE(0x05) | 0x15)
#define AD7293_REG_BI_VOUT2_MON_HL		(AD7293_R2B | AD7293_PAGE(0x05) | 0x16)
#define AD7293_REG_BI_VOUT3_MON_HL		(AD7293_R2B | AD7293_PAGE(0x05) | 0x17)
#define AD7293_REG_RS0_MON_HL			(AD7293_R2B | AD7293_PAGE(0x05) | 0x28)
#define AD7293_REG_RS1_MON_HL			(AD7293_R2B | AD7293_PAGE(0x05) | 0x29)
#define AD7293_REG_RS2_MON_HL			(AD7293_R2B | AD7293_PAGE(0x05) | 0x2A)
#define AD7293_REG_RS3_MON_HL			(AD7293_R2B | AD7293_PAGE(0x05) | 0x2B)

/* AD7293 Register Map Page 0x06 */
#define AD7293_REG_VIN0_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x10)
#define AD7293_REG_VIN1_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x11)
#define AD7293_REG_VIN2_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x12)
#define AD7293_REG_VIN3_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x13)
#define AD7293_REG_TSENSE_D0_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x20)
#define AD7293_REG_TSENSE_D1_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x21)
#define AD7293_REG_TSENSE_D2_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x22)
#define AD7293_REG_ISENSE0_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x28)
#define AD7293_REG_ISENSE1_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x29)
#define AD7293_REG_ISENSE2_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x2A)
#define AD7293_REG_ISENSE3_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x2B)

/* AD7293 Register Map Page 0x07 */
#define AD7293_REG_AVDD_LL			(AD7293_R2B | AD7293_PAGE(0x07) | 0x10)
#define AD7293_REG_DACVDD_UNI_LL		(AD7293_R2B | AD7293_PAGE(0x07) | 0x11)
#define AD7293_REG_DACVDD_BI_LL			(AD7293_R2B | AD7293_PAGE(0x07) | 0x12)
#define AD7293_REG_AVSS_LL			(AD7293_R2B | AD7293_PAGE(0x07) | 0x13)
#define AD7293_REG_BI_VOUT0_MON_LL		(AD7293_R2B | AD7293_PAGE(0x07) | 0x14)
#define AD7293_REG_BI_VOUT1_MON_LL		(AD7293_R2B | AD7293_PAGE(0x07) | 0x15)
#define AD7293_REG_BI_VOUT2_MON_LL		(AD7293_R2B | AD7293_PAGE(0x07) | 0x16)
#define AD7293_REG_BI_VOUT3_MON_LL		(AD7293_R2B | AD7293_PAGE(0x07) | 0x17)
#define AD7293_REG_RS0_MON_LL			(AD7293_R2B | AD7293_PAGE(0x07) | 0x28)
#define AD7293_REG_RS1_MON_LL			(AD7293_R2B | AD7293_PAGE(0x07) | 0x29)
#define AD7293_REG_RS2_MON_LL			(AD7293_R2B | AD7293_PAGE(0x07) | 0x2A)
#define AD7293_REG_RS3_MON_LL			(AD7293_R2B | AD7293_PAGE(0x07) | 0x2B)

/* AD7293 Register Map Page 0x08 */
#define AD7293_REG_VIN0_HYS			(AD7293_R2B | AD7293_PAGE(0x08) | 0x10)
#define AD7293_REG_VIN1_HYS			(AD7293_R2B | AD7293_PAGE(0x08) | 0x11)
#define AD7293_REG_VIN2_HYS			(AD7293_R2B | AD7293_PAGE(0x08) | 0x12)
#define AD7293_REG_VIN3_HYS			(AD7293_R2B | AD7293_PAGE(0x08) | 0x13)
#define AD7293_REG_TSENSE_INT_HYS		(AD7293_R2B | AD7293_PAGE(0x08) | 0x20)
#define AD7293_REG_TSENSE_D0_HYS		(AD7293_R2B | AD7293_PAGE(0x08) | 0x21)
#define AD7293_REG_TSENSE_D1_HYS		(AD7293_R2B | AD7293_PAGE(0x08) | 0x22)
#define AD7293_REG_ISENSE0_HYS			(AD7293_R2B | AD7293_PAGE(0x08) | 0x28)
#define AD7293_REG_ISENSE1_HYS			(AD7293_R2B | AD7293_PAGE(0x08) | 0x29)
#define AD7293_REG_ISENSE2_HYS			(AD7293_R2B | AD7293_PAGE(0x08) | 0x2A)
#define AD7293_REG_ISENSE3_HYS			(AD7293_R2B | AD7293_PAGE(0x08) | 0x2B)

/* AD7293 Register Map Page 0x09 */
#define AD7293_REG_AVDD_HYS			(AD7293_R2B | AD7293_PAGE(0x09) | 0x10)
#define AD7293_REG_DACVDD_UNI_HYS		(AD7293_R2B | AD7293_PAGE(0x09) | 0x11)
#define AD7293_REG_DACVDD_BI_HYS		(AD7293_R2B | AD7293_PAGE(0x09) | 0x12)
#define AD7293_REG_AVSS_HYS			(AD7293_R2B | AD7293_PAGE(0x09) | 0x13)
#define AD7293_REG_BI_VOUT0_MON_HYS		(AD7293_R2B | AD7293_PAGE(0x09) | 0x14)
#define AD7293_REG_BI_VOUT1_MON_HYS		(AD7293_R2B | AD7293_PAGE(0x09) | 0x15)
#define AD7293_REG_BI_VOUT2_MON_HYS		(AD7293_R2B | AD7293_PAGE(0x09) | 0x16)
#define AD7293_REG_BI_VOUT3_MON_HYS		(AD7293_R2B | AD7293_PAGE(0x09) | 0x17)
#define AD7293_REG_RS0_MON_HYS			(AD7293_R2B | AD7293_PAGE(0x09) | 0x28)
#define AD7293_REG_RS1_MON_HYS			(AD7293_R2B | AD7293_PAGE(0x09) | 0x29)
#define AD7293_REG_RS2_MON_HYS			(AD7293_R2B | AD7293_PAGE(0x09) | 0x2A)
#define AD7293_REG_RS3_MON_HYS			(AD7293_R2B | AD7293_PAGE(0x09) | 0x2B)

/* AD7293 Register Map Page 0x0A */
#define AD7293_REG_VIN0_MIN			(AD7293_R2B | AD7293_PAGE(0x0A) | 0x10)
#define AD7293_REG_VIN1_MIN			(AD7293_R2B | AD7293_PAGE(0x0A) | 0x11)
#define AD7293_REG_VIN2_MIN			(AD7293_R2B | AD7293_PAGE(0x0A) | 0x12)
#define AD7293_REG_VIN3_MIN			(AD7293_R2B | AD7293_PAGE(0x0A) | 0x13)
#define AD7293_REG_TSENSE_INT_MIN		(AD7293_R2B | AD7293_PAGE(0x0A) | 0x20)
#define AD7293_REG_TSENSE_D0_MIN		(AD7293_R2B | AD7293_PAGE(0x0A) | 0x21)
#define AD7293_REG_TSENSE_D1_MIN		(AD7293_R2B | AD7293_PAGE(0x0A) | 0x22)
#define AD7293_REG_ISENSE0_MIN			(AD7293_R2B | AD7293_PAGE(0x0A) | 0x28)
#define AD7293_REG_ISENSE1_MIN			(AD7293_R2B | AD7293_PAGE(0x0A) | 0x29)
#define AD7293_REG_ISENSE2_MIN			(AD7293_R2B | AD7293_PAGE(0x0A) | 0x2A)
#define AD7293_REG_ISENSE3_MIN			(AD7293_R2B | AD7293_PAGE(0x0A) | 0x2B)

/* AD7293 Register Map Page 0x0B */
#define AD7293_REG_AVDD_MIN			(AD7293_R2B | AD7293_PAGE(0x0B) | 0x10)
#define AD7293_REG_DACVDD_UNI_MIN		(AD7293_R2B | AD7293_PAGE(0x0B) | 0x11)
#define AD7293_REG_DACVDD_BI_MIN		(AD7293_R2B | AD7293_PAGE(0x0B) | 0x12)
#define AD7293_REG_AVSS_MIN			(AD7293_R2B | AD7293_PAGE(0x0B) | 0x13)
#define AD7293_REG_BI_VOUT0_MON_MIN		(AD7293_R2B | AD7293_PAGE(0x0B) | 0x14)
#define AD7293_REG_BI_VOUT1_MON_MIN		(AD7293_R2B | AD7293_PAGE(0x0B) | 0x15)
#define AD7293_REG_BI_VOUT2_MON_MIN		(AD7293_R2B | AD7293_PAGE(0x0B) | 0x16)
#define AD7293_REG_BI_VOUT3_MON_MIN		(AD7293_R2B | AD7293_PAGE(0x0B) | 0x17)
#define AD7293_REG_RS0_MON_MIN			(AD7293_R2B | AD7293_PAGE(0x0B) | 0x28)
#define AD7293_REG_RS1_MON_MIN			(AD7293_R2B | AD7293_PAGE(0x0B) | 0x29)
#define AD7293_REG_RS2_MON_MIN			(AD7293_R2B | AD7293_PAGE(0x0B) | 0x2A)
#define AD7293_REG_RS3_MON_MIN			(AD7293_R2B | AD7293_PAGE(0x0B) | 0x2B)

/* AD7293 Register Map Page 0x0C */
#define AD7293_REG_VIN0_MAX			(AD7293_R2B | AD7293_PAGE(0x0C) | 0x10)
#define AD7293_REG_VIN1_MAX			(AD7293_R2B | AD7293_PAGE(0x0C) | 0x11)
#define AD7293_REG_VIN2_MAX			(AD7293_R2B | AD7293_PAGE(0x0C) | 0x12)
#define AD7293_REG_VIN3_MAX			(AD7293_R2B | AD7293_PAGE(0x0C) | 0x13)
#define AD7293_REG_TSENSE_INT_MAX		(AD7293_R2B | AD7293_PAGE(0x0C) | 0x20)
#define AD7293_REG_TSENSE_D0_MAX		(AD7293_R2B | AD7293_PAGE(0x0C) | 0x21)
#define AD7293_REG_TSENSE_D1_MAX		(AD7293_R2B | AD7293_PAGE(0x0C) | 0x22)
#define AD7293_REG_ISENSE0_MAX			(AD7293_R2B | AD7293_PAGE(0x0C) | 0x28)
#define AD7293_REG_ISENSE1_MAX			(AD7293_R2B | AD7293_PAGE(0x0C) | 0x29)
#define AD7293_REG_ISENSE2_MAX			(AD7293_R2B | AD7293_PAGE(0x0C) | 0x2A)
#define AD7293_REG_ISENSE3_MAX			(AD7293_R2B | AD7293_PAGE(0x0C) | 0x2B)

/* AD7293 Register Map Page 0x0D */
#define AD7293_REG_AVDD_MAX			(AD7293_R2B | AD7293_PAGE(0x0D) | 0x10)
#define AD7293_REG_DACVDD_UNI_MAX		(AD7293_R2B | AD7293_PAGE(0x0D) | 0x11)
#define AD7293_REG_DACVDD_BI_MAX		(AD7293_R2B | AD7293_PAGE(0x0D) | 0x12)
#define AD7293_REG_AVSS_MAX			(AD7293_R2B | AD7293_PAGE(0x0D) | 0x13)
#define AD7293_REG_BI_VOUT0_MON_MAX		(AD7293_R2B | AD7293_PAGE(0x0D) | 0x14)
#define AD7293_REG_BI_VOUT1_MON_MAX		(AD7293_R2B | AD7293_PAGE(0x0D) | 0x15)
#define AD7293_REG_BI_VOUT2_MON_MAX		(AD7293_R2B | AD7293_PAGE(0x0D) | 0x16)
#define AD7293_REG_BI_VOUT3_MON_MAX		(AD7293_R2B | AD7293_PAGE(0x0D) | 0x17)
#define AD7293_REG_RS0_MON_MAX			(AD7293_R2B | AD7293_PAGE(0x0D) | 0x28)
#define AD7293_REG_RS1_MON_MAX			(AD7293_R2B | AD7293_PAGE(0x0D) | 0x29)
#define AD7293_REG_RS2_MON_MAX			(AD7293_R2B | AD7293_PAGE(0x0D) | 0x2A)
#define AD7293_REG_RS3_MON_MAX			(AD7293_R2B | AD7293_PAGE(0x0D) | 0x2B)

/* AD7293 Register Map Page 0x0E */
#define AD7293_REG_VIN0_OFFSET			(AD7293_R1B | AD7293_PAGE(0x0E) | 0x10)
#define AD7293_REG_VIN1_OFFSET			(AD7293_R1B | AD7293_PAGE(0x0E) | 0x11)
#define AD7293_REG_VIN2_OFFSET			(AD7293_R1B | AD7293_PAGE(0x0E) | 0x12)
#define AD7293_REG_VIN3_OFFSET			(AD7293_R1B | AD7293_PAGE(0x0E) | 0x13)
#define AD7293_REG_TSENSE_INT_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0E) | 0x20)
#define AD7293_REG_TSENSE_D0_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0E) | 0x21)
#define AD7293_REG_TSENSE_D1_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0E) | 0x22)
#define AD7293_REG_ISENSE0_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0E) | 0x28)
#define AD7293_REG_ISENSE1_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0E) | 0x29)
#define AD7293_REG_ISENSE2_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0E) | 0x2A)
#define AD7293_REG_ISENSE3_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0E) | 0x2B)
#define AD7293_REG_UNI_VOUT0_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0E) | 0x30)
#define AD7293_REG_UNI_VOUT1_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0E) | 0x31)
#define AD7293_REG_UNI_VOUT2_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0E) | 0x32)
#define AD7293_REG_UNI_VOUT3_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0E) | 0x33)
#define AD7293_REG_BI_VOUT0_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0E) | 0x34)
#define AD7293_REG_BI_VOUT1_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0E) | 0x35)
#define AD7293_REG_BI_VOUT2_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0E) | 0x36)
#define AD7293_REG_BI_VOUT3_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0E) | 0x37)

/* AD7293 Register Map Page 0x0F */
#define AD7293_REG_AVDD_OFFSET			(AD7293_R1B | AD7293_PAGE(0x0F) | 0x10)
#define AD7293_REG_DACVDD_UNI_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x11)
#define AD7293_REG_DACVDD_BI_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x12)
#define AD7293_REG_AVSS_OFFSET			(AD7293_R1B | AD7293_PAGE(0x0F) | 0x13)
#define AD7293_REG_BI_VOUT0_MON_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x14)
#define AD7293_REG_BI_VOUT1_MON_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x15)
#define AD7293_REG_BI_VOUT2_MON_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x16)
#define AD7293_REG_BI_VOUT3_MON_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x17)
#define AD7293_REG_RS0_MON_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x28)
#define AD7293_REG_RS1_MON_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x29)
#define AD7293_REG_RS2_MON_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x2A)
#define AD7293_REG_RS3_MON_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x2B)

/* AD7293 Register Map Page 0x10 */
#define AD7293_REG_ALERT_SUM			(AD7293_R2B | AD7293_PAGE(0x10) | 0x10)
#define AD7293_REG_VINX_ALERT			(AD7293_R2B | AD7293_PAGE(0x10) | 0x12)
#define AD7293_REG_TSENSEX_ALERT		(AD7293_R2B | AD7293_PAGE(0x10) | 0x14)
#define AD7293_REG_ISENSEX_ALERT		(AD7293_R2B | AD7293_PAGE(0x10) | 0x15)
#define AD7293_REG_BI_VOUTX_MON_ALERT		(AD7293_R2B | AD7293_PAGE(0x10) | 0x18)
#define AD7293_REG_RSX_MON_ALERT		(AD7293_R2B | AD7293_PAGE(0x10) | 0x19)
#define AD7293_REG_INT_LIMIT_AVSS_ALERT		(AD7293_R2B | AD7293_PAGE(0x10) | 0x1A)

/* AD7293 Register Map Page 0x11 */
#define AD7293_REG_VINX_ALERT0			(AD7293_R2B | AD7293_PAGE(0x11) | 0x12)
#define AD7293_REG_TSENSEX_ALERT0		(AD7293_R2B | AD7293_PAGE(0x11) | 0x14)
#define AD7293_REG_ISENSEX_ALERT0		(AD7293_R2B | AD7293_PAGE(0x11) | 0x15)
#define AD7293_REG_BI_VOUTX_MON_ALERT0		(AD7293_R2B | AD7293_PAGE(0x11) | 0x18)
#define AD7293_REG_RSX_MON_ALERT0		(AD7293_R2B | AD7293_PAGE(0x11) | 0x19)
#define AD7293_REG_INT_LIMIT_AVSS_ALERT0	(AD7293_R2B | AD7293_PAGE(0x11) | 0x1A)

/* AD7293 Register Map Page 0x12 */
#define AD7293_REG_VINX_ALERT1			(AD7293_R2B | AD7293_PAGE(0x12) | 0x12)
#define AD7293_REG_TSENSEX_ALERT1		(AD7293_R2B | AD7293_PAGE(0x12) | 0x14)
#define AD7293_REG_ISENSEX_ALERT1		(AD7293_R2B | AD7293_PAGE(0x12) | 0x15)
#define AD7293_REG_BI_VOUTX_MON_ALERT1		(AD7293_R2B | AD7293_PAGE(0x12) | 0x18)
#define AD7293_REG_RSX_MON_ALERT1		(AD7293_R2B | AD7293_PAGE(0x12) | 0x19)
#define AD7293_REG_INT_LIMIT_AVSS_ALERT1	(AD7293_R2B | AD7293_PAGE(0x12) | 0x1A)

struct ad7293_dev {
	struct spi_device	*spi;
	u8 page_select;
	u8 data[3] ____cacheline_aligned;
};

static int ad7293_page_select(struct ad7293_dev *dev, u8 page_nr)
{
	dev->data[0] = AD7293_REG_ADDR(AD7293_REG_PAGE_SELECT);
	dev->data[1] = page_nr;

	return spi_write(dev->spi, &dev->data[0], 2);
}

static int ad7293_spi_read(struct ad7293_dev *dev, unsigned int reg,
			      unsigned int *val)
{
	int ret;
	struct spi_transfer t = {0};

	if (dev->page_select != AD7293_PAGE_ADDR(reg)) {
		ret = ad7293_page_select(dev, AD7293_PAGE_ADDR(reg));
		if (ret)
			return ret;

		dev->page_select = AD7293_PAGE_ADDR(reg);
	}

	dev->data[0] = AD7293_READ | AD7293_REG_ADDR(reg);
	dev->data[1] = 0x0;
	dev->data[2] = 0x0;

	t.tx_buf = &dev->data[0];
	t.rx_buf = &dev->data[0];
	t.len = 1 + AD7293_TRANSF_LEN(reg);

	ret = spi_sync_transfer(dev->spi, &t, 1);
	if (ret)
		return ret;

	*val = ((dev->data[1] << 8) | dev->data[2]) >> (8 * (2 - AD7293_TRANSF_LEN(reg)));

	return ret;
}

static int ad7293_spi_write(struct ad7293_dev *dev, unsigned int reg,
			      unsigned int val)
{
	int ret;

	if (dev->page_select != AD7293_PAGE_ADDR(reg)) {
		ret = ad7293_page_select(dev, AD7293_PAGE_ADDR(reg));
		if (ret)
			return ret;

		dev->page_select = AD7293_PAGE_ADDR(reg);
	}

	dev->data[0] = AD7293_WRITE | AD7293_REG_ADDR(reg);

	if (AD7293_TRANSF_LEN(reg) == 1) {
		dev->data[1] = val;
	} else {
		dev->data[1] = val >> 8;
		dev->data[2] = val;
	}

	return spi_write(dev->spi, &dev->data[0], 1 + AD7293_TRANSF_LEN(reg));
}

static int ad7293_reg_access(struct iio_dev *indio_dev,
				unsigned int reg,
				unsigned int write_val,
				unsigned int *read_val)
{
	struct ad7293_dev *dev = iio_priv(indio_dev);
	int ret;

	if (read_val)
		ret = ad7293_spi_read(dev, reg, read_val);
	else
		ret = ad7293_spi_write(dev, reg, write_val);

	return ret;
}

#define AD7293_CHAN_ADC(_channel) {				\
	.type = IIO_VOLTAGE,					\
	.output = 0,						\
	.indexed = 1,						\
	.channel = _channel,					\
	.info_mask_separate =					\
		BIT(IIO_CHAN_INFO_RAW) | 			\
		BIT(IIO_CHAN_INFO_SCALE) | 			\
		BIT(IIO_CHAN_INFO_OFFSET), 			\
	.info_mask_shared_by_type_available = 			\
		BIT(IIO_CHAN_INFO_SCALE)			\
}

#define AD7293_CHAN_DAC(_channel) {				\
	.type = IIO_VOLTAGE,					\
	.output = 1,						\
	.indexed = 1,						\
	.channel = _channel,					\
	.info_mask_separate =					\
		BIT(IIO_CHAN_INFO_RAW) | 			\
		BIT(IIO_CHAN_INFO_OFFSET), 			\
	.info_mask_shared_by_type_available =			\
		BIT(IIO_CHAN_INFO_OFFSET),			\
}

#define AD7293_CHAN_ISENSE(_channel) {				\
	.type = IIO_CURRENT,					\
	.output = 0,						\
	.indexed = 1,						\
	.channel = _channel,					\
	.info_mask_separate =					\
		BIT(IIO_CHAN_INFO_RAW) | 			\
		BIT(IIO_CHAN_INFO_OFFSET) |			\
		BIT(IIO_CHAN_INFO_HARDWAREGAIN),		\
	.info_mask_shared_by_type_available =			\
		BIT(IIO_CHAN_INFO_HARDWAREGAIN)			\
}

#define AD7293_CHAN_TEMP(_channel) {				\
	.type = IIO_TEMP,					\
	.output = 0,						\
	.indexed = 1,						\
	.channel = _channel,					\
	.info_mask_separate =					\
		BIT(IIO_CHAN_INFO_RAW) | 			\
		BIT(IIO_CHAN_INFO_OFFSET) 			\
}

static const struct iio_chan_spec ad7293_channels[] = {
	AD7293_CHAN_ADC(0),
	AD7293_CHAN_ADC(1),
	AD7293_CHAN_ADC(2),
	AD7293_CHAN_ADC(3),
	AD7293_CHAN_ISENSE(0),
	AD7293_CHAN_ISENSE(1),
	AD7293_CHAN_ISENSE(2),
	AD7293_CHAN_ISENSE(3),
	AD7293_CHAN_TEMP(0),
	AD7293_CHAN_TEMP(1),
	AD7293_CHAN_TEMP(2),
	AD7293_CHAN_DAC(0),
	AD7293_CHAN_DAC(1),
	AD7293_CHAN_DAC(2),
	AD7293_CHAN_DAC(3),
	AD7293_CHAN_DAC(4),
	AD7293_CHAN_DAC(5),
	AD7293_CHAN_DAC(6),
	AD7293_CHAN_DAC(7)
};

static int ad7293_init(struct ad7293_dev *dev)
{
	int ret;
	unsigned int chip_id;
	struct spi_device *spi = dev->spi;

	ret = ad7293_spi_read(dev, AD7293_REG_DEVICE_ID, &chip_id);
	if (ret < 0)
		return ret;

	if (chip_id != AD7293_CHIP_ID) {
		dev_err(&spi->dev, "Invalid Chip ID.\n");
		return -EINVAL;
	}

	return 0;
}

static const struct iio_info ad7293_info = {
	.debugfs_reg_access = &ad7293_reg_access,
};

static int ad7293_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad7293_dev *dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
	if (!indio_dev)
		return -ENOMEM;

	dev = iio_priv(indio_dev);
	
	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &ad7293_info;
	indio_dev->name = "ad7293";
	indio_dev->channels = ad7293_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad7293_channels);

	dev->spi = spi;
	dev->page_select = 0;

	ret = ad7293_init(dev);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id ad7293_id[] = {
	{ "ad7293", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad7293_id);

static const struct of_device_id ad7293_of_match[] = {
	{ .compatible = "adi,ad7293" },
	{}
};
MODULE_DEVICE_TABLE(of, ad7293_of_match);

static struct spi_driver ad7293_driver = {
	.driver = {
		.name = "ad7293",
		.of_match_table = ad7293_of_match,
	},
	.probe = ad7293_probe,
	.id_table = ad7293_id,
};
module_spi_driver(ad7293_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com");
MODULE_DESCRIPTION("Analog Devices AD7293");
MODULE_LICENSE("GPL v2");
