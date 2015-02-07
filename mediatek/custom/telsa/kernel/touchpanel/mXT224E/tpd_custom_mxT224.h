/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
#define TPD_TYPE_RESISTIVE
#define TPD_POWER_SOURCE         
#define TPD_I2C_NUMBER           0
#define TPD_WAKEUP_TRIAL         60
#define TPD_WAKEUP_DELAY         100

#define TPD_VELOCITY_CUSTOM_X 15
#define TPD_VELOCITY_CUSTOM_Y 20


#define TPD_DELAY                (2*HZ/100)
//#define TPD_RES_X                480
//#define TPD_RES_Y                800
#define TPD_CALIBRATION_MATRIX  {962,0,0,0,1600,0,0,0};

//#define TPD_HAVE_CALIBRATION
//#define TPD_HAVE_BUTTON
//#define TPD_HAVE_TREMBLE_ELIMINATION
#define TPD_HAVE_BUTTON
#define TPD_BUTTON_HEIGH        (100)
#define TPD_KEY_COUNT           4
#define TPD_KEYS                { KEY_MENU, KEY_HOMEPAGE ,KEY_BACK, KEY_SEARCH}
#define TPD_KEYS_DIM            {{60,850,120,TPD_BUTTON_HEIGH},{180,850,120,TPD_BUTTON_HEIGH},{300,850,120,TPD_BUTTON_HEIGH}, {420,850,120,TPD_BUTTON_HEIGH}}








#ifndef _LINUX_ATMEL_H
#define _LINUX_ATMEL_H

#define ATMEL_QT602240_NAME "atmel_qt602240"

#define RESERVED_T0                               0u
#define RESERVED_T1                               1u
#define DEBUG_DELTAS_T2                           2u
#define DEBUG_REFERENCES_T3                       3u
#define DEBUG_SIGNALS_T4                          4u
#define GEN_MESSAGEPROCESSOR_T5                   5u
#define GEN_COMMANDPROCESSOR_T6                   6u
#define GEN_POWERCONFIG_T7                        7u
#define GEN_ACQUISITIONCONFIG_T8                  8u
#define TOUCH_MULTITOUCHSCREEN_T9                 9u
#define TOUCH_SINGLETOUCHSCREEN_T10               10u
#define TOUCH_XSLIDER_T11                         11u
#define TOUCH_YSLIDER_T12                         12u
#define TOUCH_XWHEEL_T13                          13u
#define TOUCH_YWHEEL_T14                          14u
#define TOUCH_KEYARRAY_T15                        15u
#define PROCG_SIGNALFILTER_T16                    16u
#define PROCI_LINEARIZATIONTABLE_T17              17u
#define SPT_COMCONFIG_T18                         18u
#define SPT_GPIOPWM_T19                           19u
#define PROCI_GRIPFACESUPPRESSION_T20             20u
#define RESERVED_T21                              21u
#define PROCG_NOISESUPPRESSION_T22                22u
#define TOUCH_PROXIMITY_T23	                  23u
#define PROCI_ONETOUCHGESTUREPROCESSOR_T24        24u
#define SPT_SELFTEST_T25                          25u
#define DEBUG_CTERANGE_T26                        26u
#define PROCI_TWOTOUCHGESTUREPROCESSOR_T27        27u
#define SPT_CTECONFIG_T28                         28u
#define SPT_GPI_T29                               29u
#define SPT_GATE_T30                              30u
#define TOUCH_KEYSET_T31                          31u
#define TOUCH_XSLIDERSET_T32                      32u
#define DIAGNOSTIC_T37			37u
struct info_id_t {
	uint8_t family_id;
	uint8_t variant_id;
	uint8_t version;
	uint8_t build;
	uint8_t matrix_x_size;
	uint8_t matrix_y_size;
	uint8_t num_declared_objects;
};

struct object_t {
	uint8_t object_type;
	uint16_t i2c_address;
	uint8_t size;
	uint8_t instances;
	uint8_t num_report_ids;
	uint8_t report_ids;
};

struct atmel_virtual_key {
	int keycode;
	int range_min;
	int range_max;
};

struct atmel_finger_data {
	int x;
	int y;
	int w;
	int z;
	int id;
};

struct atmel_i2c_platform_data {
	uint16_t version;
	uint16_t source;
	uint16_t abs_x_min;
	uint16_t abs_x_max;
	uint16_t abs_y_min;
	uint16_t abs_y_max;
	uint8_t abs_pressure_min;
	uint8_t abs_pressure_max;
	uint8_t abs_width_min;
	uint8_t abs_width_max;
	int gpio_irq;
	int (*power)(int on);
	int8_t config_T6[6];
	int8_t config_T7[3];
	int8_t config_T8[8];
	int8_t config_T9[31];
	int8_t config_T18[2];
	int8_t config_T15[11];
	int8_t config_T19[12];
	int8_t config_T20[12];
	int8_t config_T22[17];
	int8_t config_T23[13];
	int8_t config_T24[19];
	int8_t config_T25[14];
	int8_t config_T27[7];
	int8_t config_T28[6];
	int8_t config_T38[8];
	uint8_t object_crc[3];
	int8_t cable_config[4];
	int8_t cable_config_T7[3];
	int8_t cable_config_T8[8];
	int8_t cable_config_T9[31];
	int8_t cable_config_T22[17];
	int8_t cable_config_T28[6];
	uint16_t filter_level[4];
	uint8_t GCAF_level[5];
	int display_width;	/* display width in pixel */
	int display_height;	/* display height in pixel */
};

struct atmel_config_data {
	int8_t config[4];
	int8_t *config_T7;
	int8_t *config_T8;
	int8_t *config_T9;
	int8_t *config_T22;
	int8_t *config_T28;
};

#endif




struct atmel_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *atmel_wq;
	struct work_struct work;
	int (*power) (int on);
	struct early_suspend early_suspend;
	struct info_id_t *id;
	struct object_t *object_table;
	uint8_t finger_count;
	uint16_t abs_x_min;
	uint16_t abs_x_max;
	uint16_t abs_y_min;
	uint16_t abs_y_max;
	uint8_t abs_pressure_min;
	uint8_t abs_pressure_max;
	uint8_t abs_width_min;
	uint8_t abs_width_max;
	uint8_t first_pressed;
	uint8_t debug_log_level;
	struct atmel_finger_data finger_data[10];
	uint8_t finger_type;
	uint8_t finger_support;
	uint16_t finger_pressed;
	uint8_t face_suppression;
	uint8_t grip_suppression;
	uint8_t noise_status[2];
	uint16_t *filter_level;
	uint8_t calibration_confirm;
	uint64_t timestamp;
	struct atmel_config_data config_setting[2];
	uint8_t status;
	uint8_t GCAF_sample;
	uint8_t *GCAF_level;
	uint8_t noisethr;
#ifdef ATMEL_EN_SYSFS
	struct device dev;
#endif

};





#endif /* TOUCHPANEL_H__ */
