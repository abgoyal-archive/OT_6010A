/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein is
 * confidential and proprietary to MediaTek Inc. and/or its licensors. Without
 * the prior written permission of MediaTek inc. and/or its licensors, any
 * reproduction, modification, use or disclosure of MediaTek Software, and
 * information contained herein, in whole or in part, shall be strictly
 * prohibited.
 * 
 * MediaTek Inc. (C) 2010. All rights reserved.
 * 
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER
 * ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
 * NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH
 * RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 * INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES
 * TO LOOK ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO.
 * RECEIVER EXPRESSLY ACKNOWLEDGES THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO
 * OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES CONTAINED IN MEDIATEK
 * SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE
 * RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S
 * ENTIRE AND CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE
 * RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE
 * MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE
 * CHARGE PAID BY RECEIVER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek
 * Software") have been modified by MediaTek Inc. All revisions are subject to
 * any receiver's applicable license agreements with MediaTek Inc.
 */

#ifndef _CFG_SETTING_IMGSENSOR_H_
#define _CFG_SETTING_IMGSENSOR_H_


/*******************************************************************************
* Image Sensor Orientation
*******************************************************************************/
SensorOrientation_T const&
getSensorOrientation()
{
    static SensorOrientation_T inst = {
        u4Degree_0  : 90,   //  main sensor in degree (0, 90, 180, 270)
        u4Degree_1  : 270,    //  sub  sensor in degree (0, 90, 180, 270)
    };
    //
    char str_value[32] = {'\0'};
    char str_default[32] = {'\0'};
    //
    ::sprintf(str_default, "%d", inst.u4Degree_0);
    property_get("persist.imgsensor0.orientation", str_value, str_default);
    property_set("persist.imgsensor0.orientation", str_value);
    inst.u4Degree_0 = ::atoi(str_value);
    //
    ::sprintf(str_default, "%d", inst.u4Degree_1);
    property_get("persist.imgsensor1.orientation", str_value, str_default);
    property_set("persist.imgsensor1.orientation", str_value);
    inst.u4Degree_1 = ::atoi(str_value);
    //
    return inst;
}


/*******************************************************************************
* Return fake orientation for front sensor or not
*******************************************************************************/
MBOOL isRetFakeSubOrientation()  
{
	return MFALSE;  // MTRUE: return degree 90 for front sensor in degree 0 or 180; MFALSE: not return fake orientation.
}


/*******************************************************************************
* Sensor Input Data Bit Order
*   Return:
*       0   : raw data input [9:2]
*       1   : raw data input [7:0]
*       -1  : error
*******************************************************************************/
MINT32
getSensorInputDataBitOrder(EDevId const eDevId)
{
    switch  (eDevId)
    {
    case eDevId_ImgSensor0:
        return  0;
    case eDevId_ImgSensor1:
        return  0;
    default:
        break;
    }
    return  -1;
}


/*******************************************************************************
* Sensor Pixel Clock Inverse in PAD side.
*   Return:
*       0   : no inverse
*       1   : inverse
*       -1  : error
*******************************************************************************/
MINT32
getSensorPadPclkInv(EDevId const eDevId)
{
    switch  (eDevId)
    {
    case eDevId_ImgSensor0:
        return  0;
    case eDevId_ImgSensor1:
        return  0;
    default:
        break;
    }
    return  -1;
}


#endif //  _CFG_SETTING_IMGSENSOR_H_

