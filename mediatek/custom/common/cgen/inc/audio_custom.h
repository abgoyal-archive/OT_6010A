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

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

/*******************************************************************************
 *
 * Filename:
 * ---------
 * aud_custom_exp.h
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 * This file is the header of audio customization related function or definition.
 *
 * Author:
 * -------
 * JY Huang
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 05 26 2010 chipeng.chang
 * [ALPS00002287][Need Patch] [Volunteer Patch] ALPS.10X.W10.11 Volunteer patch for audio paramter
 * modify audio parameter.
 *
 * 05 26 2010 chipeng.chang
 * [ALPS00002287][Need Patch] [Volunteer Patch] ALPS.10X.W10.11 Volunteer patch for audio paramter
 * modify for Audio parameter
 *
 *    mtk80306
 * [DUMA00132370] waveform driver file re-structure.
 * waveform driver file re-structure.
 *
 * Jul 28 2009 mtk01352
 * [DUMA00009909] Check in TWO_IN_ONE_SPEAKER and rearrange
 *
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#ifndef AUDIO_CUSTOM_H
#define AUDIO_CUSTOM_H

/* define Gain For Normal */
/* Normal volume: TON, SPK, MIC, FMR, SPH, SID, MED */

#define GAIN_NOR_TON_VOL      8
#define GAIN_NOR_KEY_VOL      43
#define GAIN_NOR_MIC_VOL      26
#define GAIN_NOR_FMR_VOL      0
#define GAIN_NOR_SPH_VOL      20
#define GAIN_NOR_SID_VOL      100
#define GAIN_NOR_MED_VOL      25


/* define Gain For Headset */
/* Headset volume: TON, SPK, MIC, FMR, SPH, SID, MED */

#define GAIN_HED_TON_VOL      8
#define GAIN_HED_KEY_VOL      24
#define GAIN_HED_MIC_VOL      20
#define GAIN_HED_FMR_VOL      24
#define GAIN_HED_SPH_VOL      12
#define GAIN_HED_SID_VOL      100
#define GAIN_HED_MED_VOL      12

/* define Gain For Handfree */
/* Handfree volume: TON, SPK, MIC, FMR, SPH, SID, MED */

#define GAIN_HND_TON_VOL      8
#define GAIN_HND_KEY_VOL      24
#define GAIN_HND_MIC_VOL      20
#define GAIN_HND_FMR_VOL      24
#define GAIN_HND_SPH_VOL      12
#define GAIN_HND_SID_VOL      100
#define GAIN_HND_MED_VOL      12

/* 0: Input FIR coefficients for 2G/3G Normal mode */
/* 1: Input FIR coefficients for 2G/3G/VoIP Headset mode */
/* 2: Input FIR coefficients for 2G/3G Handfree mode */
/* 3: Input FIR coefficients for 2G/3G/VoIP BT mode */
/* 4: Input FIR coefficients for VoIP Normal mode */
/* 5: Input FIR coefficients for VoIP Handfree mode */
#define SPEECH_INPUT_FIR_COEFF \
     32767,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
                                       \
    32767,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
                                       \
    32767,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
                                       \
    32767,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
                                       \
    32767,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
                                       \
    32767,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0

/* 0: Output FIR coefficients for 2G/3G Normal mode */
/* 1: Output FIR coefficients for 2G/3G/VoIP Headset mode */
/* 2: Output FIR coefficients for 2G/3G Handfree mode */
/* 3: Output FIR coefficients for 2G/3G/VoIP BT mode */
/* 4: Output FIR coefficients for VoIP Normal mode */
/* 5: Output FIR coefficients for VoIP Handfree mode */
#define SPEECH_OUTPUT_FIR_COEFF \
    32767,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
                                       \
    32767,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
                                       \
     -917,   865,  -778,   863,  -718,\
      485,  -599,  1006, -1199,  1092,\
     -763,   626, -1069,  2034, -2737,\
     2628, -1897,  1651, -2801,  5436,\
    -9540, 26032, 26032, -9540,  5436,\
    -2801,  1651, -1897,  2628, -2737,\
     2034, -1069,   626,  -763,  1092,\
    -1199,  1006,  -599,   485,  -718,\
      863,  -778,   865,  -917,     0,\
                                       \
    32767,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
                                       \
    32767,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
                                       \
    32767,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0

#define   DG_DL_Speech    0xE3D
#define   DG_Microphone   0x1400
#define   FM_Record_Vol   6     /* 0 is smallest. each step increase 1dB.
                                    Be careful of distortion when increase too much.
                                    Generally, it's not suggested to tune this parameter */

/* 0: Input FIR coefficients for 2G/3G Normal mode */
/* 1: Input FIR coefficients for 2G/3G/VoIP Headset mode */
/* 2: Input FIR coefficients for 2G/3G Handfree mode */
/* 3: Input FIR coefficients for 2G/3G/VoIP BT mode */
/* 4: Input FIR coefficients for VoIP Normal mode */
/* 5: Input FIR coefficients for VoIP Handfree mode */
#define WB_Speech_Input_FIR_Coeff \
      -52,    22,    28,   -18,   -32,    35,    26,   -71,    38,    17,\
        7,   -71,   100,   -88,    54,   -16,   -31,    71,   -51,    -7,\
       24,    91,  -292,   358,  -189,   -29,    34,   195,  -356,   199,\
      146,  -360,   351,  -344,   568,  -869,   910,  -796,  1084, -1989,\
     2976, -3437,  3799, -6094, 20675, 20675, -6094,  3799, -3437,  2976,\
    -1989,  1084,  -796,   910,  -869,   568,  -344,   351,  -360,   146,\
      199,  -356,   195,    34,   -29,  -189,   358,  -292,    91,    24,\
       -7,   -51,    71,   -31,   -16,    54,   -88,   100,   -71,     7,\
       17,    38,   -71,    26,    35,   -32,   -18,    28,    22,   -52,\
\
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  \
\
     -336,  -131,  -227,  -257,  -236,  -369,  -182,  -192,  -293,  -456,\
     -294,  -442,  -256,  -348,  -308,  -354,   -95,  -351,  -334,  -293,\
     -667,     4,  -546,  -123,  -549,    64,   218,   337,   430,   560,\
      811,   646,  1289,     5,   922,   -36,   572,  -242,  1209, -1864,\
     1151,  -274,  1294,  3366, 20675, 20675,  3366,  1294,  -274,  1151,\
    -1864,  1209,  -242,   572,   -36,   922,     5,  1289,   646,   811,\
      560,   430,   337,   218,    64,  -549,  -123,  -546,     4,  -667,\
     -293,  -334,  -351,   -95,  -354,  -308,  -348,  -256,  -442,  -294,\
     -456,  -293,  -192,  -182,  -369,  -236,  -257,  -227,  -131,  -336,\
\
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  \
\
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  \
\
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0

/* 0: Output FIR coefficients for 2G/3G Normal mode */
/* 1: Output FIR coefficients for 2G/3G/VoIP Headset mode */
/* 2: Output FIR coefficients for 2G/3G Handfree mode */
/* 3: Output FIR coefficients for 2G/3G/VoIP BT mode */
/* 4: Output FIR coefficients for VoIP Normal mode */
/* 5: Output FIR coefficients for VoIP Handfree mode */

#define WB_Speech_Output_FIR_Coeff \
     -162,   -16,  -152,  -139,   -77,  -114,  -144,  -125,   -96,  -339,\
     -107,  -277,   -49,  -427,  -151,  -332,  -115,  -100,  -278,  -228,\
     -522,   -50,  -916,   -43,  -625,  -400, -1057,   573,  -410,  -496,\
      204,   120,    89, -1301,  2991,  -507,    79, -4767,  1349, -1503,\
     7986, -8335, 11547, -9449, 32767, 32767, -9449, 11547, -8335,  7986,\
    -1503,  1349, -4767,    79,  -507,  2991, -1301,    89,   120,   204,\
     -496,  -410,   573, -1057,  -400,  -625,   -43,  -916,   -50,  -522,\
     -228,  -278,  -100,  -115,  -332,  -151,  -427,   -49,  -277,  -107,\
     -339,   -96,  -125,  -144,  -114,   -77,  -139,  -152,   -16,  -162,\
\
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
\
       93,    13,  -126,     6,   -26,    66,    58,   -94,    56,   -15,\
       62,   -96,   -59,   -61,   169,   144,     4,  -191,  -203,    -3,\
      216,   341,  -197,    47,  -704,   337,   111,   432,   123,  -587,\
     -295,   -62,   536,   205,   569, -1396,   768, -1674,  2439, -1608,\
     2690, -3996,  4219, -8329, 26028, 26028, -8329,  4219, -3996,  2690,\
    -1608,  2439, -1674,   768, -1396,   569,   205,   536,   -62,  -295,\
     -587,   123,   432,   111,   337,  -704,    47,  -197,   341,   216,\
       -3,  -203,  -191,     4,   144,   169,   -61,   -59,   -96,    62,\
      -15,    56,   -94,    58,    66,   -26,     6,  -126,    13,    93,\
\
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
\
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
\
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0

/*
 * The Bluetooth DAI Hardware COnfiguration Parameter
 */
#define DEFAULT_BLUETOOTH_SYNC_TYPE               0
#define DEFAULT_BLUETOOTH_SYNC_LENGTH             1

#endif
