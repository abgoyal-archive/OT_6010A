#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/input/mt.h> 

#include "tpd_custom_mxT224.h"
#ifdef MT6575
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#endif


#ifdef MT6577
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#endif

#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>

#include "cust_gpio_usage.h"

#define KPD_BUTTON_KEY	0
#define CHARGER_STATUS_TP 1

#if KPD_BUTTON_KEY
void backkey_handler(void);
void homekey_handler(void);
void menukey_handler(void);



static DECLARE_TASKLET(homekey_tasklet, homekey_handler, 0);
static DECLARE_TASKLET(menukey_tasklet, menukey_handler, 0);

static DECLARE_TASKLET(backkey_tasklet, backkey_handler, 0);

#endif
 
extern struct tpd_device *tpd;
 
struct i2c_client *i2c_client = NULL;
struct task_struct *thread = NULL;
 
static DECLARE_WAIT_QUEUE_HEAD(waiter);
 
 
static void tpd_eint_interrupt_handler(void);
 
#ifdef MT6575 
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);

 extern void mt65xx_eint_unmask(unsigned int line);
 extern void mt65xx_eint_mask(unsigned int line);
 extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
 extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
 extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
									  kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
									  kal_bool auto_umask);
#endif
#ifdef MT6577
	extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
	extern void mt65xx_eint_unmask(unsigned int line);
	extern void mt65xx_eint_mask(unsigned int line);
	extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
	extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
	extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif

 
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
 

static int tpd_flag = 0;
//static int point_num = 0;
static int p_point_num = 0;

//#define TPD_CLOSE_POWER_IN_SLEEP

#define TPD_OK 0
//register define

#define DEVICE_MODE 0x00
#define GEST_ID 0x01
#define TD_STATUS 0x02

#define TOUCH1_XH 0x03
#define TOUCH1_XL 0x04
#define TOUCH1_YH 0x05
#define TOUCH1_YL 0x06

#define TOUCH2_XH 0x09
#define TOUCH2_XL 0x0A
#define TOUCH2_YH 0x0B
#define TOUCH2_YL 0x0C

#define TOUCH3_XH 0x0F
#define TOUCH3_XL 0x10
#define TOUCH3_YH 0x11
#define TOUCH3_YL 0x12
//register define

#define TPD_RESET_ISSUE_WORKAROUND

#define TPD_MAX_RESET_COUNT 3

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif


struct touch_info {
    int y[3];
    int x[3];
    int p[3];
    int id[3];
    int count;
};




#define CFG_CRC1	0xaa
#define CFG_CRC2	0xbb
#define CFG_CRC3	0xcc

extern kal_bool *bat_status_for_tp;

/* Registers */
#define mxt_FAMILY_ID			0x00
#define mxt_VARIANT_ID			0x01
#define mxt_VERSION				0x02
#define mxt_BUILD				0x03
#define mxt_MATRIX_X_SIZE		0x04
#define mxt_MATRIX_Y_SIZE		0x05
#define mxt_OBJECT_NUM			0x06
#define mxt_OBJECT_START		0x07

#define mxt_OBJECT_SIZE			6

/* Object types */
#define mxt_DEBUG_DIAGNOSTIC	37
#define mxt_GEN_MESSAGE			5
#define mxt_GEN_COMMAND			6
#define mxt_GEN_POWER			7
#define mxt_GEN_ACQUIRE			8
#define mxt_TOUCH_MULTI			9
#define mxt_TOUCH_KEYARRAY		15
#define mxt_TOUCH_PROXIMITY		23
#define mxt_PROCI_GRIPFACE		20
#define mxt_PROCG_NOISE			22
#define mxt_PROCI_ONETOUCH		24
#define mxt_PROCI_TWOTOUCH		27
#define mxt_SPT_COMMSCONFIG		18	/* firmware ver 21 over */
#define mxt_SPT_GPIOPWM			19
#define mxt_SPT_SELFTEST		25
#define mxt_SPT_CTECONFIG		28
#define mxt_SPT_USERDATA		38	/* firmware ver 21 over */
#define mxt_golden_reference	64

/* mxt_GEN_COMMAND field T6*/
#define mxt_COMMAND_RESET		0
#define mxt_COMMAND_BACKUPNV	1
#define mxt_COMMAND_CALIBRATE	2
#define mxt_COMMAND_REPORTALL	3
#define mxt_COMMAND_DIAGNOSTIC	5

/* mxt_GEN_POWER field T7*/
#define mxt_POWER_IDLEACQINT	0
#define mxt_POWER_ACTVACQINT	1
#define mxt_POWER_ACTV2IDLETO	2

/* mxt_GEN_ACQUIRE field T8*/
#define mxt_ACQUIRE_CHRGTIME		0
#define mxt_ACQUIRE_TCHDRIFT		2
#define mxt_ACQUIRE_DRIFTST			3
#define mxt_ACQUIRE_TCHAUTOCAL		4
#define mxt_ACQUIRE_SYNC			5
#define mxt_ACQUIRE_ATCHCALST		6
#define mxt_ACQUIRE_ATCHCALSTHR		7
#define mxt_ACQUIRE_ATCHFRCCALTHR	8
#define mxt_ACQUIRE_ATCHFRCCALRATIO	9

/* mxt_TOUCH_MULTI field T9*/
#define mxt_TOUCH_CTRL			0
#define mxt_TOUCH_XORIGIN		1
#define mxt_TOUCH_YORIGIN		2
#define mxt_TOUCH_XSIZE			3
#define mxt_TOUCH_YSIZE			4
#define mxt_TOUCH_BLEN			6
#define mxt_TOUCH_TCHTHR		7
#define mxt_TOUCH_TCHDI			8
#define mxt_TOUCH_ORIENT		9
#define mxt_TOUCH_MOVHYSTI		11
#define mxt_TOUCH_MOVHYSTN		12
#define mxt_TOUCH_NUMTOUCH		14
#define mxt_TOUCH_MRGHYST		15
#define mxt_TOUCH_MRGTHR		16
#define mxt_TOUCH_AMPHYST		17
#define mxt_TOUCH_XRANGE_LSB	18
#define mxt_TOUCH_XRANGE_MSB	19
#define mxt_TOUCH_YRANGE_LSB	20
#define mxt_TOUCH_YRANGE_MSB	21
#define mxt_TOUCH_XLOCLIP		22
#define mxt_TOUCH_XHICLIP		23
#define mxt_TOUCH_YLOCLIP		24
#define mxt_TOUCH_YHICLIP		25
#define mxt_TOUCH_XEDGECTRL		26
#define mxt_TOUCH_XEDGEDIST		27
#define mxt_TOUCH_YEDGECTRL		28
#define mxt_TOUCH_YEDGEDIST		29
#define mxt_TOUCH_JUMPLIMIT		30	
#define mxt_TOUCH_TCHHYST		31

/* mxt_PROCI_GRIPFACE field T20*/
#define mxt_GRIPFACE_CTRL		0
#define mxt_GRIPFACE_XLOGRIP	1
#define mxt_GRIPFACE_XHIGRIP	2
#define mxt_GRIPFACE_YLOGRIP	3
#define mxt_GRIPFACE_YHIGRIP	4
#define mxt_GRIPFACE_MAXTCHS	5
#define mxt_GRIPFACE_SZTHR1		7
#define mxt_GRIPFACE_SZTHR2		8
#define mxt_GRIPFACE_SHPTHR1	9
#define mxt_GRIPFACE_SHPTHR2	10
#define mxt_GRIPFACE_SUPEXTTO	11

/* mxt_PROCI_NOISE field T22*/
#define mxt_NOISE_CTRL			0
#define mxt_NOISE_OUTFLEN		1
#define mxt_NOISE_GCAFUL_LSB	3
#define mxt_NOISE_GCAFUL_MSB	4
#define mxt_NOISE_GCAFLL_LSB	5
#define mxt_NOISE_GCAFLL_MSB	6
#define mxt_NOISE_ACTVGCAFVALID	7
#define mxt_NOISE_NOISETHR		8
#define mxt_NOISE_FREQHOPSCALE	10
#define mxt_NOISE_FREQ0			11
#define mxt_NOISE_FREQ1			12
#define mxt_NOISE_FREQ2			13
#define mxt_NOISE_FREQ3			14
#define mxt_NOISE_FREQ4			15
#define mxt_NOISE_IDLEGCAFVALID	16

/* mxt_SPT_COMMSCONFIG */
#define mxt_COMMS_CTRL		0
#define mxt_COMMS_CMD		1

/* mxt_SPT_CTECONFIG field */
#define mxt_CTE_CTRL		0
#define mxt_CTE_CMD		1
#define mxt_CTE_MODE		2
#define mxt_CTE_IDLEGCAFDEPTH	3
#define mxt_CTE_ACTVGCAFDEPTH	4
#define mxt_CTE_VOLTAGE		5	/* firmware ver 21 over */

#define mxt_VOLTAGE_DEFAULT	2700000
#define mxt_VOLTAGE_STEP		10000

/* Define for mxt_GEN_COMMAND */
#define mxt_BOOT_VALUE			0xa5
#define mxt_BACKUP_VALUE		0x55
#define mxt_BACKUP_TIME			25	/* msec */
#define mxt_RESET_TIME			65	/* msec */

#define mxt_FWRESET_TIME		175	/* msec */

/* Command to unlock bootloader */
#define mxt_UNLOCK_CMD_MSB		0xaa
#define mxt_UNLOCK_CMD_LSB		0xdc

/* Bootloader mode status */
#define mxt_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define mxt_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define mxt_FRAME_CRC_CHECK	0x02
#define mxt_FRAME_CRC_FAIL		0x03
#define mxt_FRAME_CRC_PASS		0x04
#define mxt_APP_CRC_FAIL		0x40	/* valid 7 8 bit only */
#define mxt_BOOT_STATUS_MASK	0x3f

/* Touch status */
#define mxt_SUPPRESS		(1 << 1)
#define mxt_AMP			(1 << 2)
#define mxt_VECTOR			(1 << 3)
#define mxt_MOVE			(1 << 4)
#define mxt_RELEASE		(1 << 5)
#define mxt_PRESS			(1 << 6)
#define mxt_DETECT			(1 << 7)

/* Touchscreen absolute values */
#define mxt_MAX_XC			0x3ff
#define mxt_MAX_YC			0x3ff
#define mxt_MAX_AREA		0xff

#define mxt_MAX_FINGER		10


/*Add for debug */
#define mxt_T8_FLAG

#ifdef mxt_T8_FLAG

#define unlock_time_threshold 10
int start_workround_time =0;
//2013-05-06
u8 flag_t8 = 0;	//add for mark the T8 object
struct timex  txc;
struct rtc_time tm;

#endif

#if 1
struct atmel_object_data {
	u8 config_T7[3];//GEN_POWER
	u8 config_T8[10];//GEN_ACQUIRE
	u8 config_T9[35];//TOUCH_MULTI
	u8 config_T15[11];//TOUCH_KEYARRAY
	u8 config_T18[2];
	u8 config_T19[16];//GPIOPWM
//	u8 config_T20[12];//GRIPFACE
//	u8 config_T22[15];//NOISE
	u8 config_T23[15];//TOUCH_PROXIMITY
//	u8 config_T24[14];//gesture_ONETOUCH
	u8 config_T25[15];//SELFTEST
//	u8 config_T27[6];//gesture_TWOTOUCH
//	u8 config_T28[6];//CTECONFIG
//	u8 config_T38[8];//golden reference
	u8 config_T40[5];
	u8 config_T42[8];
	u8 config_T46[9];
	u8 config_T47[10];
	u8 config_T48[54];

};
struct atmel_object_data atmel_initial_data =
{
	.config_T7 = {20,10,25},
	.config_T8 = {24,0,10,10,0,0,255,1,0,0},
	.config_T9={131,0,0,19,11,0,16,40,2,1,5,3,1,0,10,15,15,15,31,03,223,01,3,3,0,0,0xe0,45,0xA0,76,15,10,0,0,0},//byte 14 point num, performance
	.config_T15={0,0,0,0,0,0,0,0,0,0,0},
	.config_T18={0,0},
	.config_T19={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	.config_T23={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	.config_T25={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	.config_T40={0,0,0,0,0},
	.config_T42={3,40,60,60,0,0,10,0},			//palm open , 4th byte means area
	.config_T46={0,3,16,16,0,0,1,0,0},
	.config_T47={0,0,0,0,0,0,0,0,0,0},
	.config_T48={0,128,242,0,0,  0,0,0,15,30,  0,20,0,32,16,  0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,20,  32,15,5,0,0,  50,2,10,10,0,  
	10,10,10,3,3,  0,0,224,45,160,  76,15,10,2},		//first byte for charger on or off

//	.config_T48={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
//		0,0,0,0,0,0,0,0},				//charger off
};
#endif

#if 0

struct atmel_object_data {
	u8 config_T7[3];//GEN_POWER
	u8 config_T8[10];//GEN_ACQUIRE
	u8 config_T9[35];//TOUCH_MULTI
	u8 config_T15[11];//TOUCH_KEYARRAY
	u8 config_T18[2];
	u8 config_T19[16];//GPIOPWM
//	u8 config_T20[12];//GRIPFACE
//	u8 config_T22[15];//NOISE
	u8 config_T23[15];//TOUCH_PROXIMITY
//	u8 config_T24[14];//gesture_ONETOUCH
	u8 config_T25[14];//SELFTEST
//	u8 config_T27[6];//gesture_TWOTOUCH
//	u8 config_T28[6];//CTECONFIG
//	u8 config_T38[8];//golden reference
	u8 config_T40[5];
	u8 config_T42[8];
	u8 config_T46[9];
	u8 config_T47[10];
	u8 config_T48[54];

};

struct atmel_object_data atmel_initial_data = {    
	/*[GEN_POWERCONFIG_T7 INSTANCE 0] 272*/    
	.config_T7 ={0x20, 0x0c, 0x32},	
	/*[GEN_ACQUISITIONCONFIG_T8 INSTANCE 0]275*/    
	.config_T8 ={0x18, 0x00, 0x14, 0x14, 0x00, 0x00, 0x00, 0x28, 0x32, 0x00},  
	/*[TOUCH_MULTITOUCHSCREEN_T9 INSTANCE 0]285*/    
	.config_T9 ={0x83, 0x00, 0x00, 0x13, 0x0b, 0x00, 0x15, 0x28, 0x02, 0x01,
	0x0a, 0x01, 0x01, 0x42, 0x04, 0x0a, 0x1e, 0x0a, 0xff, 0x0f,
	0xff, 0x0f, 0x0f, 0x00, 0x04, 0x04, 0x97, 0x2f, 0x9c, 0x4d,
	0x64, 0x0a, 0x00, 0x00, 0x02},
	/*[TOUCH_KEYARRAY_T15 INSTANCE 0]320*/    
	.config_T15 ={0x00, 0x00, 0x0b, 0x02, 0x01, 0x00, 0x01, 0x28, 0x02, 0x00,
	0x00},   
	/*[SPT_COMMSCONFIG_T18 INSTANCE 0]331*/    
	.config_T18 ={0x00, 0x00},    
	/*[SPT_GPIOPWM_T19 INSTANCE 0]333*/    
	.config_T19 ={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 	
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    
	/*[TOUCH_PROXIMITY_T23 INSTANCE 0]349*/    
	.config_T23 ={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 	
	0x00, 0x00, 0x00, 0x00, 0x00},    
	/*[SPT_SELFTEST_T25 INSTANCE 0]364*/    
	.config_T25 = {0x03, 0x00, 0xf8, 0x2a, 0x58, 0x1b, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00},    
	/*[PROCI_GRIPSUPPRESSION_T40 INSTANCE 0]378*/    
	.config_T40 ={0x00, 0x00, 0x00, 0x00, 0x00},    
	/*[PROCI_TOUCHSUPPRESSION_T42 INSTANCE 0]383*/    
	.config_T42={ 0x00, 0x14, 0x20, 0x37, 0x00, 0x00, 0x04, 0x00},   
	/*[SPT_CTECONFIG_T46 INSTANCE 0]391*/
	.config_T46 ={0x00, 0x03, 0x20, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00},    
	/*[PROCI_STYLUS_T47 INSTANCE 0]400*/    
	.config_T47 ={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    
	/*[PROCG_NOISESUPPRESSION_T48 INSTANCE 0]410*/    
	.config_T48 ={0x00, 0x83, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x09,     
	0x00, 0x00, 0x00, 0x14, 0x14, 0x00, 0x00, 0x50, 0x0a, 0x28,     
	0x1e, 0x00, 0x14, 0x04, 0x00, 0x22, 0x00, 0x14, 0x00, 0x00,	
	0x00, 0x00, 0x00, 0x00, 0x05, 0x30, 0x02, 0x01, 0x01, 0x42,        
	0x04, 0x0a, 0x0a, 0x0d, 0x02, 0x04, 0x04, 0x97, 0x2f, 0x9c,     
	0x4d, 0x64, 0x0b, 0x02},
};
#endif


struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
	u8 config_CRC[6];
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size;
	u8 instances;
	u8 num_report_ids;

	/* to map object and message */
	u8 max_reportid;
};

struct mxt_message {
	u8 reportid;
	u8 field[7];
	u8 checksum;
};

struct mxt_finger {
	int status;
	int previews_status;
	int report_enable;
	int x;
	int y;
	int area;
	int strenth;
};

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct mxt_platform_data *pdata;
	struct mxt_object *object_table;
	struct mxt_info info;
	struct mxt_finger finger[mxt_MAX_FINGER];
	unsigned int irq;
};



 static const struct i2c_device_id mxt224e_tpd_id[] = {{"mxt224e",0},{}};
 static struct i2c_board_info __initdata mxt224e_i2c_tpd={ I2C_BOARD_INFO("mxt224e", 0x4a)};
 
 
 static struct i2c_driver tpd_i2c_driver = {
  .driver = {
	 .name = "mxt224e",//.name = TPD_DEVICE,
//	 .owner = THIS_MODULE,
  },
  .probe = tpd_probe,
  .remove = __devexit_p(tpd_remove),
  .id_table = mxt224e_tpd_id,
  .detect = tpd_detect,
//  .address_data = &addr_data,
 };

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	case mxt_GEN_MESSAGE:
	case mxt_GEN_COMMAND:
	case mxt_GEN_POWER:
	case mxt_GEN_ACQUIRE:
	case mxt_TOUCH_MULTI:
	case mxt_TOUCH_KEYARRAY:
	case mxt_TOUCH_PROXIMITY:
	case mxt_PROCI_GRIPFACE:
	case mxt_PROCG_NOISE:
	case mxt_PROCI_ONETOUCH:
	case mxt_PROCI_TWOTOUCH:
	case mxt_SPT_COMMSCONFIG:
	case mxt_SPT_GPIOPWM:
	case mxt_SPT_SELFTEST:
	case mxt_SPT_CTECONFIG:
	case mxt_SPT_USERDATA:
		return true;
	default:
		return false;
	}
}

static bool mxt_object_writable(unsigned int type)
{
	switch (type) {
	case mxt_GEN_COMMAND:
	case mxt_GEN_POWER:
	case mxt_GEN_ACQUIRE:
	case mxt_TOUCH_MULTI:
	case mxt_TOUCH_KEYARRAY:
	case mxt_TOUCH_PROXIMITY:
	case mxt_PROCI_GRIPFACE:
	case mxt_PROCG_NOISE:
	case mxt_PROCI_ONETOUCH:
	case mxt_PROCI_TWOTOUCH:
	case mxt_SPT_GPIOPWM:
	case mxt_SPT_SELFTEST:
	case mxt_SPT_CTECONFIG:
		return true;
	default:
		return false;
	}
}

static void mxt_dump_message(struct device *dev,
				  struct mxt_message *message)
{
	dev_dbg(dev, "reportid:\t0x%x\n", message->reportid);
	dev_dbg(dev, "message1:\t0x%x\n", message->field[0]);
	dev_dbg(dev, "message2:\t0x%x\n", message->field[1]);
	dev_dbg(dev, "message3:\t0x%x\n", message->field[2]);
	dev_dbg(dev, "message4:\t0x%x\n", message->field[3]);
	dev_dbg(dev, "message5:\t0x%x\n", message->field[4]);
	dev_dbg(dev, "message6:\t0x%x\n", message->field[5]);
	dev_dbg(dev, "message7:\t0x%x\n", message->field[6]);
	dev_dbg(dev, "checksum:\t0x%x\n", message->checksum);
}

static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
//	struct i2c_msg xfer[2];
	u8 buf[2];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;


	i2c_master_send(i2c_client, buf, 2);
	i2c_master_recv(i2c_client, val, len);
#if 0
	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	if (i2c_transfer(client->adapter, xfer, 2) != 2) {
		dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
		return -EIO;
	}
#endif
	return 0;
}



static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = val;

	if (i2c_master_send(client, buf, 3) != 3) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt_read_object_table(struct i2c_client *client,
				      u16 reg, u8 *object_buf)
{
	return __mxt_read_reg(client, reg, mxt_OBJECT_SIZE,
				   object_buf);
}


static struct mxt_object *
mxt_get_object_type(struct mxt_data *data, u8 report_id)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;
		if ((object->max_reportid<= (report_id+object->num_report_ids))&&(object->max_reportid>=report_id))
			return object;
	}

	dev_err(&data->client->dev, "Invalid object type\n");
	return NULL;
}



static struct mxt_object * mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_err(&data->client->dev, "Invalid object type\n");
	return NULL;
}

static int mxt_read_message(struct mxt_data *data,
				 struct mxt_message *message)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, mxt_GEN_MESSAGE);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->client, reg,
			sizeof(struct mxt_message), message);
}

static int mxt_read_object(struct mxt_data *data,
				u8 type, u8 offset, u8 *val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->client, reg + offset, 1, val);
}



static int mxt_write_byte(struct mxt_data *data,u8 type,u8 offset,u8 value)
{
	struct mxt_object *object;
	u16 reg;
	u8 buffer[3];
	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	reg +=offset;
	buffer[0]=reg & 0xff;
	buffer[1]=(reg >> 8) & 0xff;
	buffer[2]=value;
	if (i2c_master_send(data->client, buffer, 3) != 3) ////should be modify by customer
		{
		dev_err(&data->client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
		}
	return 0;
}

static int mxt_write_object(struct mxt_data *data,
				 u8 type, u8 length, u8 *table)
{
	struct mxt_object *object;
	u16 reg;
	u8	i;
	u8	buffer[100];
	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	buffer[0]=reg & 0xff;
	buffer[1]=(reg >> 8) & 0xff;


	for(i=0;i<(object->size + 1);i++)
		{
		buffer[i+2]=table[i];
		}
	if (i2c_master_send(data->client, buffer, object->size+3) != object->size+3) ////should be modify by customer
		{
		dev_err(&data->client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
		}
	return 0;
}

static void mxt_input_report(struct mxt_data *data, int single_id)
{
	struct mxt_finger *finger = data->finger;
	int finger_num = 0;
	int id;

	for (id = 0; id < mxt_MAX_FINGER; id++) {
		if (!finger[id].report_enable)
			continue;
		finger[id].report_enable=0;
		input_mt_sync(tpd->dev);
		input_report_key(tpd->dev, BTN_TOUCH, 1);

		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR,
				255);
		input_report_abs(tpd->dev, ABS_MT_POSITION_X,
				480-finger[id].x);
		input_report_abs(tpd->dev, ABS_MT_POSITION_Y,
				800-finger[id].y);
		input_mt_sync(tpd->dev);
	}

//	input_report_key(tpd->dev, BTN_TOUCH, finger_num > 0);

	//if (status != mxt_RELEASE) {
	//	input_report_abs(tpd->dev, ABS_X, finger[single_id].x);
	//	input_report_abs(tpd->dev, ABS_Y, finger[single_id].y);
	//}
	input_sync(tpd->dev);
}

static void mxt_input_touchevent(struct mxt_finger *data,
				      struct mxt_message *message, int id)
{
//	struct mxt_finger *finger = data->finger;
//	u8 status = message->field[0];
	int x;
	int y;
	int area;

	x = (message->field[1] << 2) | ((message->field[3] & ~0x3f) >> 6);
	y = (message->field[2] << 2) | ((message->field[3] & ~0xf3) >> 2);
	area = message->field[4];
	if(id >= 2)
		id = id - 2;
	data[id].status = ((message->field[0]) & mxt_DETECT )?1: 0;
	data[id].report_enable=0x01;
	data[id].x = x;
	data[id].y = y;
	data[id].area = area;	

}

static int GEN_COMMAND_PROCESS (void)
{
return 0;
}

static int MULTI_TOUCH_PROCESS (struct mxt_finger  *data,
				 struct mxt_message *message,int id)
{
	mxt_input_touchevent(data, message, id);
	return 0;

	
}

static int TOUCH_PROCIMITY_PROCESS (void)
{
	return 0;

}
static int TOUCH_KEYARRAY_PROCESS (void)
{return 0;}

static int GRIPFACE_PROCESS (void)
{return 0;}

static int NOISE_PROCESS (void)
{return 0;}

static int ONETOUCH_GESTURE_PROCESS (void)
{return 0;}

static int TWOTOUCH_GESTURE_PROCESS (void)
{return 0;}

static void report_touch_date(struct mxt_finger *finger)
{
	unsigned char i;
#if CHARGER_STATUS_TP
	u16 reg;
	u8 charger_status_from_reg;
	unsigned char j = 0;
	struct mxt_object *object;

#endif
	unsigned int release_flag = 0;
	unsigned char point_num = 0;
	struct mxt_data *data = i2c_get_clientdata(i2c_client);
	for (i= 0; i < mxt_MAX_FINGER; i++) {
		if (!finger[i].report_enable)
			continue;
		finger[i].report_enable=0;
//		printk("%d, x= %d, y= %d, status= 0x%2x xxxx\n", i, finger[i].x, finger[i].y, finger[i].status);

		if(finger[i].status == 0){							//release
			input_mt_slot(tpd->dev, i);
			input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, -1);
		}
		else if(finger[i].status == 1){						//down
		input_mt_slot(tpd->dev, i);
		input_report_key(tpd->dev, BTN_TOUCH, 1);

		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, i+2);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR,
				255);
		input_report_abs(tpd->dev, ABS_MT_POSITION_X,
				480-finger[i].x);
		input_report_abs(tpd->dev, ABS_MT_POSITION_Y,
				800-finger[i].y);
		}
		
	}
	input_sync(tpd->dev);



#if CHARGER_STATUS_TP

	if (bat_status_for_tp !=NULL) {

		j = 0;
		for (i= 0; i < mxt_MAX_FINGER; i++) {
				if(finger[i].status == 0){							//release
					j++;
				}
		}
		if (j == 10) {
			object = mxt_get_object(data, 48);
			if (!object)
				return;
			reg = object->start_address;
			__mxt_read_reg(data->client, reg, 1, &charger_status_from_reg);

			if ((*bat_status_for_tp == 1)&&(charger_status_from_reg == 0 )) {
				printk("xxxx charger off to in\n");
				mxt_write_byte(data, 46, 2, 32);
				mxt_write_byte(data, 46, 3, 32);
				mxt_write_byte(data, 48, 0, 1);
			}
			else if ((*bat_status_for_tp == 0)&&(charger_status_from_reg == 1)) {
				printk("xxxx charger in to off\n");
				mxt_write_byte(data, 46, 2, 16);
				mxt_write_byte(data, 46, 3, 16);
				mxt_write_byte(data, 48, 0, 0);
			}
		}

	}

#endif


}



int read_mXT_data_resume()
{
	struct mxt_data *data = i2c_get_clientdata(i2c_client);
	struct mxt_message message;
	int i;

	for(i=0; i<mxt_MAX_FINGER; i++){
		if (mxt_read_message(data, &message)) 
		{
		printk("failed to read\n");
		}
	}
}


int mxt_interrupt()
{
	struct mxt_data *data = i2c_get_clientdata(i2c_client);
	struct mxt_finger finger_point[mxt_MAX_FINGER] = {0};
	struct mxt_message message;
	struct mxt_object *object;
	struct device *dev = &data->client->dev;
	u8 reportid;
	int i;
#ifdef mxt_T8_FLAG
	if (1 == flag_t8)
	{
		do_gettimeofday(&(txc.time));
	       rtc_time_to_tm(txc.time.tv_sec,&tm);
		   
//		 printk(" start_workround_time:%d\n tm.tm_sec: %d\n start_workround_time-tm.tm_sec: %d\n",
//		 	start_workround_time,tm.tm_sec,start_workround_time-tm.tm_sec);
		 
		 if (tm.tm_sec < start_workround_time)
		 {
			if(unlock_time_threshold<(start_workround_time+60-tm.tm_sec))
			{
				flag_t8=0;
		        	mxt_write_byte(data,mxt_GEN_ACQUIRE,mxt_ACQUIRE_ATCHCALST,255);
				mxt_write_byte(data,mxt_GEN_ACQUIRE,mxt_ACQUIRE_ATCHCALSTHR,1);
				mxt_write_byte(data,mxt_GEN_ACQUIRE,mxt_ACQUIRE_ATCHFRCCALTHR,0);
				mxt_write_byte(data,mxt_GEN_ACQUIRE,mxt_ACQUIRE_ATCHFRCCALRATIO,0);
				printk("unlock time out\n");
			}
		 }
		 else
		 {
		 	if(unlock_time_threshold<(start_workround_time-tm.tm_sec))
	 		{
	 			flag_t8=0;
	 			mxt_write_byte(data,mxt_GEN_ACQUIRE,mxt_ACQUIRE_ATCHCALST,255);
				mxt_write_byte(data,mxt_GEN_ACQUIRE,mxt_ACQUIRE_ATCHCALSTHR,1);
				mxt_write_byte(data,mxt_GEN_ACQUIRE,mxt_ACQUIRE_ATCHFRCCALTHR,0);
				mxt_write_byte(data,mxt_GEN_ACQUIRE,mxt_ACQUIRE_ATCHFRCCALRATIO,0);
				printk("unlock time out\n");
	 		}
		 }
	}

#endif 
	if (mxt_read_message(data, &message)) 
	{
		dev_err(dev, "Failed to read message\n");
	}

	reportid = message.reportid;
	if(reportid == 16){				//palm event support
		printk("xx id is %2x, message[0] = %2x \n", reportid, message.field[0]);
//		input_report_key(tpd->dev, KEY_JRD_PALMEVENT, message.field[0]);
//		input_sync(tpd->dev);
	}
	if((reportid == 0xff) || (reportid > 11) || (reportid <2))
		return 0;


	object=mxt_get_object_type(data,reportid);
	if (!object)
		printk("type error!\n");
#if 1
	switch (object->type)
	{
		case mxt_GEN_COMMAND:
		{
			GEN_COMMAND_PROCESS();
		}break;
		case mxt_TOUCH_MULTI:
		{
			MULTI_TOUCH_PROCESS(finger_point, &message,(int)(reportid));
		}break;
		case mxt_TOUCH_KEYARRAY:
		{
			TOUCH_KEYARRAY_PROCESS();
		}break;
		case mxt_TOUCH_PROXIMITY:
		{
			TOUCH_PROCIMITY_PROCESS();
		}break;
		case mxt_PROCI_GRIPFACE:
		{
			GRIPFACE_PROCESS();
		}break;
		case mxt_PROCG_NOISE:
		{
			NOISE_PROCESS();
		}break;
		case mxt_PROCI_ONETOUCH:
		{
			ONETOUCH_GESTURE_PROCESS();
		}break;
		case mxt_PROCI_TWOTOUCH:
		{
			TWOTOUCH_GESTURE_PROCESS();
		}break;
		default:break;
	}
#endif

	report_touch_date(finger_point);
	return 1;

} 

 static int touch_event_handler(void *unused)
{

	struct touch_info cinfo, pinfo;
	int i=0;

	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);

	do
	{
		mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		set_current_state(TASK_INTERRUPTIBLE); 
		wait_event_interruptible(waiter,tpd_flag!=0);

		tpd_flag = 0;

		set_current_state(TASK_RUNNING);
		mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

		mxt_interrupt();

	}while(!kthread_should_stop());

	return 0;
}
 
 static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info) 
 {
	 strcpy(info->type, TPD_DEVICE);	
	  return 0;
 }
 
 static void tpd_eint_interrupt_handler(void)
 {
	 TPD_DEBUG_PRINT_INT;
	 tpd_flag = 1;
	 wake_up_interruptible(&waiter);
	 
 }

static int mxt_read_test_object(struct mxt_data *data,
				u8 type, u8 *val, u8 length)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->client, reg, length, val);
}


static int mxt_check_reg_init(struct mxt_data *data)
{

unsigned char val[60];
int i= 0;
	//ini_cfg_data();

	if((CFG_CRC1 != (data->info.config_CRC[0]))||(CFG_CRC2 != (data->info.config_CRC[1]))\
		||(CFG_CRC3 != (data->info.config_CRC[2])))
//	if(1)
		{
		printk("now update Touch panel config data!\n");
		mxt_write_object(data, mxt_GEN_POWER, sizeof(atmel_initial_data.config_T7),
						atmel_initial_data.config_T7);

		mxt_read_test_object(data, mxt_GEN_POWER, val, sizeof(atmel_initial_data.config_T7));
		for(i=0; i<sizeof(atmel_initial_data.config_T7); i++)
			printk("%d, ", val[i]);
			printk("\n");
		mxt_write_object(data, mxt_GEN_ACQUIRE, sizeof(atmel_initial_data.config_T8),
						atmel_initial_data.config_T8);
		mxt_read_test_object(data, mxt_GEN_ACQUIRE, val, sizeof(atmel_initial_data.config_T8));
		for(i=0; i<sizeof(atmel_initial_data.config_T8); i++)
			printk("%d, ", val[i]);
			printk("\n");

		mxt_write_object(data, mxt_TOUCH_MULTI, sizeof(atmel_initial_data.config_T9),
						atmel_initial_data.config_T9);
		mxt_read_test_object(data, mxt_TOUCH_MULTI, val, sizeof(atmel_initial_data.config_T9));
		for(i=0; i<sizeof(atmel_initial_data.config_T9); i++)
			printk("%d, ", val[i]);
			printk("\n");

		mxt_write_object(data, mxt_TOUCH_KEYARRAY, sizeof(atmel_initial_data.config_T15),
						atmel_initial_data.config_T15);
		mxt_read_test_object(data, 15, val, sizeof(atmel_initial_data.config_T15));
		for(i=0; i<sizeof(atmel_initial_data.config_T15); i++)
			printk("%d, ", val[i]);
			printk("\n");

		mxt_write_object(data, mxt_SPT_COMMSCONFIG, sizeof(atmel_initial_data.config_T18),
						atmel_initial_data.config_T18);
		mxt_read_test_object(data, 18, val, sizeof(atmel_initial_data.config_T18));
		for(i=0; i<sizeof(atmel_initial_data.config_T18); i++)
			printk("%d, ", val[i]);
			printk("\n");


		mxt_write_object(data, mxt_SPT_GPIOPWM, sizeof(atmel_initial_data.config_T19),
						atmel_initial_data.config_T19);
		mxt_read_test_object(data, 19, val, sizeof(atmel_initial_data.config_T19));
		for(i=0; i<sizeof(atmel_initial_data.config_T19); i++)
			printk("%d, ", val[i]);
			printk("\n");

		mxt_write_object(data, mxt_TOUCH_PROXIMITY, sizeof(atmel_initial_data.config_T23),
						atmel_initial_data.config_T23);
		mxt_read_test_object(data, 23, val, sizeof(atmel_initial_data.config_T23));
		for(i=0; i<sizeof(atmel_initial_data.config_T23); i++)
			printk("%d, ", val[i]);
			printk("\n");

		mxt_write_object(data, 25, sizeof(atmel_initial_data.config_T25),
						atmel_initial_data.config_T25);
		mxt_read_test_object(data, 25, val, sizeof(atmel_initial_data.config_T25));
		for(i=0; i<sizeof(atmel_initial_data.config_T25); i++)
			printk("%d, ", val[i]);
			printk("\n");

		mxt_write_object(data, 40, sizeof(atmel_initial_data.config_T40),
						atmel_initial_data.config_T40);
		mxt_read_test_object(data, 40, val, sizeof(atmel_initial_data.config_T40));
		for(i=0; i<sizeof(atmel_initial_data.config_T40); i++)
			printk("%d, ", val[i]);
			printk("\n");


		mxt_write_object(data, 42, sizeof(atmel_initial_data.config_T42),
						atmel_initial_data.config_T42);
		mxt_read_test_object(data, 42, val, sizeof(atmel_initial_data.config_T42));
		for(i=0; i<sizeof(atmel_initial_data.config_T42); i++)
			printk("%d, ", val[i]);
			printk("\n");

		mxt_write_object(data, 46, sizeof(atmel_initial_data.config_T46),
								atmel_initial_data.config_T46);
		mxt_read_test_object(data, 46, val, sizeof(atmel_initial_data.config_T46));
		for(i=0; i<sizeof(atmel_initial_data.config_T46); i++)
			printk("%d, ", val[i]);
			printk("\n");

		mxt_write_object(data, 47, sizeof(atmel_initial_data.config_T47),
				atmel_initial_data.config_T47);
		mxt_read_test_object(data, 47, val, sizeof(atmel_initial_data.config_T47));
		for(i=0; i<sizeof(atmel_initial_data.config_T47); i++)
			printk("%d, ", val[i]);
			printk("\n");


		mxt_write_object(data, 48, sizeof(atmel_initial_data.config_T48),
						atmel_initial_data.config_T48);
		mxt_read_test_object(data, 48, val, sizeof(atmel_initial_data.config_T48));
		for(i=0; i<sizeof(atmel_initial_data.config_T48); i++)
			printk("%d, ", val[i]);
			printk("\n");


		
	}
	return 0;
}
#if 0
static int mxt_check_matrix_size(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	struct device *dev = &data->client->dev;
	int mode = -1;
	int error;
	u8 val;

	dev_dbg(dev, "Number of X lines: %d\n", pdata->x_line);
	dev_dbg(dev, "Number of Y lines: %d\n", pdata->y_line);

	switch (pdata->x_line) {
	case 0 ... 15:
		if (pdata->y_line <= 14)
			mode = 0;
		break;
	case 16:
		if (pdata->y_line <= 12)
			mode = 1;
		if (pdata->y_line == 13 || pdata->y_line == 14)
			mode = 0;
		break;
	case 17:
		if (pdata->y_line <= 11)
			mode = 2;
		if (pdata->y_line == 12 || pdata->y_line == 13)
			mode = 1;
		break;
	case 18:
		if (pdata->y_line <= 10)
			mode = 3;
		if (pdata->y_line == 11 || pdata->y_line == 12)
			mode = 2;
		break;
	case 19:
		if (pdata->y_line <= 9)
			mode = 4;
		if (pdata->y_line == 10 || pdata->y_line == 11)
			mode = 3;
		break;
	case 20:
		mode = 4;
	}

	if (mode < 0) {
		dev_err(dev, "Invalid X/Y lines\n");
		return -EINVAL;
	}

	error = mxt_read_object(data, mxt_SPT_CTECONFIG,
				mxt_CTE_MODE, &val);
	if (error)
		return error;

	if (mode == val)
		return 0;

	/* Change the CTE configuration */
	mxt_write_byte(data, mxt_SPT_CTECONFIG,
			mxt_CTE_CTRL, 1);
	mxt_write_byte(data, mxt_SPT_CTECONFIG,
			mxt_CTE_MODE, mode);
	mxt_write_byte(data, mxt_SPT_CTECONFIG,
			mxt_CTE_CTRL, 0);

	return 0;
}
#endif
static int mxt_make_highchg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int count = 10;
	int error;
	u8 val;

	/* Read dummy message to make high CHG pin */
	do {
		error = mxt_read_object(data, mxt_GEN_MESSAGE, 0, &val);
		if (error)
			return error;
	} while ((val != 0xff) && --count);

	if (!count) {
		dev_err(dev, "CHG pin isn't cleared\n");
		return -EBUSY;
	}

	return 0;
}
#if 0
static void mxt_handle_pdata(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	u8 voltage;

	/* Set touchscreen lines */
	mxt_write_byte(data, mxt_TOUCH_MULTI, mxt_TOUCH_XSIZE,
			pdata->x_line);
	mxt_write_byte(data, mxt_TOUCH_MULTI, mxt_TOUCH_YSIZE,
			pdata->y_line);

	/* Set touchscreen orient */
	mxt_write_byte(data, mxt_TOUCH_MULTI, mxt_TOUCH_ORIENT,
			pdata->orient);

	/* Set touchscreen burst length */
	mxt_write_byte(data, mxt_TOUCH_MULTI,
			mxt_TOUCH_BLEN, pdata->blen);

	/* Set touchscreen threshold */
	mxt_write_byte(data, mxt_TOUCH_MULTI,
			mxt_TOUCH_TCHTHR, pdata->threshold);

	/* Set touchscreen resolution */
	mxt_write_byte(data, mxt_TOUCH_MULTI,
			mxt_TOUCH_XRANGE_LSB, (pdata->x_size - 1) & 0xff);
	mxt_write_byte(data, mxt_TOUCH_MULTI,
			mxt_TOUCH_XRANGE_MSB, (pdata->x_size - 1) >> 8);
	mxt_write_byte(data, mxt_TOUCH_MULTI,
			mxt_TOUCH_YRANGE_LSB, (pdata->y_size - 1) & 0xff);
	mxt_write_byte(data, mxt_TOUCH_MULTI,
			mxt_TOUCH_YRANGE_MSB, (pdata->y_size - 1) >> 8);

	/* Set touchscreen voltage */
	if (data->info.version >= mxt_VER_21 && pdata->voltage) {
		if (pdata->voltage < mxt_VOLTAGE_DEFAULT) {
			voltage = (mxt_VOLTAGE_DEFAULT - pdata->voltage) /
				mxt_VOLTAGE_STEP;
			voltage = 0xff - voltage + 1;
		} else
			voltage = (pdata->voltage - mxt_VOLTAGE_DEFAULT) /
				mxt_VOLTAGE_STEP;

		mxt_write_byte(data, mxt_SPT_CTECONFIG,
				mxt_CTE_VOLTAGE, voltage);
	}
}
#endif
static int mxt_get_info(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;
	u8 val;

	error = __mxt_read_reg(client, mxt_FAMILY_ID,1, &val);
	if (error)
		return error;
	info->family_id = val;

	error = __mxt_read_reg(client, mxt_VARIANT_ID,1, &val);
	if (error)
		return error;
	info->variant_id = val;

	error = __mxt_read_reg(client, mxt_VERSION,1, &val);
	if (error)
		return error;
	info->version = val;

	error = __mxt_read_reg(client, mxt_BUILD,1, &val);
	if (error)
		return error;
	info->build = val;

	error = __mxt_read_reg(client, mxt_OBJECT_NUM,1, &val);
	if (error)
		return error;
	info->object_num = val;

	return 0;
}//get chip info is OK here


static int mxt_get_object_table(struct mxt_data *data)
{
	int error;
	int i;
	u16 reg;
	u8 reportid = 0;
	u8 buf[mxt_OBJECT_SIZE];

	for (i = 0; i < data->info.object_num; i++) 
		{
		struct mxt_object *object = data->object_table + i;

		reg = mxt_OBJECT_START + mxt_OBJECT_SIZE * i;
		error = mxt_read_object_table(data->client, reg, buf);
		if (error)
			return error;

		object->type = buf[0];
		object->start_address = (buf[2] << 8) | buf[1];
		object->size = buf[3];
		object->instances = buf[4];
		object->num_report_ids = buf[5];

		if (object->num_report_ids) 
			{
			reportid += object->num_report_ids *
					(object->instances + 1);
			object->max_reportid = reportid;
			}
		}
	reg=mxt_OBJECT_START + mxt_OBJECT_SIZE * data->info.object_num;
	error = mxt_read_object_table(data->client, reg, data->info.config_CRC);
	if (error)
			return error;
	return 0;
}//get object table is ok here

static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;
	//u8 val;
	printk("%s\n", __FUNCTION__);
	error = mxt_get_info(data);
	if (error)
		return error;

	data->object_table = kcalloc(info->object_num,
				     sizeof(struct mxt_data),
				     GFP_KERNEL);
	if (!data->object_table) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Get object table information */
	error = mxt_get_object_table(data);
	if (error)
		return error;
	printk("%s 1111111111\n", __FUNCTION__);

	/* Check register init values */
	error = mxt_check_reg_init(data);
	if (error)
		return error;

	/* Check X/Y matrix size */
	//error = mxt_check_matrix_size(data);
	//if (error)
		//return error;

//	error = mxt_make_highchg(data);				????????????
//	if (error)
//		return error;

	//mxt_handle_pdata(data);
	printk("%s 22222222222\n", __FUNCTION__);

	/* Backup to memory */
	mxt_write_byte(data, mxt_GEN_COMMAND,
			mxt_COMMAND_BACKUPNV,
			mxt_BACKUP_VALUE);
	msleep(mxt_BACKUP_TIME);
	printk("%s 3333333\n", __FUNCTION__);

	/* Soft reset */
	mxt_write_byte(data, mxt_GEN_COMMAND,
			mxt_COMMAND_RESET, 1);
	msleep(mxt_RESET_TIME);
	printk("%s4444444444\n", __FUNCTION__);

	/* Update matrix size at info struct */
	//error = mxt_read_reg(client, mxt_MATRIX_X_SIZE, &val);
	//if (error)
	//	return error;
	//info->matrix_xsize = val;

	//error = mxt_read_reg(client, mxt_MATRIX_Y_SIZE, &val);
	//if (error)
	//	return error;
	//info->matrix_ysize = val;

	dev_info(&client->dev,
			"Family ID: %d Variant ID: %d Version: %d Build: %d\n",
			info->family_id, info->variant_id, info->version,
			info->build);

	dev_info(&client->dev,
			"Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n",
			info->matrix_xsize, info->matrix_ysize,
			info->object_num);

	return 0;
}
static void mxt_start(struct mxt_data *data)
{
	/* Touch enable */
	mxt_write_byte(data,
			mxt_GEN_POWER, mxt_POWER_IDLEACQINT, atmel_initial_data.config_T7[mxt_POWER_IDLEACQINT]);
	mxt_write_byte(data,
			mxt_GEN_POWER, mxt_POWER_ACTVACQINT, atmel_initial_data.config_T7[mxt_POWER_ACTVACQINT]);

}

static void mxt_stop(struct mxt_data *data)
{
	/* Touch disable */
	mxt_write_byte(data,
			mxt_GEN_POWER, mxt_POWER_IDLEACQINT, 0);
	mxt_write_byte(data,
			mxt_GEN_POWER, mxt_POWER_ACTVACQINT, 0);
}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_start(data);

	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_stop(data);
}
#if KPD_BUTTON_KEY
void homekey_handler(void)
{
	int i;
	i = mt_get_gpio_in(GPIO72);
	printk("xxxxxxxxx value is %d xxxxxxxxx\n", i);
#if 1
	if(!i){
	printk("pressed home!xxxxxxxxx\n");
		input_report_key(tpd->dev, KEY_HOMEPAGE, 1);
		input_sync(tpd->dev);

		mt_set_gpio_mode(GPIO72, 1);
		mt_set_gpio_dir(GPIO72, GPIO_DIR_IN);
		mt65xx_eint_set_sens(4, 0);
		mt65xx_eint_set_polarity(4, 1);
//		mt65xx_eint_set_hw_debounce(4, 0);
//		mt65xx_eint_registration(4, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, 0, back_key_handler, 1); 
//		mt65xx_eint_unmask(4);

	}
	else if(i){
		printk("released home!xxxxxxx\n");
		input_report_key(tpd->dev, KEY_HOMEPAGE, 0);
		input_sync(tpd->dev);

		mt_set_gpio_mode(GPIO72, 1);
		mt_set_gpio_dir(GPIO72, GPIO_DIR_IN);
		mt65xx_eint_set_sens(4, 0);
		mt65xx_eint_set_polarity(4, 0);
//		mt65xx_eint_set_hw_debounce(4, 0);
//		mt65xx_eint_registration(4, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, 0, back_key_handler, 1); 
//		mt65xx_eint_unmask(4);

	}
#endif
}

void menukey_handler(void)
{
	int i;
	i = mt_get_gpio_in(GPIO68);
	printk("xxxxxxxxx value is %d xxxxxxxxx\n", i);
#if 1
	if(!i){
	printk("pressed menu!xxxxxxxxx\n");
		input_report_key(tpd->dev, KEY_MENU, 1);
		input_sync(tpd->dev);

		mt_set_gpio_mode(GPIO68, 2);
		mt_set_gpio_dir(GPIO68, GPIO_DIR_IN);
		mt65xx_eint_set_sens(11, 0);
		mt65xx_eint_set_polarity(11, 1);
//		mt65xx_eint_set_hw_debounce(4, 0);
//		mt65xx_eint_registration(4, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, 0, back_key_handler, 1); 
//		mt65xx_eint_unmask(4);

	}
	else if(i){
		printk("released menu!xxxxxxx\n");
		input_report_key(tpd->dev, KEY_MENU, 0);
		input_sync(tpd->dev);

		mt_set_gpio_mode(GPIO68, 2);
		mt_set_gpio_dir(GPIO68, GPIO_DIR_IN);
		mt65xx_eint_set_sens(11, 0);
		mt65xx_eint_set_polarity(11, 0);
//		mt65xx_eint_set_hw_debounce(4, 0);
//		mt65xx_eint_registration(4, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, 0, back_key_handler, 1); 
//		mt65xx_eint_unmask(4);

	}
#endif

}
void backkey_handler(void)
{
	int i;
	i = mt_get_gpio_in(GPIO14);
	printk("xxxxxxxxx value is %d xxxxxxxxx\n", i);
#if 1
	if(!i){
	printk("pressed back!xxxxxxxxx\n");
		input_report_key(tpd->dev, KEY_BACK, 1);
		input_sync(tpd->dev);

		mt_set_gpio_mode(GPIO14, 2);
		mt_set_gpio_dir(GPIO14, GPIO_DIR_IN);
		mt65xx_eint_set_sens(12, 0);
		mt65xx_eint_set_polarity(12, 1);
//		mt65xx_eint_set_hw_debounce(4, 0);
//		mt65xx_eint_registration(4, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, 0, back_key_handler, 1); 
//		mt65xx_eint_unmask(4);

	}
	else if(i){
		printk("released back!xxxxxxx\n");
		input_report_key(tpd->dev, KEY_BACK, 0);
		input_sync(tpd->dev);

		mt_set_gpio_mode(GPIO14, 2);
		mt_set_gpio_dir(GPIO14, GPIO_DIR_IN);
		mt65xx_eint_set_sens(12, 0);
		mt65xx_eint_set_polarity(12, 0);
//		mt65xx_eint_set_hw_debounce(4, 0);
//		mt65xx_eint_registration(4, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, 0, back_key_handler, 1); 
//		mt65xx_eint_unmask(4);

	}
#endif

}

static void back_key_handler(void)
{
	printk("xxxxxxxxxxxx1\n");
	tasklet_schedule(&backkey_tasklet);
}
static void home_key_handler(void)
{
	printk("xxxxxxxxxxxx2\n");
	tasklet_schedule(&homekey_tasklet);

}
static void menu_key_handler(void)
{
	printk("xxxxxxxxxxxx3\n");
	tasklet_schedule(&menukey_tasklet);
	

}
#endif
 static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
 {	 
	int retval = TPD_OK;
	int err=0;
	struct mxt_data *data;
	int error;

#if CHARGER_STATUS_TP
	struct mxt_object *object;
	u16 reg;
	u8 charger_status_from_reg;
#endif
	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if(!data)
		return -1;
		data->client = client;
		data->pdata = client->dev.platform_data;
		i2c_set_clientdata(client, data);
reset_proc:   
	i2c_client = client;

   	i2c_client->timing = 100;



#if KPD_BUTTON_KEY
//needed for button KEY

	hwPowerOn(MT65XX_POWER_LDO_VGP2,VOL_1800,"kpd_button");

	mt_set_gpio_mode(GPIO126, 0);
	mt_set_gpio_dir(GPIO126, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO126, GPIO_OUT_ONE);

//back key
	mt_set_gpio_mode(GPIO72, 1);
	mt_set_gpio_dir(GPIO72, GPIO_DIR_IN);

	mt65xx_eint_set_sens(4, 0);
	mt65xx_eint_set_hw_debounce(4, 0);
	mt65xx_eint_registration(4, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, 0, home_key_handler, 1); 
	mt65xx_eint_unmask(4);


//home key
	mt_set_gpio_mode(GPIO68, 2);
	mt_set_gpio_dir(GPIO68, GPIO_DIR_IN);

	mt65xx_eint_set_sens(11, 0);
	mt65xx_eint_set_hw_debounce(11, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt65xx_eint_registration(11, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, 0, menu_key_handler, 1); 
	mt65xx_eint_unmask(11);

//homepage key	
	mt_set_gpio_mode(GPIO14, 2);
	mt_set_gpio_dir(GPIO14, GPIO_DIR_IN);

	mt65xx_eint_set_sens(12, 0);
	mt65xx_eint_set_hw_debounce(12, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt65xx_eint_registration(12, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, 0, back_key_handler, 1); 
	mt65xx_eint_unmask(12);
#endif

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, 1);
	mt65xx_eint_set_polarity(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_POLARITY);
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 0); 
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	msleep(100);
 
	error = mxt_initialize(data);
	if (error)
		return -1;

#if CHARGER_STATUS_TP

	object = mxt_get_object(data, 48);
	if (!object)
		return -EINVAL;
	reg = object->start_address;
	__mxt_read_reg(data->client, reg, 1, &charger_status_from_reg);

	if(bat_status_for_tp !=NULL)
	{
		if ((*bat_status_for_tp == 1)&&(charger_status_from_reg == 0 )) {
			printk("xxxx charger off to in\n");			
			mxt_write_byte(data, 46, 2, 32);
			mxt_write_byte(data, 46, 3, 32);
			mxt_write_byte(data, 48, 0, 1);
		}
		else if ((*bat_status_for_tp == 0)&&(charger_status_from_reg == 1)) {
			printk("xxxx charger in to off\n");
			mxt_write_byte(data, 46, 2, 16);
			mxt_write_byte(data, 46, 3, 16);
			mxt_write_byte(data, 48, 0, 0);
		}
	}

#endif


	tpd_load_status = 1;


	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	 if (IS_ERR(thread))
		 { 
		  retval = PTR_ERR(thread);
		  TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
		}

	TPD_DMESG("mxt224e Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
   return 0;
   
 }

 static int __devexit tpd_remove(struct i2c_client *client)
 
 {
   
	 TPD_DEBUG("TPD removed\n");
 
   return 0;
 }
 
 
 static int tpd_local_init(void)
 {

 
  TPD_DMESG("Focaltech mxt224e I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
 
 
   if(i2c_add_driver(&tpd_i2c_driver)!=0)
   	{
  		TPD_DMESG("mxt224e unable to add i2c driver.\n");
      	return -1;
    }
    if(tpd_load_status == 0) 
    {
    	TPD_DMESG("mxt224e add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }

//	set_bit(KEY_JRD_PALMEVENT, tpd->dev->keybit);

#ifdef TPD_HAVE_BUTTON     
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   
  
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif
		input_mt_init_slots(tpd->dev, mxt_MAX_FINGER);
		TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
		tpd_type_cap = 1;
    return 0; 
 }

void release_all_fingers(void)
{
	unsigned char i;
	for(i=0; i<mxt_MAX_FINGER; i++){
		input_mt_slot(tpd->dev, i);
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, -1);
	}
	input_sync(tpd->dev);
}

 static void tpd_resume( struct early_suspend *h )
 {
	struct mxt_data *data = i2c_get_clientdata(i2c_client);
	printk("debug %s xxxx\n", __FUNCTION__);
	/* Soft reset */
	mxt_write_byte(data, mxt_GEN_COMMAND,
	mxt_COMMAND_RESET, 1);
	msleep(mxt_RESET_TIME);

	mxt_write_byte(data,
			mxt_GEN_POWER, mxt_POWER_IDLEACQINT, atmel_initial_data.config_T7[mxt_POWER_IDLEACQINT]);
	mxt_write_byte(data,
			mxt_GEN_POWER, mxt_POWER_ACTVACQINT, atmel_initial_data.config_T7[mxt_POWER_ACTVACQINT]);


	release_all_fingers();

#ifdef mxt_T8_FLAG
	mxt_write_byte(data,mxt_GEN_ACQUIRE,mxt_ACQUIRE_ATCHCALST,5);
	mxt_write_byte(data,mxt_GEN_ACQUIRE,mxt_ACQUIRE_ATCHCALSTHR,25);
	mxt_write_byte(data,mxt_GEN_ACQUIRE,mxt_ACQUIRE_ATCHFRCCALTHR,15);
	mxt_write_byte(data,mxt_GEN_ACQUIRE,mxt_ACQUIRE_ATCHFRCCALRATIO,192);
	flag_t8 = 1;

//start_workround_time = localtime();
//	time(&start_workround_time);

	do_gettimeofday(&(txc.time));
	rtc_time_to_tm(txc.time.tv_sec,&tm);
	start_workround_time = tm.tm_sec;
	printk("UTC time :%d-%d-%d %d:%d:%d \n",tm.tm_year+1900,tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);

	 
#endif

	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	while(!mt_get_gpio_in(GPIO_CTP_EINT_PIN)){
		printk("debug %s,  EINT low!xxxx\n", __FUNCTION__);
		read_mXT_data_resume();
		printk("debug: after reading EINT now is %d!\n", mt_get_gpio_in(GPIO_CTP_EINT_PIN));
	}
/*
	hwPowerOn(MT65XX_POWER_LDO_VGP2,VOL_1800,"kpd_button");
	mt65xx_eint_unmask(4);
	mt65xx_eint_unmask(11);
	mt65xx_eint_unmask(12);
	printk("touchkey resume\n");
*/
 }

 static void tpd_suspend( struct early_suspend *h )
 {
	struct mxt_data *data = i2c_get_clientdata(i2c_client);
	TPD_DMESG("TPD enter sleep\n");


	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	mxt_write_byte(data,
			mxt_GEN_POWER, mxt_POWER_IDLEACQINT, 0);
	mxt_write_byte(data,
			mxt_GEN_POWER, mxt_POWER_ACTVACQINT, 0);
	release_all_fingers();

/*
        hwPowerDown(MT65XX_POWER_LDO_VGP2,"kpd_button");
        mt65xx_eint_mask(4);
        mt65xx_eint_mask(11);
        mt65xx_eint_mask(12);
        printk("touchkey suspend\n");
*/
 } 


 static struct tpd_driver_t tpd_device_driver = {
		 .tpd_device_name = "mxt224e",
		 .tpd_local_init = tpd_local_init,
		 .suspend = tpd_suspend,
		 .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
		 .tpd_have_button = 1,
#else
		 .tpd_have_button = 0,
#endif		
 };
 /* called when loaded into kernel */
 static int __init tpd_driver_init(void) {
	printk("MediaTek mxt224e touch panel driver init\n");
	i2c_register_board_info(0, &mxt224e_i2c_tpd, 1);
	if(tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add mxt224e driver failed\n");
	return 0;
 }
 
 /* should never be called */
 static void __exit tpd_driver_exit(void) {
	 TPD_DMESG("MediaTek mxt224e touch panel driver exit\n");
	 //input_unregister_device(tpd->dev);
	 tpd_driver_remove(&tpd_device_driver);
 }
 
 module_init(tpd_driver_init);
 module_exit(tpd_driver_exit);


