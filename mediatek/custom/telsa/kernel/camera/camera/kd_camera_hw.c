#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    printk(KERN_INFO PFX "%s: " fmt, __FUNCTION__ ,##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         printk(KERN_ERR PFX "%s: " fmt, __FUNCTION__ ,##arg)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#endif

#define KD_MAIN_SENSOR_1ST    SENSOR_DRVNAME_OV5647_MIPI_RAW
#define KD_MAIN_SENSOR_2ND    SENSOR_DRVNAME_OV5648_MIPI_RAW
#define KD_SUB_SENSOR_1ST     SENSOR_DRVNAME_OV7690_YUV
#define KD_SUB_SENSOR_2ND     NULL
// PR 461862 rui.cui@tcl.com 2013-06-01 begin

static int count=0;

/******************************************************************************
 * Static function
******************************************************************************/
static int kd_main_1st_PowerOn(BOOL On, char* mode_name);
static int kd_main_2nd_PowerOn(BOOL On, char* mode_name);
static int kd_sub_1st_PowerOn(BOOL On, char* mode_name);
static int kd_sub_2nd_PowerOn(BOOL On, char* mode_name);

static int kd_main_1st_PowerOn(BOOL On, char* mode_name)
{

  //1st main sensor is SENSOR_DRVNAME_OV5647_MIPI_RAW
  //follow ov5647 spec, version 1.2, figure 2-4
  PK_DBG_FUNC("Start, Power: %d",On);
  
  if(On) //power on
  {

PK_DBG_FUNC("XXXXXXXXX5647 Start, Power: %d",On);
 	
    if (count==0)//before ov7690 fisrt initialize, set sub sensor PDN pin low
    {
    if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
    PK_DBG_FUNC("XXXXXXXXXGPIO_CAMERA_CMPDN1_PIN is low");
    }

    else if (count>=1)//disable sub sensor, set sub sensor PDN pin high
    {
    if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
    PK_DBG_FUNC("XXXXXXXXXGPIO_CAMERA_CMPDN1_PIN is high");
    }

    count++;// different power sequence for the first time 7690 initialize
    PK_DBG_FUNC("XXXXXXXXXXXcount is :%d",count);

    //set PDN high
    if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN_PIN,GPIO_CAMERA_CMPDN_PIN_M_GPIO)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN,GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}

    //turn on VDD_IO
    if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
    {
        PK_DBG("[CAMERA SENSOR] Fail to enable VCAM_D2\n");
        goto PowerEIO;
    }
    //turn on VDD_A
    if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
    {
        PK_DBG("[CAMERA SENSOR] Fail to enable VCAM_A\n");
        goto PowerEIO;
    }
    //turn on VDD_D
    if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name))
    {
         PK_DBG("[CAMERA SENSOR] Fail to enable VCAM_D\n");
         goto PowerEIO;
    }
    mdelay(10);//time for init cam
    
    //set PDN low
    if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN_PIN,GPIO_CAMERA_CMPDN_PIN_M_GPIO)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
    mdelay(5);//dummy delay here

    //turn on AF
    if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
    {
      PK_DBG("[CAMERA SENSOR] Fail to enable VCAM_A2\n");
      goto PowerEIO;
    }
    mdelay(5);//dummy delay here
    //enable AF
    if(mt_set_gpio_mode(GPIO214, GPIO_MODE_00)){PK_DBG("[CAMERA SENSOR AF ] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO214,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO214,GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}

  }
  else //power off
  {
    //diasble AF
    if(mt_set_gpio_mode(GPIO214, GPIO_MODE_00)){PK_DBG("[CAMERA SENSOR AF ] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO214,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO214,GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}

    //turn off AF
    if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
    {
        PK_DBG("[CAMERA SENSOR] Fail to CAMERA_POWER_VCAM_A2 power\n");
        goto PowerEIO;
    }
    //pull main camera ov5647 PDN pin high, 5647 standby
    if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN_PIN,GPIO_CAMERA_CMPDN_PIN_M_GPIO)){PK_DBG("[CAMERA LENS] set gpio mode failed!!\n");}
    if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN,GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} 

    //turn off VDD_D
    if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name))
    {
        PK_DBG("[CAMERA SENSOR] Fail to CAMERA_POWER_VCAM_D power\n");
        goto PowerEIO;
    }
    //turn off VDD_A
    if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
    {
        PK_DBG("[CAMERA SENSOR] Fail to CAMERA_POWER_VCAM_A power\n");
        goto PowerEIO;
    }        
    //turn off VDD_IO
    if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
    {
        PK_DBG("[CAMERA SENSOR] Fail to CAMERA_POWER_VCAM_D2 power\n");
        goto PowerEIO;
    }

  }
  
  //success~
  PK_DBG_FUNC("XXXXXXXXXXXX5647 Success");
  return 0;

PowerEIO:
  PK_DBG_FUNC("Failed!");
  return -EIO;
}
/*delete 5648
static int kd_main_2nd_PowerOn(BOOL On, char* mode_name)
{
  //2nd main sensor is SENSOR_DRVNAME_OV5648_MIPI_RAW
  //follow ov5648 spec, version 1.1, figure 2-4
  PK_DBG_FUNC("XXXXXXXXX5648 Start, Power: %d",On);
  
  if(On) //power on
  {
   if (count==0)//TEST LOW
    {
    if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
    PK_DBG_FUNC("5648XXXXXXXXXGPIO_CAMERA_CMPDN1_PIN is low");
    }

    else if (count>=1)//disable sub sensor, set sub sensor PDN pin high
    {
    if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
    PK_DBG_FUNC("5648XXXXXXXXXGPIO_CAMERA_CMPDN1_PIN is high");
    }
    count++;//for different power
    PK_DBG_FUNC("5648XXXXXXXXXXXcount is :%d",count);

    //set PDN low 
    if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN_PIN,GPIO_CAMERA_CMPDN_PIN_M_GPIO)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}

    //turn on VDD_IO
    if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
    {
        PK_DBG("[CAMERA SENSOR] Fail to enable VCAM_D2\n");
        goto PowerEIO;
    }
    //turn on VDD_A
    if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
    {
        PK_DBG("[CAMERA SENSOR] Fail to enable VCAM_A\n");
        goto PowerEIO;
    }
    //turn on VDD_D
    if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name))
    {
         PK_DBG("[CAMERA SENSOR] Fail to enable VCAM_D\n");
         goto PowerEIO;
    }
    mdelay(20);
    
    //set PDN high 
    if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN_PIN,GPIO_CAMERA_CMPDN_PIN_M_GPIO)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN,GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
    mdelay(5);//dummy delay here
    
    //turn on AF
    if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
    {
      PK_DBG("[CAMERA SENSOR] Fail to enable VCAM_A2\n");
      goto PowerEIO;
    }
    mdelay(5);//dummy delay here
    //enable AF
    if(mt_set_gpio_mode(GPIO214, GPIO_MODE_00)){PK_DBG("[CAMERA SENSOR AF ] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO214,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO214,GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}

  }
  else //power off
  {
    //diasble AF
    if(mt_set_gpio_mode(GPIO214, GPIO_MODE_00)){PK_DBG("[CAMERA SENSOR AF ] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO214,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO214,GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}

    //turn off AF
    if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
    {
        PK_DBG("[CAMERA SENSOR] Fail to CAMERA_POWER_VCAM_A2 power\n");
        goto PowerEIO;
    }

    //pull main camera ov5647 PDN pin high or pull main 2 ov5648 camera PDN pin low ,5648 standby
    if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN_PIN,GPIO_CAMERA_CMPDN_PIN_M_GPIO)){PK_DBG("[CAMERA LENS] set gpio mode failed!!\n");}
    if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} 

    //turn off VDD_D
    if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name))
    {
        PK_DBG("[CAMERA SENSOR] Fail to CAMERA_POWER_VCAM_D power\n");
        goto PowerEIO;
    }
    //turn off VDD_A
    if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
    {
        PK_DBG("[CAMERA SENSOR] Fail to CAMERA_POWER_VCAM_A power\n");
        goto PowerEIO;
    }        
    //turn off VDD_IO
    if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
    {
        PK_DBG("[CAMERA SENSOR] Fail to CAMERA_POWER_VCAM_D2 power\n");
        goto PowerEIO;
    }

  }
  
  //success~
  PK_DBG_FUNC("XXXXXXXXX5648 Success");
  return 0;

PowerEIO:
  PK_DBG_FUNC("Failed!");
  return -EIO;
}
*/
static int kd_sub_1st_PowerOn(BOOL On, char* mode_name)
{
  //1st sub sensor is SENSOR_DRVNAME_OV7690_YUV
  //follow OVM7690 CameraCube Application Notes R1 31.pdf
  PK_DBG_FUNC("Start, Power: %d",On);
  if(On) //power on
  {
    PK_DBG_FUNC("XXXXXXXXX7690 Start, Power: %d",On);

//disable main cam 
    if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN_PIN,GPIO_CAMERA_CMPDN_PIN_M_GPIO)){PK_DBG("[CAMERA] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN,GPIO_OUT_ONE)){PK_DBG("[CAMERA] set gpio failed!! \n");}

    //set PDN low
    if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
    
    //turn on VDD_IO
    if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
    {
        PK_DBG("[CAMERA SENSOR] Fail to enable VCAM_D2\n");
        goto PowerEIO;
    }
    mdelay(5);//t0 >= 0ms
    //turn on VDD_A
    if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
    {
        PK_DBG("[CAMERA SENSOR] Fail to enable VCAM_A\n");
        goto PowerEIO;
    }
    mdelay(25);//t2 >= 3ms
    
  }
  else //power off
  {
    //set PDN high ,cc test begin,(if set high here ,can load 7690!!!!) add 5.22
    if(mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN,GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //cc test end
     
    //turn off VDD_A
    if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A, mode_name))
    {
        PK_DBG("[CAMERA SENSOR] Fail to enable VCAM_A\n");
        goto PowerEIO;
    }
    //turn off VDD_IO
    if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2, mode_name))
    {
        PK_DBG("[CAMERA SENSOR] Fail to enable VCAM_D2\n");
        goto PowerEIO;
    }    
	
  }    
  //success~
  PK_DBG_FUNC("XXXXX7690 Success");
  return 0;

PowerEIO:
  PK_DBG_FUNC("Failed!");
  return -EIO;
}

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{

  int retval;
  if((KD_MAIN_SENSOR_1ST != NULL) && (0 == strcmp(KD_MAIN_SENSOR_1ST,currSensorName)))
  {
    retval = kd_main_1st_PowerOn(On,mode_name);
  }
/*  else if((KD_MAIN_SENSOR_2ND != NULL) && (0 == strcmp(KD_MAIN_SENSOR_2ND,currSensorName)))
  {
    retval = kd_main_2nd_PowerOn(On,mode_name);
  }
*/
  else if((KD_SUB_SENSOR_1ST != NULL) && (0 == strcmp(KD_SUB_SENSOR_1ST,currSensorName)))
  {
    retval = kd_sub_1st_PowerOn(On,mode_name);
  }

  return retval;

}
// PR 461862 rui.cui@tcl.com 2013-06-01 end
EXPORT_SYMBOL(kdCISModulePowerOn);


