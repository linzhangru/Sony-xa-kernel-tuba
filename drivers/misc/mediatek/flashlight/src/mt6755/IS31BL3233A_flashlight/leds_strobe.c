#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>

#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
//CEI comments start

#ifdef FLASHLIGHT_TIMING_FIX
#include <linux/kthread.h>
#endif
//CEI comments end
//CEI comments start

#include <mt-plat/mt_gpio.h>
#include <mt-plat/mt_gpio_core.h>
#include <mach/gpio_const.h>



/*
// flash current vs index
    0       1      2       3    4       5      6       7    8       9     10
93.74  140.63  187.5  281.25  375  468.75  562.5  656.25  750  843.75  937.5
     11    12       13      14       15    16
1031.25  1125  1218.75  1312.5  1406.25  1500mA
*/
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)


/*#define DEBUG_LEDS_STROBE*/
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */
//CEI comments tart

static DEFINE_SPINLOCK(g_strobe);
//CEI comments end
//CEI comments start

static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;
//CEI comments end

//CEI comments tart

//CEI comments start

static int g_led_state = 0;
int g_duty=-1;
//CEI comments start

#ifdef FLASHLIGHT_TIMING_FIX
struct task_struct *flashlight_thread_handle = NULL;
static unsigned long flags;
extern int g_light_test;
int g_idle_cpu = -1;
#else
static struct hrtimer g_timeOutTimer;
static struct work_struct workTimeOut;
#endif
//CEI comments end
//CEI comments end
//CEI comments end
static int g_timeOutTimeMs=0;

static DEFINE_MUTEX(g_strobeSem);

/* #define FLASH_GPIO_ENF GPIO12 */
/* #define FLASH_GPIO_ENT GPIO13 */
//CEI comments start

//CEI comments start

//#define GPIO_ENF GPIO_FLASH_LED_EN
//#define GPIO_ENM GPIO_TORCH_EN
#define GPIO_ENF 8
#define GPIO_ENM 9
//CEI comments end
//CEI comments end

/*****************************************************************************
Functions
*****************************************************************************/
//CEI comments start

#ifdef FLASHLIGHT_TIMING_FIX
int flashlight_kthread(void *x);
#else
static void work_timeOutFunc(struct work_struct *data);
#endif
//CEI comments end
int FL_preOn(void)
{
    //CEI comments tart
    
    //CEI comments start
    
    //CEI comments start
    
    printk("[IS31BL3233A]g_led_state=%d, g_duty=%d\n", g_led_state, g_duty);
    return 0;
	//CEI comments end
    //CEI comments end
    //CEI comments end
}

int FL_Enable(void)
{
//CEI comments start

#ifdef FLASHLIGHT_TIMING_FIX
	int i = 0;
    //CEI comments start
    
    if (g_duty > 0)
        g_led_state = 2;
    else
        g_led_state = 1;
    //CEI comments end

    printk("[IS31BL3233A]FL_Enable g_led_state=%d, g_timeOutTimeMs=%d\n", g_led_state, g_timeOutTimeMs);

    if (g_led_state == 2) {
		for (i=0; i<8; i++) {
			if (idle_cpu(i)) {
				g_idle_cpu = i;
			}
		}
    //CEI comments start
    
        wake_up_process(flashlight_thread_handle);
   	} else if (g_led_state == 1) {
   	    mt_set_gpio_out(GPIO_ENM, GPIO_OUT_ONE);
    //CEI comments end
   	}
#else
    //CEI comments tart
    
    //CEI comments start
    
    //CEI comments start
    
    //Move torch enable code here to match calling procedure.

	ktime_t ktime;

    //CEI comments start
    
    //CEI comments start
    
    //CEI comments start
    
    if (g_duty > 0)
        g_led_state = 2;
    else
        g_led_state = 1;
    //CEI comments end
	
    printk("[IS31BL3233A]FL_Enable g_led_state=%d, g_timeOutTimeMs=%d\n", g_led_state, g_timeOutTimeMs);

    //CEI comments end
    //CEI comments end
	ktime = ktime_set(0, g_timeOutTimeMs * 1000);
    hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);

    printk("[IS31BL3233A]FL_Enable g_led_state=%d, g_duty=%d\n", g_led_state, g_duty);
    //CEI comments end
    //CEI comments end
    //CEI comments end
#endif
//CEI comments end
	return 0;
}

int FL_Disable(void)
{
    //CEI comments tart
    
    if (g_led_state == 1) {
        mt_set_gpio_out(GPIO_ENM, GPIO_OUT_ZERO);		
	}
    g_led_state = 0;
    g_duty = -1;
//CEI comments start

#ifndef FLASHLIGHT_TIMING_FIX
    if (g_timeOutTimeMs != 0) {
        hrtimer_cancel(&g_timeOutTimer);
        g_timeOutTimeMs = 0;
	}
#endif
//CEI comments end
    //CEI comments end
    //CEI comments start
    
    printk("[IS31BL3233A]FL_Disable g_led_state=%d, g_duty=%d\n", g_led_state, g_duty);
    //CEI comments end

	return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
    //CEI comments tart
    
    //CEI comments start
    
    printk("[IS31BL3233A]duty=%u, g_duty=%d\n", duty, g_duty);

	if (duty == 0) {
		g_duty = duty;
	} else if (duty > 0) {
	    g_duty = (duty > 6) ? 6 : duty;
}
	printk("[IS31BL3233A]g_duty=%d, g_led_state=%d\n", g_duty, g_led_state);
	//CEI comments end
    //CEI comments end
	return 0;
}


int FL_Init(void)
{
//CEI comments start

#ifdef FLASHLIGHT_TIMING_FIX    
	int err;
#endif
//CEI comments end
    //int i=0;
    //CEI comments start
    
    if(mt_set_gpio_mode(GPIO_ENF,GPIO_MODE_00)){printk("[IS31BL3233A _flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_ENF,GPIO_DIR_OUT)){printk("[IS31BL3233A _flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ZERO)){printk("[IS31BL3233A _flashlight] set gpio failed!! \n");}

    if(mt_set_gpio_mode(GPIO_ENM,GPIO_MODE_00)){printk("[IS31BL3233A _flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_ENM,GPIO_DIR_OUT)){printk("[IS31BL3233A _flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_ENM,GPIO_OUT_ZERO)){printk("[IS31BL3233A _flashlight] set gpio failed!! \n");}
//CEI comments start

#ifdef FLASHLIGHT_TIMING_FIX
	if (flashlight_thread_handle == NULL)
	    flashlight_thread_handle = kthread_create(flashlight_kthread, (void *)NULL, "flashlight_thread");
	if (IS_ERR(flashlight_thread_handle)) {
        printk("[IS31BL3233A]FL_Enable Unable to start flashlight kernel thread./n");  
        err = PTR_ERR(flashlight_thread_handle);    
        flashlight_thread_handle = NULL;  
        return err;  
	}
#endif
//CEI comments end
    printk("[IS31BL3233A]FL_Init line=%d\n",__LINE__);
    //CEI comments end
	return 0;
}


int FL_Uninit(void)
{
    mt_set_gpio_out(GPIO_ENF, GPIO_OUT_ZERO);
    mt_set_gpio_out(GPIO_ENM, GPIO_OUT_ZERO);
//CEI comments start

#ifdef FLASHLIGHT_TIMING_FIX
	if (flashlight_thread_handle) {
	    kthread_stop(flashlight_thread_handle);
	    flashlight_thread_handle = NULL;
	}
#endif
//CEI comments end
    //CEI comments start
    
	printk("[IS31BL3233A]FL_Uninit line=%d\n", __LINE__);
    //CEI comments end
	return 0;
}

int FL_hasLowPowerDetect(void)
{
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/
//CEI comments start

#ifdef FLASHLIGHT_TIMING_FIX
int flashlight_kthread(void *x)
{
	unsigned int cycle = 0, hightime = 0, lowtime = 0;
    //CEI comments start
    
	int i = 0, ret = 0, old_idle_cpu = -1;
    //CEI comments end

    while (1) {
		set_current_state(TASK_UNINTERRUPTIBLE);
        if(kthread_should_stop()) 
			break;
		
        if (g_led_state == 2) {
            if (g_light_test) {
                cycle = 500 * 1000 / 200;		
                hightime = (40 + g_duty * 10) << 1;
                lowtime = 200 - hightime;
				g_led_state = 0;
                g_light_test = 0;
            } else {            
                if (g_idle_cpu != -1 && g_idle_cpu != old_idle_cpu) {
                    ret = set_cpus_allowed(current, cpumask_of_cpu(g_idle_cpu));
                    //printk("[IS31BL3233A]%s: set_cpus_allowed() g_idle_cpu=%d, return %d\n", __func__, g_idle_cpu, ret);
                    if (ret != 0) {
                        for (i=0; i<8; i++) {
                            if (idle_cpu(i)) {
                                g_idle_cpu = i;
                                ret = set_cpus_allowed(current, cpumask_of_cpu(g_idle_cpu));
                                //printk("[IS31BL3233A]%s: set_cpus_allowed() g_idle_cpu=%d, return %d\n", __func__, g_idle_cpu, ret);
                                if (ret == 0) {
                                    old_idle_cpu = g_idle_cpu;
                                    break;
                                }
                            }
                        }
                    } else {
                        old_idle_cpu = g_idle_cpu;
                    }
                }

                cycle = 250 * 1000 / 200;		
                hightime = (40 + g_duty * 10) << 1;
                lowtime = 200 - hightime;
            }
            spin_lock_irqsave(&g_strobe, flags);		
            for (i = 0; i < cycle; i++) {		
                mt_set_gpio_out(GPIO_ENF, GPIO_OUT_ONE);
                udelay(hightime);
                mt_set_gpio_out(GPIO_ENF, GPIO_OUT_ZERO);
                udelay(lowtime);
            }
            spin_unlock_irqrestore(&g_strobe, flags);		
            //printk("[IS31BL3233A]%s: flash\n", __func__);
        //CEI comments start
        
        //} else if (g_led_state == 1 && torch_set == 0) {	        
        //    mt_set_gpio_out(GPIO_ENM, GPIO_OUT_ONE);
        //    torch_set = 1;
        //    printk("[IS31BL3233A]%s: torch\n", __func__);
        //} else {
        //CEI comments end
       	} else if (g_led_state == 0) {
             g_idle_cpu = -1;
             schedule_timeout(HZ);
        }
    }
    return 0;
}
#else
static void work_timeOutFunc(struct work_struct *data)
{
    //CEI comments tart
    
    unsigned int cycle = 0, hightime = 0, lowtime = 0;
    int i = 0;
    if (g_led_state == 2) {
        cycle = 500 * 1000 / 200;		
        hightime = (40 + g_duty * 10) << 1;
        lowtime = 200 - hightime;
        spin_lock_irq(&g_strobe);
        //CEI comments start
        
        //CEI comments start
        
        for (i = 0; i < cycle && g_led_state == 2; i++) {
        //CEI comments end
        //CEI comments end
            mt_set_gpio_out(GPIO_ENF, GPIO_OUT_ONE);
            udelay(hightime);
            mt_set_gpio_out(GPIO_ENF, GPIO_OUT_ZERO);
            udelay(lowtime);
	    }
	    spin_unlock_irq(&g_strobe);
    } else if (g_led_state == 1) {
        mt_set_gpio_out(GPIO_ENM, GPIO_OUT_ONE);
    }
    //CEI comments end
}

enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}
void timerInit(void)
{
	static int init_flag;

	if (init_flag == 0) {
		init_flag = 1;
	    INIT_WORK(&workTimeOut, work_timeOutFunc);
	    g_timeOutTimeMs = 1000;
	    hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	    g_timeOutTimer.function = ledTimeOutCallback;
    }
}
#endif
//CEI comments end
static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
    //CEI comments start
    
    //For FLASH_IOC_GET_PRE_ON_TIME_MS
    int temp;
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;	

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
	printk("[IS31BL3233A]constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		printk("[IS31BL3233A]FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		printk("[IS31BL3233A]FLASH_IOC_SET_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		printk("[IS31BL3233A]FLASH_IOC_SET_STEP: %d\n", (int)arg);
		break;

    case FLASH_IOC_PRE_ON:
        printk("[IS31BL3233A]FLASH_IOC_PRE_ON: %d\n", (int)arg);
        FL_preOn();
        break;
    //CEI comments tart
    
    case FLASH_IOC_GET_PRE_ON_TIME_MS_DUTY:
		printk("[IS31BL3233A]FLASH_IOC_GET_PRE_ON_TIME_MS_DUTY: %d\n",(int)arg);
        i4RetValue= g_duty;
		
		break;
    //Add this case to let upper layer know it could send FLASH_IOC_PRE_ON command
    case FLASH_IOC_GET_PRE_ON_TIME_MS:
        printk("[IS31BL3233A]FLASH_IOC_GET_PRE_ON_TIME_MS: %d\n",(int)arg);
        i4RetValue= g_timeOutTimeMs;		
    //CEI comments end		
		break;

	case FLASH_IOC_SET_ONOFF:
		printk("[IS31BL3233A]FLASH_IOC_SET_ONOFF: %d\n", (int)arg);
		if (arg == 1) {
			FL_Enable();
		} else {
            if (g_led_state) {				
			FL_Disable();
           	}
		}
		break;
       
    case FLASH_IOC_HAS_LOW_POWER_DETECT:
   		printk("[IS31BL3233A]FLASH_IOC_HAS_LOW_POWER_DETECT");
   		temp=FL_hasLowPowerDetect();
   		if (copy_to_user((void __user *) arg , (void*)&temp , 4)) {
            printk("[IS31BL3233A]ioctl copy to user failed\n");
            return -1;
		}
		break;

	default:
		printk("[IS31BL3233A]No such command\n");
		i4RetValue = -EPERM;
		break;
	}
    //CEI comments end
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;
    //CEI comments start
    
	printk("[IS31BL3233A]constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
//CEI comments start

#ifndef FLASHLIGHT_TIMING_FIX
		timerInit();
#endif
//CEI comments end
	}
	printk("[IS31BL3233A]constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		printk("[IS31BL3233A]busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	printk("[IS31BL3233A]constant_flashlight_open line=%d\n", __LINE__);
    //CEI comments end
	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    //CEI comments start
    
	printk("[IS31BL3233A]constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	printk("[IS31BL3233A]Done\n");
    //CEI comments end

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
//CEI comments end
