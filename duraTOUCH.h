/************************************************************
 * @file   duraTOUCH.h
 * @author Qiuliang Fu <qfubox@gmail.com>
 * @date   2020-07-23
 * @brief  A kernel module for controlling a duraTOUCH dT4xx IC.
 *
 * Modified by Bryan Cribbs <bryan@pixotech.com> on 2019-10-16
 * Modified by John Mathew  <jmathew@uico.com>   on 2020-08-21
 * @see http://www.uico.com/
************************************************************/
#ifndef _DURATOUCH_H_
#define _DURATOUCH_H_

#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/input/mt.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <linux/slab.h>

#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/interrupt.h>            // Required for the IRQ code
#include <linux/proc_fs.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/gpio.h>                 // Required for the GPIO functions
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/input/mt.h>
#include <linux/fs.h>
#include <asm/uaccess.h>



#define I2C_ADDR               0x48   // DO NOT CHANGE! The Touch IC firmware is hardcoded to use this address

#define MAX_POINT              2      // Max number of touches that can be reported by the duraTOUCH IC
#define GPIO_NUM_FOR_IRQ_PIN   8      // GPIO number of GPIO Pin used for Touch IRQ



// used with input_set_abs_params() function in input/input.c to specify the "max" argument assoicated with the ABS_MT_TOUCH_MAJOR axis
// do not change this unless you know exactly what you are doing!
#define PRESS_MAX              1


// used with input_report_abs() function in input/input.c to to specify the "value" argument
// associated with the ABS_MT_TOUCH_MAJOR code
// and with the ABS_MT_WIDTH_MAJOR code
// do not change this unless you know exactly what you are doing!
#define AREA_SETTING           1


//*********************************************************************************
// GPIO numbers and IRQ numbers are not the same!
// The gpio_to_irq function performs the mapping.
// irqNumber = gpio_to_irq(GPIO_NUM_FOR_IRQ_PIN);
//*********************************************************************************


struct duraTOUCH_event {
	uint16_t   flag;
	uint16_t   x;
	uint16_t   y;
	uint16_t   pressure;
	uint16_t   w;
};

struct duraTOUCH_data {
	struct i2c_client *client;
	struct input_dev  *dev;
	int               reset_gpio;
	int               touch_en_gpio;
	int               last_point_num;

	struct workqueue_struct *workqueue;
	struct work_struct duraTOUCH_event_work;
};





#endif
