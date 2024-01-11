/************************************************************
 * @file   duraTOUCH.c
 * @author Qiuliang Fu <qfubox@gmail.com>
 * @date   2020-07-23
 * @brief  A kernel module for controlling a duraTOUCH dT4xx IC.
 *
 * Modified by Bryan Cribbs <bryan@pixotech.com> on 2019-10-16
 * Modified by John Mathew <jmathew@uico.com>    on 2020-08-21
 * Modified by Ben Young <ben@pixotech.com>      on 2020-10-21
 * @see http://www.uico.com/
************************************************************/
#ifndef _DURATOUCH_C_
#define _DURATOUCH_C_

#include "duraTOUCH.h"

//************************************************
// Global VAR prototype for the driver
//************************************************
static struct i2c_client *g_i2c_client;
static struct duraTOUCH_event dTevent[MAX_POINT];
static struct duraTOUCH_data *duraTouchDev = NULL;

static unsigned int irqNumber;                       // Used to share the IRQ number within this file
static uint16_t CurFingerNum;
static uint16_t ScreenXResolution;
static uint16_t ScreenYResolution;

static int gpio_penirq = 0;
module_param(gpio_penirq, int, 0644);
MODULE_PARM_DESC(gpio_penirq, "GPIO that the touch IRQ is connected to");

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend duraTOUCH_early_suspend;
static void duraTOUCH_i2c_early_suspend(struct early_suspend *h);
static void duraTOUCH_i2c_late_resume(struct early_suspend *h);
#endif

//*********************************************************************************************
// Function prototype for the custom IRQ handler function -- see below for the implementation
//*********************************************************************************************
static irq_handler_t  duraTOUCH_irq_handler(unsigned int irq, void *handle);

static int duraTOUCH_i2cWrite(uint8_t num, uint8_t *bytes)
{
	int status = 0, mytry = 0, retryLimit = 2;
        struct i2c_msg mymsg;


	if(!duraTouchDev->client->adapter) {
		printk(KERN_ALERT "Cannot get I2C Adapter\n");
		status = -1;
		return status;
	}

	mymsg.addr =  I2C_ADDR;
	mymsg.flags = 0;
	mymsg.buf = bytes;
	mymsg.len = num;

	mytry = 0;
#if 1
	while(mytry < retryLimit) {
		status = i2c_transfer(duraTouchDev->client->adapter, &mymsg, 1);
		if(status < 0)
		{
			mytry++;
			printk(KERN_ALERT "I2C Transfer Retry %d, return val: %d\n", mytry, status);
		}
		else if(status != 1) {
			mytry++;
			printk(KERN_ALERT "I2C Write Transfer %d bytes\n", status);
		}
		else break;
	}

	if(mytry >= retryLimit) {
		printk(KERN_ALERT "Cannot write data into i2C Adapter\n");
	}
#else
	status = i2c_transfer(duraTouchDev->client->adapter, &mymsg, 1);
#endif
	return status;
}


static int duraTOUCH_i2cRead(uint8_t num, uint8_t *bytes)
{
	int status = 0, mytry = 0, retryLimit = 2;

	if(!duraTouchDev->client->adapter) {
		printk(KERN_ALERT "Cannot get I2C Adapter\n");
		status = -1;
		return status;
	}

	mytry = 0;
#if 1
	while(mytry < retryLimit) {
		status = i2c_master_recv(duraTouchDev->client, bytes, num);
		if(status < 0)
		{
			mytry++;
			printk(KERN_ALERT "I2C Transfer Retry %d, return val: %d\n", mytry, status);
		}
		else if(status != num) {
			mytry++;
			printk(KERN_ALERT "I2C Transfer %d bytes\n", status);
		}
		else break;
	}

	if(mytry >= retryLimit) {
		printk(KERN_ALERT "Cannot read data from I2C Adapter\n");
	}
#else
	i2c_master_recv(duraTouchDev->client, bytes, num);
#endif
	return status;
}

static int duraTOUCH_ChipInit(struct i2c_client *client)
{
        return 0;
}


static void duraTOUCH_ISR_XY_Data(struct work_struct *work)
{
	uint8_t rdata[32], wdata[32], num, TouchDetected;
	uint16_t NumberExtraSurfaceTouches;

	// Write 0x20 into Touch IC
 	wdata[0] = 0x20;
        num = 1;
        duraTOUCH_i2cWrite(num, wdata);

	// Read 2 bytes from Touch IC
	num = 2;
        duraTOUCH_i2cRead(num, rdata);

	// Check the length of rdata[1]
	if(rdata[1] >0)
	{
            duraTOUCH_i2cRead(rdata[1]+2, rdata);
	}

	// Show the value in the LOG
        // for(num=0; num<2+rdata[1]; num++)
	//	printk("%2x ", rdata[num]);
	// printk("\n");

	// Clear the INT to the touch IC
	wdata[0] = 0x20;
        wdata[1] = 1;
        num = 2;
        duraTOUCH_i2cWrite(num, wdata);

	//****************************************
    // See D-0082B2 duarTOUCH Interface Document Table 13 (on page 8) for more details
	// [0] Command Index
	// [1] Length - 2
	// [2] Packet Sequence Number
	// [3] Reported Touch Metadata
	// [4] First Finger: X - High Byte
	// [5] First Finger: X - Low Byte
	// [6] First Finger: Y - High Byte
	// [7] First Finger: Y - Low Byte
    // [8] First Finger: X Pressure
    // [9] First Finger: Y Pressure
    // [10] Second Finger: X - High Byte
	// [11] Second Finger: X - Low Byte
	// [12] Second Finger: Y - High Byte
	// [13] Second Finger: Y - Low Byte
    // [14] Second Finger: X Pressure
    // [15] Second Finger: Y Pressure
	//****************************************
        TouchDetected = rdata[3] & 0x1;
        NumberExtraSurfaceTouches = (rdata[3] & 0x60) >> 5;
        if (TouchDetected == 0)
        {
            CurFingerNum = 0;
        }
        else if (NumberExtraSurfaceTouches == 0)
        {
            CurFingerNum = 1;
        }
        else
        {
            CurFingerNum = 2;
        }
        dTevent[0].x = (uint16_t)rdata[4];
        dTevent[0].x = (dTevent[0].x<<8) + (uint16_t)rdata[5];
        dTevent[0].y = (uint16_t)rdata[6];
        dTevent[0].y = (dTevent[0].y<<8) + (uint16_t)rdata[7];
        if (rdata[1] >= 14)
        {
            dTevent[1].x = (uint16_t)rdata[10];
            dTevent[1].x = (dTevent[1].x<<8) + (uint16_t)rdata[11];
            dTevent[1].y = (uint16_t)rdata[12];
            dTevent[1].y = (dTevent[1].y<<8) + (uint16_t)rdata[13];
        }
#if 1
	printk(KERN_INFO "%d finger(s) is found @ location:(%d, %d)\n", CurFingerNum, dTevent[0].x, dTevent[0].y);
    if (CurFingerNum == 0)
    {
        printk(KERN_INFO "Touch not detected.\n");
    }
    else if (CurFingerNum == 1)
    {
        printk(KERN_INFO "%d finger found @ location:(%d, %d)\n", CurFingerNum, dTevent[0].x, dTevent[0].y);
    }
    else
    {
        printk(KERN_INFO "%d fingers found @ location1:(%d, %d) and location2:(%d, %d)\n", CurFingerNum, dTevent[0].x, dTevent[0].y,dTevent[1].x, dTevent[1].y);
    }
#endif
}


static void duraTOUCH_ISR(struct work_struct *work)
{
        int i;

	duraTOUCH_ISR_XY_Data(work);

#if 1

        //if(dTevent[0].x >= 200 && dTevent[0].y >=200) {
	//	CurFingerNum = 0;
	//}

	for(i=0; i<MAX_POINT; i++)
	{
	    if(CurFingerNum>0)
        {
  	        input_mt_slot(duraTouchDev->dev, i);
		    input_mt_report_slot_state(duraTouchDev->dev, MT_TOOL_FINGER, true);
		    input_report_abs(duraTouchDev->dev, ABS_MT_TRACKING_ID, i);
		    input_report_abs(duraTouchDev->dev, ABS_MT_TOUCH_MAJOR, AREA_SETTING);
		    input_report_abs(duraTouchDev->dev, ABS_MT_POSITION_X, dTevent[i].x);
		    input_report_abs(duraTouchDev->dev, ABS_MT_POSITION_Y, dTevent[i].y);
		    input_report_abs(duraTouchDev->dev, ABS_MT_WIDTH_MAJOR, AREA_SETTING);
	    }
        else
        {
            input_mt_slot(duraTouchDev->dev, i);
		    input_report_abs(duraTouchDev->dev, ABS_MT_TOUCH_MAJOR, -1);
            input_mt_report_slot_state(duraTouchDev->dev, MT_TOOL_FINGER, false);
	    }

		input_mt_sync(duraTouchDev->dev);
	    //printk(KERN_INFO "INPUT system reported @ %d", i);
	}
	input_sync(duraTouchDev->dev);
#endif

	// Enable the IRQ
	enable_irq(irqNumber);
}

static void duraTOUCH_get_controller_params(void)
{
	uint8_t buf[19];

	buf[0] = 0x00; // write register address
	buf[1] = 0x85; // sytem information command
	buf[2] = 0x01; // parm(0) user system info

	duraTOUCH_i2cWrite(3, buf);

	buf[0] = 0x20; // response register address
	duraTOUCH_i2cWrite(1, buf);
	duraTOUCH_i2cRead(19, buf);

	ScreenXResolution = ((uint16_t)buf[13] << 8) | (uint16_t)buf[14];
	ScreenYResolution = ((uint16_t)buf[15] << 8) | (uint16_t)buf[16];
}

static void duraTOUCH_echoTest(struct i2c_client *client)
{
         uint8_t rdata[32], wdata[32], num;

	// Write ECHO command into Touch IC
 	wdata[0] = 0x00; // Write Register Address
 	wdata[1] = 0x8A; // ECHO command
 	wdata[2] = 0x55; // Data0
 	wdata[3] = 0xAA; // Data1
 	wdata[4] = 0xFF; // Data2
 	wdata[5] = 0xE7; // Data3
        num = 6;
        duraTOUCH_i2cWrite(num, wdata);

	msleep(6);
 	wdata[0] = 0x20; // Read Register Address
        num = 1;
        duraTOUCH_i2cWrite(num, wdata);

	// Read 6 bytes from Touch IC
	num = 6;
        duraTOUCH_i2cRead(num, rdata);
	printk("Read Data back from dT101: ");
	for(num=0; num<6; num++) printk("%x ", rdata[num]);
	printk("\n");
}

static int duraTOUCH_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int status = 0;

    //******************************************************
    // Call i2c_set_clientdata();
    //******************************************************
    g_i2c_client = client;
	duraTouchDev->client = g_i2c_client;
    duraTouchDev->last_point_num = 0;
	i2c_set_clientdata(client, duraTouchDev);

    //******************************************************
    // Do what a Chip should do @ its Reset/Init procedure
    //******************************************************
    duraTOUCH_ChipInit(client);

    //******************************************************
    // Check I2C channel works or NOT ?
    //******************************************************
    if(!i2c_check_functionality(duraTouchDev->client->adapter, I2C_FUNC_I2C))
    {
	    dev_err(&duraTouchDev->client->dev, "I2C functionality not supported\n");
	    printk(KERN_ALERT "CAN NOT check i2c function properly\n");
	    return -ENODEV;
    }
    printk(KERN_INFO "i2c_probe is done correctly\n");


    // if( (i2c_detect(duraTouchDev->client->adapter, duraTOUCH_i2c_driver) < 0) {
    // 	    printk(KERN_ALERT "I2C Detection is fail\n");
    // }

    // duraTOUCH_echoTest(g_i2c_client);

    //******************************************************
    // 1. ISR is mapped by calling INIT_WORK;
    // 2. creat a working queue for the Interrupt;
    //******************************************************
    INIT_WORK(&duraTouchDev->duraTOUCH_event_work, duraTOUCH_ISR);
    duraTouchDev->workqueue = create_singlethread_workqueue("duraTOUCH_wq");
    if(duraTouchDev->workqueue == NULL)
    {
	    printk(KERN_ALERT "CAN NOT create work queue properly\n");
            return -ESRCH;
    }
    printk(KERN_INFO "work queue is created correctly\n");

#if 1
    //******************************************************
    // 1. Setup a INPUT system
    // 2. Setup parameter for the input system
    //******************************************************
    duraTouchDev->dev = input_allocate_device();
    if(duraTouchDev->dev == NULL)
    {
	    destroy_workqueue(duraTouchDev->workqueue);
	    printk(KERN_ALERT "CAN NOT get the input device\n");
	    return -ENOMEM;
    }
    printk(KERN_INFO "Input device is allocated!\n");

    __set_bit(EV_ABS, duraTouchDev->dev->evbit); // ABS Axial Location
    __set_bit(EV_KEY, duraTouchDev->dev->evbit); // Event KEY
    __set_bit(EV_REP, duraTouchDev->dev->evbit); // Repeat
    __set_bit(INPUT_PROP_DIRECT, duraTouchDev->dev->propbit); // TouchScreen Device

#ifdef ANDROID_FUNC
    input_mt_init_slots(duraTouchDev->dev, MAX_POINT);
#else
    input_mt_init_slots(duraTouchDev->dev, MAX_POINT, 0);
#endif

    set_bit(ABS_MT_TOUCH_MAJOR, duraTouchDev->dev->absbit);
    set_bit(ABS_MT_WIDTH_MAJOR, duraTouchDev->dev->absbit);
    set_bit(ABS_MT_POSITION_X, duraTouchDev->dev->absbit);
    set_bit(ABS_MT_POSITION_Y, duraTouchDev->dev->absbit);
    set_bit(ABS_MT_TRACKING_ID, duraTouchDev->dev->absbit);

	duraTOUCH_get_controller_params();

    input_set_abs_params(duraTouchDev->dev, ABS_MT_POSITION_X, 0, ScreenXResolution, 0, 0);
    input_set_abs_params(duraTouchDev->dev, ABS_MT_POSITION_Y, 0, ScreenYResolution, 0, 0);
    input_set_abs_params(duraTouchDev->dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
    input_set_abs_params(duraTouchDev->dev, ABS_MT_WIDTH_MAJOR, 0, ScreenXResolution, 0, 0);
    input_set_abs_params(duraTouchDev->dev, ABS_MT_WIDTH_MINOR, 0, ScreenYResolution, 0, 0);

    duraTouchDev->dev->name = "duraTOUCHic";
    duraTouchDev->dev->phys = "input/duraTOUCH";
    duraTouchDev->dev->id.bustype = BUS_I2C;
    duraTouchDev->dev->id.vendor = 0xDEAD;
    duraTouchDev->dev->id.product = 0xBEEF;
    duraTouchDev->dev->id.version = 0x0101;
    duraTouchDev->dev->dev.parent = &client->dev;

    status = input_register_device(duraTouchDev->dev);
    if(status < 0) {
	    printk(KERN_ALERT "CANNOT regsiter input device");
	    input_free_device(duraTouchDev->dev);
	    destroy_workqueue(duraTouchDev->workqueue);
	    return -ENOMEM;
    }
    printk(KERN_INFO "Input device is registered !\n");
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
    duraTOUCH_early_suspend.suspend = duraTOUCH_i2c_early_suspend;
    duraTOUCH_early_suspend.resume = duraTOUCH_i2c_late_resume;
    duraTOUCH_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN+1;
    register_early_suspend(&duraTOUCH_early_suspend);
    printk(KERN_INFO "early suspend is registered!\n");
#endif
    return status;
}


static int duraTOUCH_i2c_remove(struct i2c_client *client)
{

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&duraTOUCH_early_suspend);
#endif

	input_unregister_device(duraTouchDev->dev);
	input_free_device(duraTouchDev->dev);
	destroy_workqueue(duraTouchDev->workqueue);
	duraTouchDev->client = NULL;
	//if(duraTouchDev) kfree(duraTouchDev);
	return 0;
}

//*****************************************************
// 1. early_suspend is pointed to suspend finally
// 2. late_resume is pointed to resume finally
//
//     Whatever we define CONFIG_HAS_EARLYSUSPEND,
// we need the following function for sure.
//*****************************************************

#if 0
static int duraTOUCH_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    int status = 0;

	return status;
}

static int duraTOUCH_i2c_resume(struct i2c_client *client)
{
    int status = 0;

	return status;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void duraTOUCH_i2c_early_suspend(struct early_suspend *h)
{
	duraTOUCH_i2c_suspend(duraTouchDev->client, PMSG_SUSPEND);
}

static void duraTOUCH_i2c_late_resume(struct early_suspend *h)
{
	duraTOUCH_i2c_resume(duraTouchDev->client);
}
#endif

static const struct i2c_device_id duraTOUCH_i2c_id[] = {
        {"duraTOUCHic", 0}, // I2CSEL},
	{},
};

static struct i2c_driver duraTOUCH_i2c_driver = {
	.probe = duraTOUCH_i2c_probe,
	.remove = duraTOUCH_i2c_remove,
	.id_table = duraTOUCH_i2c_id,
#ifndef CONFIG_HAS_EARLYSUSPEND
	//.suspend = duraTOUCH_i2c_suspend,
	//.resume = duraTOUCH_i2c_resume,
#endif
	.driver = {
		.name = "duraTOUCHic",
		.owner = THIS_MODULE,
	},
};

/** @brief The Driver initialization function
 *  The static keyword restricts the visibility of the function to within this C file.
 *  The __init macro means that for a built-in driver (not a LKM)
 *  the function is only used at initialization time and that it can be discarded
 *  and its memory freed up after that point. In this example:
 *  this function sets up the GPIOs and the IRQ
 *  @return returns 0 if successful
 */
static int __init duraTOUCH_init(void){
   int result = 0;

   printk(KERN_INFO "UICO duraTOUCH: Init the UICO_duraTOUCH driver\n");

   //************************************************************************
   // Extract kernel memory for duraTouchDev
   //************************************************************************
   duraTouchDev = kmalloc(sizeof(*duraTouchDev), GFP_KERNEL);
   if(!duraTouchDev) {
           printk(KERN_ALERT "duraTouchDev cannot get memory in kernel\n");
	   return PTR_ERR(duraTouchDev);
   }
   printk(KERN_INFO "duraTouchDev got memory for duraTouchDev in kernel\n");

   //************************************************************************
   // Add I2C driver with probe function mainly
   //************************************************************************
   result = i2c_add_driver(&duraTOUCH_i2c_driver);
   if(result<0) {
	   printk(KERN_ALERT "Unable add I2C driver properly\n");
	   kfree(duraTouchDev);
	   return result;
   }
   printk(KERN_INFO "duraTouchDev add I2C driver correctly\n");

   //*********************************************************************************
   // Config the interrupt GPIO(Pin)
   // GPIO numbers and IRQ numbers are not the same! This function performs the mapping for us
   //*********************************************************************************
   gpio_request(gpio_penirq, "sysfs");    // Set up the gpio_penirq
   gpio_direction_input(gpio_penirq);     // Set the button GPIO to be an input
   gpio_export(gpio_penirq, false);       // Causes gpio60 to appear in /sys/class/gpio
			                    // the bool argument prevents the direction from being changed
   gpio_set_debounce(gpio_penirq, 5);     // Debounce the button with a delay of 200ms(5ms now)
   //printk(KERN_INFO "UICO duraTOUCH: The INT pin is currently: %d\n", gpio_get_value(gpio_penirq));
   irqNumber = gpio_to_irq(gpio_penirq);
   printk(KERN_INFO "UICO duraTOUCH: The INT-Pin is mapped to IRQ: %d\n", irqNumber);

   //*********************************************************************************
   // This next call requests an interrupt line
   //*********************************************************************************
   result = request_irq(irqNumber,                             // The interrupt number requested
                        (irq_handler_t) duraTOUCH_irq_handler, // The pointer to the handler function below
                        IRQF_TRIGGER_FALLING,                  // Interrupt on rising edge (button press, not release)
                        "duraTOUCH_ISR_handler",               // Used in /proc/interrupts to identify the owner
                        NULL);                                 // The *dev_id for shared interrupt lines, NULL is okay
   printk(KERN_INFO "UICO duraTOUCH: The interrupt request result is: %d\n", result);
   return result;
}

/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required. Used to release the
 *  GPIOs and display cleanup messages.
 */
static void __exit duraTOUCH_exit(void) {

   free_irq(irqNumber, NULL);               // Free the IRQ number, no *dev_id required in this case
   gpio_unexport(gpio_penirq);            // Unexport the Button GPIO
   gpio_free(gpio_penirq);                // Free the Button GPIO

   i2c_del_driver(&duraTOUCH_i2c_driver);
   kfree(duraTouchDev);

   printk(KERN_INFO "UICO duraTOUCH: Goodbye from the duraTOUCH driver!\n\n");
}

/** @brief The GPIO IRQ Handler function
 *  This function is a custom interrupt handler that is attached to the GPIO above. The same interrupt
 *  handler cannot be invoked concurrently as the interrupt line is masked out until the function is complete.
 *  This function is static as it should not be invoked directly from outside of this file.
 *  @param irq       the IRQ number that is associated with the GPIO -- useful for logging.
 *  @param dev_id    the *dev_id that is provided -- can be used to identify which device caused the interrupt
 *                   Not used in this example as NULL is passed.
 *  @param regs      h/w specific register values -- only really ever used for debugging.
 *  return returns   IRQ_HANDLED if successful -- should return IRQ_NONE otherwise.
 */
static irq_handler_t duraTOUCH_irq_handler(unsigned int irq, void *handle)
{
   disable_irq_nosync(irq);
   queue_work(duraTouchDev->workqueue, &duraTouchDev->duraTOUCH_event_work);

   return (irq_handler_t) IRQ_HANDLED;      // Announce that the IRQ has been handled correctly
}

///*****************************************************************************
/// This next calls are  mandatory -- they identify the initialization function
/// and the cleanup function (as above).
///*****************************************************************************
module_init(duraTOUCH_init);
module_exit(duraTOUCH_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("UICO");
MODULE_DESCRIPTION("UICO duraTOUCH dT4xx Linux driver");
MODULE_VERSION("0.6");

#endif
