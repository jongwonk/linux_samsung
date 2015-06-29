/*
 * linux/drivers/input/touchscreen/aesop-v210-ts.c
 *
 * aESOP S5PV210 platform Touch Screen Driver
 *
 * Copyright (C) 2010 aESOP Embedded Forum (http://www.aesop.or.kr)
 * Written by Jhoonkim <jhoon_kim@nate.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 */
 
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>

#ifdef CONFIG_OF
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#endif
#include <asm/io.h>
#include <asm/irq.h>

#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <asm/delay.h>

//#include <mach/gpio-bank.h>

#define DEBUG 1

/* General Touch Screen Information */
#define HIDIS_VERSION		0x0101
#define HIDIS_VENDOR		0xDEAD
#define HIDIS_PRODUCT		0xBEEF

#define IRQ_EINT11 			11+32 // datasheet pg#609
#define HIDIS_IRQ_NUM		IRQ_EINT11
#define HIDIS_X_RESOLUTION	480
#define HIDIS_Y_RESOLUTION	800
#define TOUCH_READ_TIME		( (HZ/100) )  // 10ms

//KJW #define S5PV210_GPH1DAT S5P_VA_GPIO+(0xC24)

/* Defined for use k_thread */ 
static wait_queue_head_t idle_wait;
static int wake_up_work = 0;

static struct task_struct *kidle_task;

/* Touch Screen Private Data */
/*
static struct hidis_priv
{
	struct delayed_work	task_work;
	struct i2c_client 	*i2c_client;
	struct i2c_driver 	hidis_i2c_driver;

	struct device *dev;
	struct input_dev *input;
	struct clk *clock;
	void __iomem *io;
	unsigned long xp;
	unsigned long yp;
	int irq_tc;
	int count;
	int shift;
	int features;

	struct timer_list	pwn_down_check_timer;

	long ts_xp_old;
	long ts_yp_old;

	unsigned char touch_irq_flag;
	unsigned char touch_ready_flag;
	unsigned int interval;
	unsigned int pen_down;

	unsigned long xp;
	unsigned long yp;
	
	spinlock_t lock;

} *p_hidis_priv;
*/

struct hidis_i2c_dev {
	struct i2c_client * client;
	struct input_dev 	*input;
	struct i2c_adapter	adapter;
	struct completion	complete;
	struct device		*dev;
	struct clk 			*clock;
	void __iomem		*pinbase;
	struct clk			*clk;
	int					mode;
	int					irq;
	u16					cmd_status;
	unsigned int 		interval;
	unsigned int 		pen_down;
	spinlock_t 			lock;
	unsigned long xp;
	unsigned long yp;	
};

#ifdef TS_ANDROID_PM_ON
#ifdef CONFIG_HAS_WAKELOCK
static void ts_early_suspend(struct early_suspend *h)
{
	ts = container_of(h, struct s3c_ts_info, early_suspend);
	hidis_i2c_suspend(NULL, PMSG_SUSPEND); // platform_device is now used
}
static void ts_late_resume(struct early_suspend *h)
{
	ts = container_of(h, struct s3c_ts_info, early_suspend);
	hidis_i2c_resume(NULL); // platform_device is now used
}
#endif
#endif //TS_ANDROID_PM_ON

#ifdef CONFIG_HAS_WAKELOCK
extern suspend_state_t get_suspend_state(void);
#endif

/* Touch Screen Read Data Position 
 *
 * Protocol Spec (7bit I2C Address).
 * X-Position MSB : LSB 4bit (0x22)
 * X-Position LSB : LSB 8bit (0x20)
 * Y-Position MSB : MSB 4bit (0x21)
 * Y-Position LSB : LSB 8bit (0x22)
 *
 */
static void hidis_i2c_read_data_position(void *data)
{
	long i,j;
	unsigned char x_pos_lsb, x_y_pos_msb, y_pos_lsb;
	unsigned char x_pos_msb, y_pos_msb;
	
	struct hidis_i2c_dev *i2c_dev = (struct hidis_i2c_dev *)data;	
	struct i2c_client *client = (struct i2c_client *)(i2c_dev->client);
		
	x_pos_lsb = i2c_smbus_read_byte_data(client, 0x20);
	dev_dbg(&client->dev, "%s : Check hidis ts 0x20 = %x \n", __func__, x_pos_lsb);

	x_y_pos_msb = i2c_smbus_read_byte_data(client, 0x21);
	dev_dbg(&client->dev, "%s : Check hidis ts 0x21 = %x \n", __func__, x_y_pos_msb);

	y_pos_lsb = i2c_smbus_read_byte_data(client, 0x22);
	dev_dbg(&client->dev, "%s : Check hidis ts 0x22 = %x \n", __func__, y_pos_lsb);

	x_pos_msb = (x_y_pos_msb & 0xf);
	y_pos_msb = (x_y_pos_msb >> 4) & 0xf;

	dev_dbg(&client->dev,"%s : x_pos_msb - %x, y_pos_msb - %x\n", __func__, x_pos_msb, y_pos_msb);

	i2c_dev->xp = (unsigned long)((x_pos_msb << 8) | (x_pos_lsb & 0x00ff));
	i2c_dev->yp = (unsigned long)((y_pos_msb << 8) | (y_pos_lsb & 0x00ff));
	
	i = (x_pos_msb << 8) | (x_pos_lsb & 0xff);
        j = (y_pos_msb << 8) | (y_pos_lsb & 0xff);

	dev_dbg(&client->dev, "%s : TS Pos X - %ld(%lx), TS Pos Y - %ld(%lx)\n",
			__func__, i2c_dev->xp, i2c_dev->xp, i2c_dev->yp, i2c_dev->yp);
} 

/* Touch Screen Report Data Position for Applications */ 
static void hidis_touch_report_position(void *data)
{
	struct hidis_i2c_dev *i2c_dev = (struct hidis_i2c_dev *)data;	
	
	if((i2c_dev->xp > 0) && (i2c_dev->yp > 0) &&
		(i2c_dev->xp <= HIDIS_X_RESOLUTION) && (i2c_dev->yp <= HIDIS_Y_RESOLUTION)) {
			input_report_abs(i2c_dev->input, ABS_X, i2c_dev->xp);
			input_report_abs(i2c_dev->input, ABS_Y, i2c_dev->yp);
			input_report_key(i2c_dev->input, BTN_TOUCH, 1);
			input_report_abs(i2c_dev->input, ABS_PRESSURE, 1);
			
			// printk("X:%d,Y:%d\n",hidis_i2c_dev->xp,hidis_i2c_dev->yp);
	}
	else {
		i2c_dev->xp = 0;
		i2c_dev->yp = 0;
	}
	input_sync(i2c_dev->input);
}

/* Kernel Thread, Touch Screen Read Data Position on Interrupt */
int thread_touch_report_position(void *data)
{
	unsigned long flags;
	unsigned int ret;
	
	struct hidis_i2c_dev *i2c_dev = (struct hidis_i2c_dev *)data;
		
	init_waitqueue_head(&idle_wait);
	
	//dev_info(&pdev->dev, 
	printk("%s : Touch Report thread Initialized.\n", __func__);	
	
	do {
		if(i2c_dev->pen_down) {
			ret = wait_event_interruptible_timeout(idle_wait,0,i2c_dev->interval/*TOUCH_READ_TIME*/); 
			if( ret ) {
				printk("%s : Homing routine Fatal kernel error or EXIT to run.\n", __func__);	
			}
			else {
				if(i2c_dev->pen_down)		
					hidis_i2c_read_data_position(data);
				  
				spin_lock_irqsave(&i2c_dev->lock,flags) ;
				
				if(i2c_dev->pen_down)		
					hidis_touch_report_position(i2c_dev);
				spin_unlock_irqrestore(&i2c_dev->lock,flags);	
			}				
		}
		else {
			wait_event_interruptible(idle_wait,(wake_up_work!=0));
			wake_up_work = 0;
		}
		
	} while (!kthread_should_stop());
	
	return 0;
}

/* Touch Screen Interrupt Handler */
static irqreturn_t hidis_touch_isr(int irq, void *data)
{
	unsigned int ret_val;
	struct hidis_i2c_dev *i2c_dev = (struct hidis_i2c_dev *)data;		
	
	spin_lock(&i2c_dev->lock);
	
	ret_val = readl(i2c_dev->pinbase) & 0x8;
	i2c_dev->pen_down = ret_val;

	if(i2c_dev->pen_down) {		/* Touch Pen Down */
		//dev_dbg(&pdev->dev, 
		printk("pen-down\n");
		
		wake_up_work = 1;
		wake_up_interruptible(&idle_wait);
	}
	else {							/* Touch Pen Up */
		// dev_dbg(&pdev->dev, 
		printk("pen-up\n");
		input_report_key(i2c_dev->input, BTN_TOUCH, 0);
		input_report_abs(i2c_dev->input, ABS_PRESSURE, 0);
		input_sync(i2c_dev->input);
	} 

	spin_unlock(&i2c_dev->lock);

	return IRQ_HANDLED;	
}

static int hidis_i2c_remove(struct platform_device* pdev)
{
		struct hidis_i2c_dev *i2c_dev = platform_get_drvdata(pdev);
        free_irq(i2c_dev->irq, NULL);
		
	return 0;
}

static void hidis_i2c_early_suspend(struct early_suspend *h)
{
        disable_irq(HIDIS_IRQ_NUM);
        return ;
}

static void hidis_i2c_early_resume(struct early_suspend *h)
{
        enable_irq(HIDIS_IRQ_NUM);
	return ;
}

static int hidis_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
        disable_irq(HIDIS_IRQ_NUM);
        return 0;
}

static int hidis_i2c_resume(struct i2c_client *client)
{
        enable_irq(HIDIS_IRQ_NUM);
        return 0;
}

/* Touch Screen Driver Initialize */
static int hidis_i2c_probe(struct i2c_client *client, const struct i2c_device_id *i2c_id)
{
	struct device_node *np = client->dev.of_node;
	struct hidis_i2c_dev *i2c_dev;
	struct resource irq_res;
	int ret, err;
	int interval;

	i2c_dev =  devm_kzalloc(&client->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if(!i2c_dev)
		return -ENOMEM;
	
	i2c_dev->client = client;
	i2c_dev->dev = &client->dev;

	err = of_irq_to_resource(np,0,&irq_res);
	if(err == 0)
	{
		pr_warn("Could not allocate irq\n");
		return err;
	}

	i2c_dev->irq = irq_res.start;
	i2c_dev->pinbase = ioremap(0xE0200C24,4);	

	err = of_property_read_u32(np,"interval",&interval );
	if(!err)
	{
		printk("interval: %d\n",i2c_dev->interval);
		i2c_dev->interval = msecs_to_jiffies(interval);
	}
	else
		i2c_dev->interval = msecs_to_jiffies(2);
		
	i2c_dev->input = input_allocate_device();

	if (!i2c_dev->input)
	{
		dev_err(&client->dev, "Failed to allocate input device.\n");
		err = -ENOMEM;
		
		return err;
	}

	__set_bit(EV_SYN,i2c_dev->input->evbit);
	__set_bit(EV_KEY,i2c_dev->input->evbit);
	__set_bit(EV_ABS,i2c_dev->input->evbit);
	__set_bit(BTN_TOUCH, i2c_dev->input->keybit);

	// i2c_dev->input->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	// i2c_dev->input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(i2c_dev->input, ABS_X, 0, HIDIS_X_RESOLUTION, 0, 0);
	input_set_abs_params(i2c_dev->input, ABS_Y, 0, HIDIS_Y_RESOLUTION, 0, 0);
	input_set_abs_params(i2c_dev->input, ABS_PRESSURE, 0, 1, 0, 0);


	i2c_dev->input->name = client->name;
	i2c_dev->input->id.bustype = BUS_HOST;
	i2c_dev->input->dev.parent = &client->dev;

	i2c_dev->input->id.vendor = HIDIS_VERSION;
	i2c_dev->input->id.product = HIDIS_PRODUCT;
	i2c_dev->input->id.version = HIDIS_VERSION;

	spin_lock_init(&i2c_dev->lock);
	ret = request_irq(i2c_dev->irq, hidis_touch_isr, IRQ_TYPE_EDGE_BOTH, "hidis-touch", i2c_dev);
	
	if (ret) {
		dev_err(&client->dev, "request_irq failed (IRQ_TOUCH)!\n");
		ret = -EIO;
	}

	/* All went ok, so register to the input system */
	ret = input_register_device(i2c_dev->input);

	kidle_task = kthread_run(thread_touch_report_position, i2c_dev, "kidle_timeout");
	
	if( IS_ERR(kidle_task) ) {
		dev_err(&client->dev, "HIDIS Touch Screen Driver Initialize Faled.\n");
		return -EIO;
	}
	dev_info(&client->dev, "aESOP S5PV210 Touch Screen Driver Initialized.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id hidis_dt_ids[] = {
	{ .compatible = "hidis,ts", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hidis_dt_ids);
#endif


static struct i2c_device_id hidis_ids[] = {
	{ "hidis-ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hidis_ids);

static struct i2c_driver hidis_i2c_driver = {
	.probe = hidis_i2c_probe,
	.remove = hidis_i2c_remove,
	.driver = {
		.name = "hidis-ts",
		.of_match_table = of_match_ptr(hidis_dt_ids),
		},
		.id_table = hidis_ids,
};

module_i2c_driver(hidis_i2c_driver);

MODULE_DESCRIPTION("aESOP S5PV210 Touch Driver");
MODULE_AUTHOR("JhoonKim");
MODULE_VERSION("1.2");
MODULE_LICENSE("GPL");
