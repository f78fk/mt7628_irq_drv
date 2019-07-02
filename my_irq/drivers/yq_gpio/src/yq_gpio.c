/*
 *  YQ GPIO Interrupt driver
 *
 *  Copyright (C) 2019 Liu Yu <f78fk@live.com>
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kmod.h>
#include <linux/workqueue.h>
#include <linux/skbuff.h>
#include <linux/netlink.h>
#include <linux/kobject.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio_keys.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/wait.h>

#define  YQ_BLE_CONFIG_GPIO_PIN 3

#if	0
#define DEBUG
#endif

static wait_queue_head_t yq_wq;
static uint8_t is_wake_up = 0;
#define DEVICE_NAME "yq_gpio_irq"
static unsigned int yq_gpio_minor;
static unsigned int yq_gpio_major;
static struct cdev  yq_gpio_cdev;
static dev_t ndev;
static unsigned int yq_ble_config_irq_number=0; 

struct class *yq_gpio_class = NULL;
struct device *yq_gpio_device = NULL;
static uint8_t is_region_success = 0;
static uint8_t is_free_irq = 1;
static uint8_t is_mask_interrupt  = 0; //0 Œ¥∆¡±Œ÷–∂œ 1 “—∆¡±Œ÷–∂œ

volatile unsigned long *GPIO_CTRL_0;   //--- GPIO0 to GPIO31 direction control register  0-input 1-output
volatile unsigned long *GINT_REDGE_0;  //--GPIO0 to GPIO31 rising edge interrupt enable register
volatile unsigned long *GINT_FEDGE_0;  //--GPIO0 to GPIO31 falling edge interrupt enable register
volatile unsigned long *GINT_STAT_0;  //---GPIO0 to GPIO31 interrupt status register 1-int  0 -no int
//volatile unsigned long *GINT_EDGE_0;  //---GPIO0 to GPIO31 interrupt edge status register 1-rising 0-falling



static void yq_init_gpio(void)
{
	GPIO_CTRL_0=(volatile unsigned long *)ioremap(0x10000600,4);  //--- GPIO0 to GPIO31 direction control register  0-input 1-output
	GINT_REDGE_0=(volatile unsigned long *)ioremap(0x10000650,4); //--GPIO0 to GPIO31 rising edge interrupt enable register
	GINT_FEDGE_0=(volatile unsigned long *)ioremap(0x10000660,4); //--GPIO0-31, falling edge interrupt enable register
	GINT_STAT_0=(volatile unsigned long *)ioremap(0x10000690,4);  //---GPIO0 to GPIO31 interrupt status register 1-int  0 -no int
												
	*GPIO_CTRL_0 &=~(0x1<<YQ_BLE_CONFIG_GPIO_PIN); //---bit set 0, input mode
	*GINT_REDGE_0 &=~(0x1<<YQ_BLE_CONFIG_GPIO_PIN); //--bit set 0,disable Rising Edge interrupt  
	*GINT_FEDGE_0 |= (0x1<<YQ_BLE_CONFIG_GPIO_PIN); //--bit set 0,enabled Falling Edge interrupt  
}

static void yq_gpio_unmap(void)
{
	iounmap(GPIO_CTRL_0);
	iounmap(GINT_REDGE_0);
	iounmap(GINT_FEDGE_0);
	iounmap(GINT_STAT_0);
}

static irqreturn_t yq_gpio_irq_handle(int irq, void *_bdata)
{

	if(((*GINT_STAT_0)>>YQ_BLE_CONFIG_GPIO_PIN)&0x1)  //-----confirm the Interrupt
	{
#ifdef DEBUG	
		printk("irq\n");
#endif
		is_wake_up = 1;
		wake_up(&yq_wq);

	}
	*GINT_REDGE_0 &=~(0x1<<YQ_BLE_CONFIG_GPIO_PIN); //--bit set 0,disable Rising Edge interrupt  
	*GINT_FEDGE_0 |= (0x1<<YQ_BLE_CONFIG_GPIO_PIN); //--bit set 0,enabled Falling Edge interrupt  
	if(is_mask_interrupt == 0)
	{
		disable_irq_nosync(yq_ble_config_irq_number);
		is_mask_interrupt = 1;
	}
	return IRQ_HANDLED;
}

static int yq_register_irq(void)
{
	int int_result;

	int_result=request_irq(yq_ble_config_irq_number,yq_gpio_irq_handle, IRQF_TRIGGER_FALLING,"yq_ble_config_irq",NULL);
	if(int_result)
	{
		printk("request_irq fail.\n"); 
		return int_result;
	}

	is_free_irq = 0;
#ifdef DEBUG	
	printk("request_irq success.\n"); 
#endif 
	return 0;
}


static void yq_get_irq_number(void)
{
    yq_ble_config_irq_number=gpio_to_irq(YQ_BLE_CONFIG_GPIO_PIN);  
    if(yq_ble_config_irq_number!=0)
    {
#ifdef DEBUG	    
		printk("gpio%d get IRQ= %d successfully!\n",YQ_BLE_CONFIG_GPIO_PIN,yq_ble_config_irq_number); 
#endif
    }
    else
    {    
		printk("get yq_ble_config_irq_numberber failed!\n");
    }
}

static int yq_gpio_chrdev_open(struct inode *inode,struct file *file)
{
	int ret;
#ifdef DEBUG	
  	printk("open yq_gpio major=%d,minor=%d\n",imajor(inode), iminor(inode));
#endif 
  	yq_init_gpio();
	yq_get_irq_number();
	ret = yq_register_irq();
	init_waitqueue_head(&yq_wq);

	return ret;
}


static int yq_gpio_chredv_close(struct inode *inode, struct file *file)
{
	free_irq(yq_ble_config_irq_number,NULL);
	yq_gpio_unmap();
	is_free_irq = 1;
	is_mask_interrupt = 0;
#ifdef DEBUG
	printk("free irq.\n");
#endif

	return 0;
}

static long yq_gpio_chrdev_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
#ifdef DEBUG
	printk("yq_gpio_chrdev_ioctl\n");
#endif

	int ret = 0;
	
	switch(cmd)
	{
		case 0x89ABCDEF:
			is_wake_up = 0;
			ret = wait_event_interruptible(yq_wq,is_wake_up);
			if(ret<0)
			{
#ifdef DEBUG			
				printk("wait sleep fail\n");
#endif 
				return ret;
			}
			break;
		
		default:
			ret = -1;
	}

	return ret;

}

static ssize_t yq_gpio_chrdev_write( struct file *file , const char __user *buffer,
			   size_t len , loff_t *offset )
{
#ifdef DEBUG
	printk("yq_gpio_chrdev_write\n");
#endif 
	if(len>0)
	{
		if(buffer[0]==1)
		{
			if(is_mask_interrupt == 1)
			{
				enable_irq(yq_ble_config_irq_number);
				is_mask_interrupt = 0;
			}
		}
	}
	return 0;
}

static struct file_operations yq_gpio_fops={
	.owner = THIS_MODULE,
	.open = yq_gpio_chrdev_open,
	.release = yq_gpio_chredv_close,
	.unlocked_ioctl = yq_gpio_chrdev_ioctl,
	.write   = yq_gpio_chrdev_write,
};


static int __init yq_gpio_init(void)
{
	int result;
	
	result=alloc_chrdev_region(&ndev,yq_gpio_minor,1,DEVICE_NAME);
	yq_gpio_major=MAJOR(ndev);
	if(result<0)
	{
		goto dev_reg_error;
	}

	is_region_success = 1;
	
	cdev_init(&yq_gpio_cdev,&yq_gpio_fops);
	yq_gpio_cdev.owner=THIS_MODULE;
	result=cdev_add(&yq_gpio_cdev,ndev,1);
	if(result)
	{
		goto cdev_add_error;
	}

	yq_gpio_class = class_create(THIS_MODULE,"yq_gpio_class");
	if( IS_ERR(yq_gpio_class) )
	{
		goto class_c_error;
	}

	yq_gpio_device = device_create(yq_gpio_class,NULL,ndev,NULL,"yq_gpio_dev");
	if( IS_ERR(yq_gpio_device) )
	{
		goto device_c_error;
	}

	goto init_success;


dev_reg_error:
	printk("alloc_chrdev_region failed\n"); 
	return result;

cdev_add_error:
	printk("cdev_add failed\n");
	unregister_chrdev_region(ndev, 1);
	is_region_success = 0;
	return result;
	
class_c_error:
	printk("class_create failed\n");
	cdev_del(&yq_gpio_cdev);
	unregister_chrdev_region(ndev, 1);
	is_region_success = 0;
	return PTR_ERR(yq_gpio_class);

device_c_error:
	printk("device_create failed\n");
	cdev_del(&yq_gpio_cdev);
	unregister_chrdev_region(ndev, 1);
	class_destroy(yq_gpio_class);
	is_region_success = 0;
	return PTR_ERR(yq_gpio_device);

	
init_success:
		printk("yq_gpio_init Initialization successful\n");
		return 0;


}

static void __exit yq_gpio_exit(void)
{
	if(is_region_success == 1)
	{
		cdev_del(&yq_gpio_cdev);
 		unregister_chrdev_region(ndev, 1);
		device_unregister(yq_gpio_device);
		class_destroy(yq_gpio_class);
	}

	if(is_free_irq == 0)
	{
		free_irq(yq_ble_config_irq_number,NULL);
		is_mask_interrupt = 0;
		yq_gpio_unmap();
	}
	printk("yq_gpio_exit release resources successful\n");

}

module_init(yq_gpio_init);
module_exit(yq_gpio_exit);

MODULE_AUTHOR("Liu Yu <f78fk@live.com>");
MODULE_DESCRIPTION("yq_gpio_irq driver");
MODULE_LICENSE("GPL v2");
