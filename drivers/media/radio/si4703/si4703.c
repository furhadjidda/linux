/*
 * drivers/media/radio/si4703/si4703.c
 *
 * I2C client driver for Si4703
 *
 * Copyright (c) 2017 Furhad Jidda
 * Author: Furhad Jidda<furhadjidda@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/sched/signal.h>
#include <linux/signal.h>

#include "si4703_include.h"
#include "protocol_struct.h"

#define DEBUG_KERN 0

static int major;
static char kernelBuffer[BUFFER_LENGTH];
static char kernelRcvBuffer[BUFFER_LENGTH];
static char buffer[BUFFER_LENGTH] = {};
static struct class *i2ccharClass;
static struct si4703_dev *si4703_devices;
static const unsigned short number_of_devices = 3;
struct task_struct *task;
static unsigned int irqNumber;

static DECLARE_COMPLETION(my_completion);

struct i2cState {
	struct i2c_client	*i2cClient;
	struct i2c_device_id	*i2cDeviceId;
};
static struct i2cState *state;

static int read_rds_data(void *data)
{
	int bytes = 0;

	allow_signal(SIGINT);
	while (!kthread_should_stop() && state != NULL) {

		wait_for_completion_interruptible(&my_completion);

		dev_info(&state->i2cClient->dev, "Reading RDS data\n");

		bytes = i2c_master_recv(state->i2cClient, buffer, 12);
		dev_info(&state->i2cClient->dev, "Bytes read =%d\n", bytes);

		dev_info(&state->i2cClient->dev, "RDS Group A 0x %02x %02x\n",
				buffer[4], buffer[5]);
		dev_info(&state->i2cClient->dev, "RDS Group B 0x %02x %02x\n",
				buffer[6], buffer[7]);
		dev_info(&state->i2cClient->dev, "RDS Group C 0x %02x %02x\n",
				buffer[8], buffer[9]);
		dev_info(&state->i2cClient->dev, "RDS Group D 0x %02x %02x\n",
				buffer[10], buffer[11]);
	}

	return 0;
}


static irqreturn_t irqHandler(int irq, void *dev_id)
{
	dev_info(&state->i2cClient->dev, "Calling irq handler\n");
	complete(&my_completion);

	return  IRQ_HANDLED;
}

static int si4703_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int result = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"Need I2C_FUNC_I2C functions\n");
		return -ENODEV;
	}

	state = devm_kzalloc(&client->dev, sizeof(struct i2cState), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;

	state->i2cClient = client;
	i2c_set_clientdata(client, state);

	// initializing i2c
	gpio_free(6);
	gpio_free(23);
	gpio_free(0);

	//Requesting GPIO
	gpio_request(6, "6");
	gpio_request(23, "23");
	gpio_request(0, "0");
	//Settng directions
	gpio_direction_output(0, 0);
	mdelay(500);
	gpio_direction_output(23, 0);
	mdelay(500);
	gpio_direction_output(23, 1);
	mdelay(500);
	gpio_direction_input(6);

	irqNumber = gpio_to_irq(6);// map your GPIO to an IRQ

	result = request_irq(irqNumber, //requested interrupt
				irqHandler, //pointer to handler function
				IRQF_TRIGGER_RISING, //interrupt mode flag
				"irqHandler", //used in /proc/interrupts
				0);//the *dev_id shared interrupt lines,
				//NULL is okay

	dev_dbg(dev, "\nsi4703_probe called\n");
	dev_dbg(dev, "\nChip address=%d\n", client->addr);

	task = kthread_run(&read_rds_data, NULL, "1");
	dev_dbg(dev, "Kernel Thread : %s\n", task->comm);

	return 0;
}

static int si4703_remove(struct i2c_client *client)
{
	if (!IS_ERR_OR_NULL(task)) {
		send_sig(SIGINT, task, 1);
		kthread_stop(task);
	}
	free_irq(irqNumber, 0);

	return 0;
}


static const struct i2c_device_id si4703_dev_id[] = {
		{DRIVER_NAME, 0},
		{}
};

MODULE_DEVICE_TABLE(i2c, si4703_dev_id);


static struct i2c_driver si4703_driver = {
	.probe = si4703_probe,
	.remove = si4703_remove,
	.id_table = si4703_dev_id,
	.driver = {
	.owner = THIS_MODULE,
	.name = DRIVER_NAME
	},
};

static long si4703_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return -EINVAL;
}

static int si4703_open(struct inode *inode, struct file *filp)
{
	dev_info(&state->i2cClient->dev, "Si4703_open called\n");
	return 0;
}

static int  si4703_close(struct inode *inode, struct file *filp)
{
	dev_info(&state->i2cClient->dev, "Si4703_close called\n");
	return 0;
}

ssize_t si4703_read(struct file *file, char __user *buff,
			size_t len, loff_t *offset)
{
	int bytes = 0;
	int i = 0;
	int j = 0;

	dev_info(&state->i2cClient->dev, "Si4703_read called\n");
	bytes = i2c_master_recv(state->i2cClient, kernelRcvBuffer, len);
	dev_info(&state->i2cClient->dev, "Bytes read =%d\n", bytes);

	for ( ; j < 32; ++i) {
		dev_info(&state->i2cClient->dev,
				"Reading Registers %d [%d]=%x [%d]=%x\n",
				i,
				j,
				kernelRcvBuffer[j], j+1, kernelRcvBuffer[j+1]);
		j += 2;
	}

	bytes = copy_to_user(buff, kernelRcvBuffer, len);

	return 0;
}

static void Tune(unsigned int freq)
{
	unsigned int nc = 0;
	int bytes = 0;

	nc = freq; //this is 98.1 FM !!!
	nc *= 10;  //this math is for USA FM only
	nc -= 8750;
	nc /= 20;
	char list4[] = {64, 1, 128, nc};

	char list5[] = {64, 1, 0, nc};

	bytes = i2c_master_send(state->i2cClient, list4, sizeof(list4));
	mdelay(1000);
	bytes = i2c_master_send(state->i2cClient, list5, sizeof(list5));
	mdelay(1000);
}


ssize_t  si4703_write(struct file *file,
				const char __user *buffer,
				size_t length,
				loff_t *offset)
{
	ssize_t ret = 0;
	int bytes = 0;
	struct Message *msg = NULL;

	dev_info(&state->i2cClient->dev, "Si4703_write called\n");

	if (length >= BUFFER_LENGTH)
		length = BUFFER_LENGTH;

	ret = copy_from_user(kernelBuffer, buffer, length);

	msg = kmalloc(sizeof(struct Message), GFP_KERNEL);
	msg = (struct Message *)kernelBuffer;

	if (length == sizeof(struct Message) && (msg->Id == TUNE)) {
		Tune(msg->freq);
		dev_info(&state->i2cClient->dev,
				"Tuning to frequency=%d\n", msg->freq);
	} else {
		// Writing data in I2C Bus
		bytes = i2c_master_send(state->i2cClient, kernelBuffer, length);
		dev_info(&state->i2cClient->dev,
				"# bytes written %d\n", bytes);
	}

	return ret;
}

static const struct file_operations si4703_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl	= si4703_ioctl,
	.open		= si4703_open,
	.release	= si4703_close,
	.read		= si4703_read,
	.write		= si4703_write
};

static int si4703_construct_devices(struct si4703_dev *dev, int minor,
		struct class *class)
{
	int err = 0;
	dev_t devno = MKDEV(major, minor);
	struct device *device = NULL;

	BUG_ON(dev == NULL || class == NULL);

	/*Memory is to be allocated when the device is opened the first time*/
	mutex_init(&dev->mutex);

	cdev_init(&dev->cdev, &si4703_fops);
	dev->cdev.owner = THIS_MODULE;

	err = cdev_add(&dev->cdev, devno, 1);
	if (err) {
		dev_err(&state->i2cClient->dev,
			"[target] Error %d while trying to add %s%d",
			err, DEVICE_NAME, minor);
		return err;
	}

	device = device_create(class, NULL, /* no parent device */
		devno, NULL, /* no additional data */
		DEVICE_NAME "%d", minor);

	if (IS_ERR(device)) {
		err = PTR_ERR(device);
		dev_err(&state->i2cClient->dev,
			"[target] Error %d while trying to create %s%d",
			err, DEVICE_NAME, minor);
		cdev_del(&dev->cdev);
		return err;
	}
	return 0;
}

/* Destroy the device and free its buffer */
static void si4703_destroy_device(struct si4703_dev *dev, int minor,
	struct class *class)
{
	BUG_ON(dev == NULL || class == NULL);
	device_destroy(class, MKDEV(major, minor));
	cdev_del(&dev->cdev);
	mutex_destroy(&dev->mutex);
}

static void si4703_cleanup_module(int devices_to_destroy)
{
	int i;

	/* Get rid of character devices (if any exist) */
	if (si4703_devices) {
		for (i = 0; i < devices_to_destroy; ++i) {
			si4703_destroy_device(&si4703_devices[i], i,
					i2ccharClass);
		}
		kfree(si4703_devices);
	}

	if (i2ccharClass)
		class_destroy(i2ccharClass);

	/* [NB] si4703_cleanup_module is never called if alloc_chrdev_region()
	 * has failed.
	 */
	unregister_chrdev_region(MKDEV(major, 0), number_of_devices);
}

int si4703_init(void)
{
	int res = 0;
	int i = 0;
	dev_t dev = 0;
	int devices_to_destroy = 0;

	res = alloc_chrdev_region(&dev, 0, number_of_devices, DEVICE_NAME);
		if (res < 0) {
			pr_err("alloc_chrdev_region() failed\n");
			return res;
	}

	major = MAJOR(dev);

	// Register the device class
	i2ccharClass = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(i2ccharClass)) {
		// Check for error and clean up if there is
		pr_err("Failed to register device class\n");
		return PTR_ERR(i2ccharClass);
	}

	/* Allocate the array of devices */
	si4703_devices = kzalloc(number_of_devices *
			sizeof(struct si4703_dev), GFP_KERNEL);

	if (si4703_devices == NULL) {
		res = -ENOMEM;
		goto fail;
	}

	res = i2c_add_driver(&si4703_driver);

	if (res != 0)
		return res;

	for (i = 0; i < number_of_devices; ++i) {
		res = si4703_construct_devices(&si4703_devices[i],
				i, i2ccharClass);
		if (res) {
			devices_to_destroy = i;
			goto fail;
		}
	}

	pr_info("Device class %s created correctly\n", CLASS_NAME);

	return 0;

fail:
	si4703_cleanup_module(devices_to_destroy);
	return res;
}

void si4703_cleanup(void)
{

	i2c_del_driver(&si4703_driver);
	si4703_cleanup_module(number_of_devices);
	//printk(KERN_INFO "\nExiting Si4703 driver!!\n");
}


module_init(si4703_init);
module_exit(si4703_cleanup);

MODULE_DESCRIPTION("SI4703 I2C client driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Furhad Jidda");


