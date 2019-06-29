/*
 * drivers/spi/spi-fram-driver.c
 *
 * SPI driver for fram module
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
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/of_net.h>

#define DRIVER_NAME	"fram_driver"

#define MAJOR_DYNAMIC	0
#define MAJOR_HARDCODE	189
#define BUFFER_LENGTH	2048
#define DEVICE_NAME	"spi-fram-"	// The device will appear at
					//dev/ebbchar using this value
#define CLASS_NAME	"spifram"	// The device class
					//this is a character device driver

struct fram_dev {
	struct mutex mutex;
	struct cdev cdev;
};

static int major;
static struct class *spi_char_class; // The device-driver class struct pointer
static struct fram_dev *fram_devices;
static const unsigned short number_of_devices = 3;

static int fram_read_status_reg(struct spi_device *spi)
{
	int err;
	unsigned char ch16[] = {0x05};
	unsigned char rx16[1] = {};

	struct spi_transfer rdid[] = {
		{
			.tx_buf = ch16,
			.len = sizeof(ch16)
		},
		{
			.rx_buf = rx16,
			.len = sizeof(rx16)
		},
	};

	err = spi_sync_transfer(spi, rdid, ARRAY_SIZE(rdid));
	dev_info(&spi->dev, "rx16csr = %x\n", rx16[0]);

	if (err < 0) {
		dev_err(&spi->dev, "fram_probe spi_sync_transfer failed!\n");
		return err;
	}

	return 0;
}

static int fram_write_enable_latch(struct spi_device *spi)
{
	int err;
	unsigned char ch16[1] = {0x06};

	struct spi_transfer rdid[] = {
		{
			.tx_buf = ch16,
			.len = sizeof(ch16)
		}
	};

	err = spi_sync_transfer(spi, rdid, ARRAY_SIZE(rdid));

	if (err < 0) {
		dev_err(&spi->dev, "fram_write_enable_latch failed!\n");
		return err;
	}

	return 0;
}


static int fram_write_status_reg(struct spi_device *spi)
{
	int err;
	unsigned char ch16[2] = {0x01, 0xEA};

	struct spi_transfer rdid[] = {
		{
			.tx_buf = ch16,
			.len = sizeof(ch16)
		}
	};

	err = spi_sync_transfer(spi, rdid, ARRAY_SIZE(rdid));

	if (err < 0) {
		dev_err(&spi->dev, "fram_write_status_reg failed!\n");
		return err;
	}

	return 0;
}


static int fram_write_memory(struct spi_device *spi, uint16_t address)
{
	int err;
	unsigned char ch16[] = {0x02, 0x0F, 0xFF, 0x45};

	struct spi_transfer rdid[] = {
		{
			.tx_buf = ch16,
			.len = sizeof(ch16)
		}
	};


	err = spi_sync_transfer(spi, rdid, ARRAY_SIZE(rdid));

	if (err < 0) {
		dev_err(&spi->dev, "fram_write_memory failed!\n");
		return err;
	}

	return 0;
}


static int fram_read_memory(struct spi_device *spi, uint16_t address)
{
	int err;
	unsigned char ch16[] = {0x03, 0x0F, 0xFF};
	unsigned char rx16[1] = {};

	struct spi_transfer rdid[] = {
		{
			.tx_buf = ch16,
			.len = sizeof(ch16)
		},
		{
			.rx_buf = rx16,
			.len = sizeof(rx16)
		},
	};

	err = spi_sync_transfer(spi, rdid, ARRAY_SIZE(rdid));
	dev_info(&spi->dev, "fdata = %x\n", rx16[0]);
	if (err < 0) {
		dev_info(&spi->dev, "fram_probe spi_sync_transfer failed!\n");
		return err;
	}

	return 0;
}



static int fram_probe(struct spi_device *spi)
{
	int err;
	unsigned char ch16[] = {0x9F};
	unsigned char rx16[4] = {};

	struct spi_transfer rdid[] = {
		{
			.tx_buf = ch16,
			.len = sizeof(ch16)
		},
		{
			.rx_buf = rx16,
			.len = sizeof(rx16)
		},
	};

	dev_info(&spi->dev, "fram_probe called\n");

	spi->max_speed_hz = 1000000;
	spi->bits_per_word = 8;
	spi->mode = (0);

	err = spi_setup(spi);
	if (err < 0) {
		dev_err(&spi->dev, "fram_probe spi_setup failed!\n");
		return err;
	}

	dev_info(&spi->dev, "spi_setup ok, cs: %d\n", spi->chip_select);

	err = spi_sync_transfer(spi, rdid, ARRAY_SIZE(rdid));
	dev_info(&spi->dev, "rx16=%x %x %x %x\n",
			rx16[0], rx16[1], rx16[2], rx16[3]);

	if (err < 0) {
		dev_info(&spi->dev, "fram_probe spi_sync_transfer failed!\n");
		return err;
	}
	fram_write_enable_latch(spi);
	fram_write_status_reg(spi);
	fram_read_status_reg(spi);
	fram_write_enable_latch(spi);	// This is needed before every write
					//operation
	fram_write_memory(spi, 0x45);
	fram_read_memory(spi, 0x45);

	return 0;
}

static int fram_remove(struct spi_device *spi)
{
	return 0;
}


static const struct of_device_id fram_ids[] = {
	{.compatible = "fram-char-driver"},
	{}
};

MODULE_DEVICE_TABLE(of, fram_ids);

static struct spi_driver fram_driver = {
		.probe = fram_probe,
		.remove = fram_remove,
		.driver = {
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table = fram_ids
		},
};


static int fram_open(struct inode *inode, struct file *filp)
{
	//printk(KERN_INFO "\n fram_open called\n");
	return 0;
}

static int  fram_close(struct inode *inode, struct file *filp)
{
	//printk(KERN_INFO "\nfram_close called\n");
	return 0;
}

ssize_t  fram_read(struct file *file, char __user *buff, size_t len,
			loff_t *offset)
{
	//printk(KERN_INFO "\nfram_read called\n");
	return 0;
}

ssize_t  fram_write(struct file *file,
			const char __user *buffer,
			size_t length,
			loff_t *offset)
{
	//printk(KERN_INFO "\nfram_write called\n");
	return 0;
}

static const struct file_operations fram_fops = {
	.owner		= THIS_MODULE,
	.open		= fram_open,
	.release	= fram_close,
	.read		= fram_read,
	.write		= fram_write

};

static int fram_construct_devices(struct fram_dev *dev, int minor,
			struct class *class)
{
	int err = 0;
	dev_t devno = MKDEV(major, minor);
	struct device *device = NULL;

	BUG_ON(dev == NULL || class == NULL);

	/*Memory is to be allocated when the device is opened the first time*/
	mutex_init(&dev->mutex);

	cdev_init(&dev->cdev, &fram_fops);
	dev->cdev.owner = THIS_MODULE;

	err = cdev_add(&dev->cdev, devno, 1);
	if (err) {
		//dev_dbg(&dev, "[target]Error %d while trying to add %s%d\n",
		//	err, DEVICE_NAME, minor);
		return err;
	}

	device = device_create(class, NULL, /* no parent device */
		devno, NULL, /* no additional data */
		DEVICE_NAME "%d", minor);

	if (IS_ERR(device)) {
		err = PTR_ERR(device);
		//printk(KERN_WARNING "[target] Error %d while trying "
		//		"to create %s%d\n", err, DEVICE_NAME, minor);
		cdev_del(&dev->cdev);
		return err;
	}
	return 0;
}

/* Destroy the device and free its buffer */
static void fram_destroy_device(struct fram_dev *dev, int minor,
	struct class *class)
{
	BUG_ON(dev == NULL || class == NULL);
	device_destroy(class, MKDEV(major, minor));
	cdev_del(&dev->cdev);
	mutex_destroy(&dev->mutex);
}


static void fram_cleanup_module(int devices_to_destroy)
{
	int i;

	/* Get rid of character devices (if any exist) */
	if (fram_devices) {
		for (i = 0; i < devices_to_destroy; ++i) {
			fram_destroy_device(&fram_devices[i],
					i, spi_char_class);
		}
		kfree(fram_devices);
	}

	if (spi_char_class)
		class_destroy(spi_char_class);

	/* [NB] fram_cleanup_module is never called if alloc_chrdev_region()
	 * has failed.
	 */
	unregister_chrdev_region(MKDEV(major, 0), number_of_devices);
}



static int __init fram_driver_init(void)
{
	int res = 0;
	int i = 0;
	dev_t dev = 0;
	int devices_to_destroy = 0;

	res = alloc_chrdev_region(&dev, 0, number_of_devices, DEVICE_NAME);
		if (res < 0) {
			//printk(KERN_WARNING "alloc_chrdev_region() failed\n");
			return res;
	}

	major = MAJOR(dev);

	// Register the device class
	spi_char_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(spi_char_class)) {
		// Check for error and clean up if there is
		pr_err("Failed to register device class\n");
		return PTR_ERR(spi_char_class);
	}

	/* Allocate the array of devices */
	fram_devices = kzalloc(number_of_devices *
			sizeof(struct fram_dev), GFP_KERNEL);

	if (fram_devices == NULL) {
		res = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < number_of_devices; ++i) {
		res = fram_construct_devices(&fram_devices[i], i,
				spi_char_class);
		if (res) {
			devices_to_destroy = i;
			goto fail;
		}
	}

	// Made it! device was initialized
	//printk(KERN_INFO "device class %s created correctly\n", CLASS_NAME);

	return spi_register_driver(&fram_driver);

fail:
	fram_cleanup_module(devices_to_destroy);
	return res;
}

module_init(fram_driver_init);

static void __exit fram_driver_exit(void)
{
	spi_unregister_driver(&fram_driver);
	fram_cleanup_module(number_of_devices);
}

module_exit(fram_driver_exit);

MODULE_DESCRIPTION("Adafruit FRAM SPI driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Furhad Jidda");

