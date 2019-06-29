/*
 * si4703_include.h
 *
 *  Created on: Jan 2, 2017
 *      Author: furhad
 */

#ifndef CODE_SI4703_INCLUDE_H_
#define CODE_SI4703_INCLUDE_H_


#define DRIVER_NAME	"si4703_driver"

#define MAJOR_DYNAMIC 	0
#define MAJOR_HARDCODE 	189
#define BUFFER_LENGTH 	2048
#define DEVICE_NAME 	"i2cchar"    ///< The device will appear at /dev/ebbchar using this value
#define CLASS_NAME  	"i2c"        ///< The device class -- this is a character device driver

struct si4703_dev {
	struct mutex mutex;
	struct cdev cdev;
};

#endif /* CODE_SI4703_INCLUDE_H_ */
