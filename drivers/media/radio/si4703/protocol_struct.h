/*
 * protocol_struct.h
 *
 *  Created on: Jan 13, 2017
 *      Author: furhad
 */

#ifndef CODE_PROTOCOL_STRUCT_H_
#define CODE_PROTOCOL_STRUCT_H_

enum Command{
	TUNE=0x12,
	SEEK_UP,
	SEEK_DOWN,
	TUNE_DIR_UP,
	TUNE_DIR_DOWN,
};

struct Message{
	enum Command Id;
	unsigned int freq;
};



#endif /* CODE_PROTOCOL_STRUCT_H_ */
