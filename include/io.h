#ifndef IO_H
#define IO_H


#include <iostream>
#include <ethercat.h>


typedef struct PACKED
{
	int32	actual_position;				//0x6064
	int32	actual_velocity;				//0x606c
	int16	actual_torque;				//0x6077
	uint16	status_word;				//0x6041
	int8	operation_mode;				//0x6061
}InputData_t;


typedef struct PACKED
{
	int32	target_position;				//0x607A
	int32	target_velocity;				//0x60ff
	int32	offset_velocity;				//0x60B1
	int16	target_torque;				//0x6071
	int16	offset_torque;				//0x60B2
	uint16	control_word;				//0x6040
	int8	operation_mode;				//0x6060
}OutputData_t;


int io_config();


#endif /* IO_H */
