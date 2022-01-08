/*
 * update_program.h
 *
 *  Created on: 2021-7-15
 *      Author: chenq
 */

#ifndef UPDATE_H_
#define UPDATE_H_

#define PROGRAM_SIZE      0x38000
#define PROGRAM_START_ADDRESS 0x40000

int update_program(void);
int check_flag(void);
void set_flag(void);
void clear_flag(void);
#endif /* UPDATE_H_ */
