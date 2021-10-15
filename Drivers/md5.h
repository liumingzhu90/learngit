/*
 * md5.h
 *
 *  Created on: 2021-7-19
 *      Author: chenq
 */

#ifndef MD5_H_
#define MD5_H_
/**
  * @input: result -- store the calculation result
  *         size   -- size of result. Make sure it's at least 33
  *                   since the result is a 32-byte hexdecimal string.
  *         message-- string to be encrypted
  *         flag   -- 0 means upper case output, 1 means lower case output
  * @return: 0 -- success
  *          1 -- result size less than 33
  *          2 -- calloc failed
  */
int32_t cal_md5(char *result, size_t size, const char *message, uint32_t flag);
int32_t md5_demo(void);

#endif /* MD5_H_ */
