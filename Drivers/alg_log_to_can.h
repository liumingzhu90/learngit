/*
 * alg_log_to_can.h
 *
 *  Created on: 2021-12-13
 *      Author: zkhy
 */

#ifndef ALG_LOG_TO_CAN_H_
#define ALG_LOG_TO_CAN_H_

#include "aeb_cms_alg.h"
#include "sss_alg.h"

#define CONFIG_SUCCESS 0x01

#define UPDATE_SUCCESS 0x01

// CAN 配置算法参数
uint8_t set_alg_config_parameters(AEB_CMS_Control_Parameters* AEB_CMS_config_p, SSS_parameters* sss_config_p);

// 算法LOG输出到CAN
uint8_t update_alg_config_parameters(AEB_CMS_Control_Parameters AEB_CMS_config_p, SSS_parameters sss_config_p);

uint8_t update_alg_state_parameters(AEB_Break_State aeb_state, CMS_Break_State cms_state, SSS_state sss_state);


#endif /* ALG_LOG_TO_CAN_H_ */
