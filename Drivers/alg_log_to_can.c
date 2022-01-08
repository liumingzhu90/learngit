/*
 * alg_log_to_can.c
 *
 *  Created on: 2021-12-13
 *      Author: zkhy
 *  用于存放需要通过CAN输出的算法控制参数和配置参数
 */
#include "alg_log_to_can.h"
/******************
 *输出参数定义：
 *
 *****************/
CMS_Break_State cms_break_state_CAN;
AEB_Break_State aeb_break_state_CAN;
AEB_CMS_Control_Parameters AEB_CMS_config_parameters_CAN;
SSS_state sss_state_CAN;
SSS_parameters sss_config_param_CAN;

// 系统参数
// 1. AEB_CMS+SSS刹车配置参数
uint8_t update_alg_config_parameters(AEB_CMS_Control_Parameters AEB_CMS_config_p, SSS_parameters sss_config_p){
	AEB_CMS_config_parameters_CAN = AEB_CMS_config_p;
	sss_config_param_CAN = sss_config_p;
	return UPDATE_SUCCESS;
}
// 2. 刹车状态参数（AEB+CMS+SSS）
uint8_t update_alg_state_parameters(AEB_Break_State aeb_state, CMS_Break_State cms_state, SSS_state sss_state){
	aeb_break_state_CAN = aeb_state;
	cms_break_state_CAN = cms_state;
	sss_state_CAN = sss_state;
	return UPDATE_SUCCESS;
}
// 2.1 报警状态(AEB+CMS+SSS)
// 2.2 刹车状态(AEB+CMS+SSS)

// 3. 算法内部log 参数




