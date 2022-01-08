cmake_minimum_required (VERSION 3.5)

project (kungfu_aebs_app)

# Configuration types 
SET(CMAKE_CONFIGURATION_TYPES "Release;Debug" CACHE STRING "Configs" FORCE)
IF(DEFINED CMAKE_BUILD_TYPE AND CMAKE_VERSION VERSION_GREATER "2.8")
	SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS  ${CMAKE_CONFIGURATION_TYPES})
ENDIF()

set ( WorkspaceDirPath ${CMAKE_CURRENT_SOURCE_DIR} )
set ( ProjName kungfu_aebs_app )
include_directories(${WorkspaceDirPath})
include_directories(${WorkspaceDirPath}/Drivers)
include_directories(${WorkspaceDirPath}/FreeRTOS/include)
include_directories(${WorkspaceDirPath}/inc)
include_directories(${WorkspaceDirPath}/Library)
include_directories(${WorkspaceDirPath}/Tasks)
include_directories(${WorkspaceDirPath}/Vehicles)
add_executable (kungfu_aebs_app
	${WorkspaceDirPath}/Drivers/AEB_CMS_alg.c
	${WorkspaceDirPath}/Drivers/bluetooth.c
	${WorkspaceDirPath}/Drivers/canhl.c
	${WorkspaceDirPath}/Drivers/config.c
	${WorkspaceDirPath}/Drivers/displayer.c
	${WorkspaceDirPath}/Drivers/EC200U.c
	${WorkspaceDirPath}/Drivers/flash.c
	${WorkspaceDirPath}/Drivers/gpio.c
	${WorkspaceDirPath}/Drivers/log.c
	${WorkspaceDirPath}/Drivers/md5.c
	${WorkspaceDirPath}/Drivers/proportional_valve.c
	${WorkspaceDirPath}/Drivers/screen.c
	${WorkspaceDirPath}/Drivers/stereo_camera.c
	${WorkspaceDirPath}/Drivers/SSS_alg.c
	${WorkspaceDirPath}/Drivers/tim.c
	${WorkspaceDirPath}/Drivers/update.c
	${WorkspaceDirPath}/Drivers/usart.c
	${WorkspaceDirPath}/Drivers/watchdog.c
	${WorkspaceDirPath}/Drivers/wheelSpeed.c
	${WorkspaceDirPath}/FreeRTOS/croutine.c
	${WorkspaceDirPath}/FreeRTOS/event_groups.c
	${WorkspaceDirPath}/FreeRTOS/heap_4.c
	${WorkspaceDirPath}/FreeRTOS/list.c
	${WorkspaceDirPath}/FreeRTOS/port.c
	${WorkspaceDirPath}/FreeRTOS/queue.c
	${WorkspaceDirPath}/FreeRTOS/stream_buffer.c
	${WorkspaceDirPath}/FreeRTOS/tasks.c
	${WorkspaceDirPath}/FreeRTOS/timers.c
	${WorkspaceDirPath}//kf_it.c
	${WorkspaceDirPath}/Library/cJSON.c
	${WorkspaceDirPath}/Library/test.c
	${WorkspaceDirPath}//main.c
	${WorkspaceDirPath}/src/kf32a_basic_adc.c
	${WorkspaceDirPath}/src/kf32a_basic_aes.c
	${WorkspaceDirPath}/src/kf32a_basic_atim.c
	${WorkspaceDirPath}/src/kf32a_basic_bkp.c
	${WorkspaceDirPath}/src/kf32a_basic_btim.c
	${WorkspaceDirPath}/src/kf32a_basic_can.c
	${WorkspaceDirPath}/src/kf32a_basic_cfgl.c
	${WorkspaceDirPath}/src/kf32a_basic_cmp.c
	${WorkspaceDirPath}/src/kf32a_basic_crc.c
	${WorkspaceDirPath}/src/kf32a_basic_dac.c
	${WorkspaceDirPath}/src/kf32a_basic_dma.c
	${WorkspaceDirPath}/src/kf32a_basic_exic.c
	${WorkspaceDirPath}/src/kf32a_basic_flash.c
	${WorkspaceDirPath}/src/kf32a_basic_gpio.c
	${WorkspaceDirPath}/src/kf32a_basic_gptim.c
	${WorkspaceDirPath}/src/kf32a_basic_i2c.c
	${WorkspaceDirPath}/src/kf32a_basic_int.c
	${WorkspaceDirPath}/src/kf32a_basic_iwdt.c
	${WorkspaceDirPath}/src/kf32a_basic_lcd.c
	${WorkspaceDirPath}/src/kf32a_basic_led.c
	${WorkspaceDirPath}/src/kf32a_basic_op.c
	${WorkspaceDirPath}/src/kf32a_basic_osc.c
	${WorkspaceDirPath}/src/kf32a_basic_pclk.c
	${WorkspaceDirPath}/src/kf32a_basic_pm.c
	${WorkspaceDirPath}/src/kf32a_basic_qei.c
	${WorkspaceDirPath}/src/kf32a_basic_rst.c
	${WorkspaceDirPath}/src/kf32a_basic_rtc.c
	${WorkspaceDirPath}/src/kf32a_basic_spi.c
	${WorkspaceDirPath}/src/kf32a_basic_sysctl.c
	${WorkspaceDirPath}/src/kf32a_basic_systick.c
	${WorkspaceDirPath}/src/kf32a_basic_usart.c
	${WorkspaceDirPath}/src/kf32a_basic_usb.c
	${WorkspaceDirPath}/src/kf32a_basic_wwdt.c
	${WorkspaceDirPath}//system_init.c
	${WorkspaceDirPath}/Tasks/aebs_task.c
	${WorkspaceDirPath}/Tasks/alarm.c
	${WorkspaceDirPath}/Tasks/can_task.c
	${WorkspaceDirPath}/Tasks/data_center_services.c
	${WorkspaceDirPath}/Tasks/data_uploading.c
	${WorkspaceDirPath}/Tasks/get_parameter.c
	${WorkspaceDirPath}/Tasks/Report.c
	${WorkspaceDirPath}/Tasks/screen_task.c
	${WorkspaceDirPath}/Tasks/set_parameter.c
	${WorkspaceDirPath}/Tasks/spi_flash.c
	${WorkspaceDirPath}/Tasks/task_manager.c
	${WorkspaceDirPath}/Tasks/uart_task.c
	${WorkspaceDirPath}/Tasks/Urader.c
	${WorkspaceDirPath}/Vehicles/vehicle_can.c
	${WorkspaceDirPath}/Vehicles/vehicle_can_dongfeng.c
	${WorkspaceDirPath}/Vehicles/vehicle_can_futian.c
	${WorkspaceDirPath}/Vehicles/vehicle_can_jinlv.c
	${WorkspaceDirPath}/Vehicles/vehicle_can_kinglong.c
	${WorkspaceDirPath}/Vehicles/vehicle_can_liuqi.c
	${WorkspaceDirPath}/Vehicles/vehicle_can_sunlong.c
	${WorkspaceDirPath}/Vehicles/vehicle_can_wanxiang.c
	${WorkspaceDirPath}/Vehicles/vehicle_can_zhongche.c
	${WorkspaceDirPath}/Vehicles/vehicle_can_zhongtong.c
	${WorkspaceDirPath}/Vehicles/vehicle_can_zhongxing.c
	${WorkspaceDirPath}/Vehicles/vehicle_can_zkhy.c
	${WorkspaceDirPath}/_config/startup.c
	${WorkspaceDirPath}/_config/vector.c)


