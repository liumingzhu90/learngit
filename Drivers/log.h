/*
 * log.h
 *
 *  Created on: 2021-7-16
 *      Author: chenq
 */

#ifndef LOG_H_
#define LOG_H_

#include "system_init.h"
struct send_packet
{
	char * method;
	char * params;
	int id;
};

struct recv_packet
{
	char * result;
	char * error;
	int id;
};

struct register_data
{
	char * sn;
	char * sign;
	char * date;
};

struct gps_data
{
	float lat;
	float lon;
	char * date;
	char * itinerary_id;
	int speed;
	char * sn;
};
struct alarm_info
{
	float TTC;
	uint8_t obstacle_type;
	float lengthways_distance;
	uint8_t LDW;
	float HMW_time;
};

struct alarm_data
{
	struct gps_data gps;
	uint8_t alarm_type;
	struct alarm_info info;
};

struct device_version
{
	char * controller_version;
	char * monitor_version;
	char * camera_version;
};
struct device_status
{
	uint8_t controller_status;
	uint8_t monitor_status;
	uint8_t camera_status;
};
struct status_data
{
	struct device_version version;
	struct device_status status;
	char * date;
	char * itinerary_id;
	char * sn;
};
struct ota_data
{
	char * type;
	char * version;
	char * date;
	char * itinerary_id;
	char * sn;
};
struct secret_data
{
	char * sn;
	char * date;
};
void device_send_secret(struct secret_data secret_d,int id);
void device_send_register(struct register_data register_d,int id);
void device_send_gps(struct gps_data gps_d,int id);
void device_send_alarm(struct alarm_data alarm_d,int id);
void device_send_status(struct status_data status_d,int id);
void device_send_ota(struct ota_data ota_d,int id);
void device_send_packet(struct send_packet packet);

#endif /* LOG_H_ */
