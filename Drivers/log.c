/*
 * log.c
 *
 *  Created on: 2021-7-16
 *      Author: chenq
 */
#include "log.h"
#include "EC200U.h"
#include "stdio.h"
#include "string.h"

void device_send_packet(struct send_packet packet)
{

	char buffer[256]={0};
	int length=0;
	length=sprintf(buffer,"{\"method\":\"%s\"},\"params\":{%s},\"id\":%d}\r\n",packet.method,packet.params,packet.id);
	//加密
	EC200U_Send_StrData(buffer,length);

}

void device_send_secret(struct secret_data secret_d,int id)
{
	struct send_packet packet;
	char method[7]="secret";
	char params[200];
	sprintf(params,"\"sn\":\"%s\",\"date\":\"%s\"",secret_d.sn,secret_d.date);
	packet.method=method;
	packet.params=params;
	packet.id=id;
	device_send_packet(packet);
}

void device_send_register(struct register_data register_d,int id)
{
	struct send_packet packet;
	char method[9]="register";
	char params[200];
	sprintf(params,"\"sn\":\"%s\",\"sign\":\"%s\",\"date\":\"%s\"",register_d.sn,register_d.sign,register_d.date);
	packet.method=method;
	packet.params=params;
	packet.id=id;
	device_send_packet(packet);
}

void device_send_gps(struct gps_data gps_d,int id)
{
	struct send_packet packet;
	char method[4]="gps";
	char params[200];
	sprintf(params,"\"lat\":\"%f\",\"lon\":\"%f\",\"date\":\"%s\",\"date\":\"%s\",\"speed\":\"%d\",\"sn\":\"%s\"",
			gps_d.lat,gps_d.lon,gps_d.date,gps_d.itinerary_id,gps_d.speed,gps_d.sn);
	packet.method=method;
	packet.params=params;
	packet.id=id;
	device_send_packet(packet);
}

void device_send_alarm(struct alarm_data alarm_d,int id)
{
	struct send_packet packet;
	char method[6]="alarm";
	int n;
	char params[400];
	//注意最后要加一个","
	n=sprintf(params,"\"lat\":\"%f\",\"lon\":\"%f\",\"date\":\"%s\",\"date\":\"%s\",\"speed\":\"%d\",\"sn\":\"%s\",",
			alarm_d.gps.lat,alarm_d.gps.lon,alarm_d.gps.date,alarm_d.gps.itinerary_id,alarm_d.gps.speed,alarm_d.gps.sn);
	sprintf(params+n,"\"alarm_type\":\"%d\",\"alarm_info\":{\"TTC\":\"%.1f\", \"obstacle_type\":\"%d\", \"lengthways_distance\":\"%.1f\",\"LDW\":\"%d\",\"HMW_time\":\"%.1f\"}",
			alarm_d.alarm_type,alarm_d.info.TTC,alarm_d.info.obstacle_type,alarm_d.info.lengthways_distance,alarm_d.info.LDW,alarm_d.info.HMW_time);
	packet.method=method;
	packet.params=params;
	packet.id=id;
	device_send_packet(packet);
}

void device_send_status(struct status_data status_d,int id)
{
	struct send_packet packet;
	char method[14]="device_status";
	int n1,n2,n3;
	char params[200];
	n1=sprintf(params,"\"version\":{\"controller\":\"%s\",\"monitor\":\"%s\",\"camera\":\"%s\"},",
			   status_d.version.controller_version,status_d.version.monitor_version,status_d.version.camera_version);
	n2=sprintf(params+n1,"\"status\":{\"controller\":\"%d\",\"monitor\":\"%d\",\"camera\":\"%d\"},",
			   status_d.status.controller_status,status_d.status.monitor_status, status_d.status.camera_status);
	sprintf(params+n1+n2,"\"date\":\"%s\",\"itinerary_id\":\"%s\",\"sn\":\"%s\"",status_d.date,status_d.itinerary_id,status_d.sn);
	packet.method=method;
	packet.params=params;
	packet.id=id;
	device_send_packet(packet);
}

void device_send_ota(struct ota_data ota_d,int id)
{
	struct send_packet packet;
	char method[19]="ota.request_update";
	int n1,n2,n3;
	char params[200];
	sprintf(params,"\"type\":\"%s\",\"version\":\"%s\",\"date\":\"%s\",\"itinerary_id\":\"%s\",\"sn\":\"%s\"",
			ota_d.type,ota_d.version,ota_d.date,ota_d.itinerary_id,ota_d.sn);
	packet.method=method;
	packet.params=params;
	packet.id=id;
	device_send_packet(packet);
}

