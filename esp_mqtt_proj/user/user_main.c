/* main.c -- MQTT client example
*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#include "ets_sys.h"
#include "uart.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"
#include "user_config.h"
#include "smartconfig.h"
#include "user_esp_platform.h"
MQTT_Client mqttClient;
 sint8 id[16];


volatile unsigned char netstate = 0;
volatile unsigned char wifi2s =0;
volatile unsigned char onoff = 0;
volatile unsigned char settimeonoff =0 ;
volatile unsigned char offlinetime =0;
volatile unsigned int errortime =2;
volatile unsigned int errortimebuff =120;
volatile unsigned int opentime;
volatile unsigned int opentimebuff;
volatile unsigned int mcounter=0;
uint8 ssidbuff[32];
uint8 passwordbuff[64];

extern volatile unsigned char  mcuonoff;
volatile unsigned char mcuonoffcounter=0;
extern struct esp_platform_saved_param esp_param;


static ETSTimer sntp_timer,wifi_timer;

static void ICACHE_FLASH_ATTR sntpfn()
{

     os_timer_disarm(&sntp_timer);
 //    MQTT_Connect(&mqttClient);

}

void wifiConnectCb(uint8_t status)
{

	if(status == STATION_GOT_IP){
        //sntp_setservername(0, "3.tw.pool.ntp.org");        // set sntp server after got ip address
      //sntp_init();
      os_timer_disarm(&sntp_timer);
      os_timer_setfn(&sntp_timer, (os_timer_func_t *)sntpfn, NULL);
      os_timer_arm(&sntp_timer, 5000, 1);//1s


	} else
	{
		MQTT_Disconnect(&mqttClient);
	}
}

void ICACHE_FLASH_ATTR
smartconfig_done(sc_status status, void *pdata)
{
    switch(status) {
        case SC_STATUS_WAIT:
            os_printf("SC_STATUS_WAIT\n");
            break;
        case SC_STATUS_FIND_CHANNEL:
            os_printf("SC_STATUS_FIND_CHANNEL\n");
            break;
        case SC_STATUS_GETTING_SSID_PSWD:
            os_printf("SC_STATUS_GETTING_SSID_PSWD\n");
			sc_type *type = pdata;
            if (*type == SC_TYPE_ESPTOUCH) {
                os_printf("SC_TYPE:SC_TYPE_ESPTOUCH\n");
            } else {
                os_printf("SC_TYPE:SC_TYPE_AIRKISS\n");
            }
            break;
        case SC_STATUS_LINK:
            os_printf("SC_STATUS_LINK\n");
             struct station_config *sta_conf = pdata;
             spi_flash_erase_sector	(1018);
             spi_flash_write (1018*4*1024,(uint32*) sta_conf->ssid, 32);
             spi_flash_erase_sector	(1017);
             spi_flash_write (1017*4*1024, (uint32*)sta_conf->password, 64);




           // wifi_station_set_config(sta_conf);
            wifi_station_disconnect();
            WIFI_Connect(sta_conf->ssid, sta_conf->password, wifiConnectCb);
            break;
        case SC_STATUS_LINK_OVER:

            os_printf("SC_STATUS_LINK_OVER\n");
            if (pdata != NULL) {
				//SC_TYPE_ESPTOUCH
                uint8 phone_ip[4] = {0};
                os_memcpy(phone_ip, (uint8*)pdata, 4);
                os_printf("Phone ip: %d.%d.%d.%d\n",phone_ip[0],phone_ip[1],phone_ip[2],phone_ip[3]);
            }
            smartconfig_stop();
            break;
    }

}



static void ICACHE_FLASH_ATTR wifi_status()
{
	sint8 pub_message[50];

	/*wifi breath to 51*/
	if(wifi2s>1)
    {
    	wifi2s = 0;
    	if(netstate == 2)
    	{
    		uart0_tx_buffer("#2 2$",5);
    	}
    	if(netstate == 3)
    	{
    		uart0_tx_buffer("#2 3$",5);
    	}
    	if(netstate == 1)
    	{
    		uart0_tx_buffer("#2 1$",5);
    	}

    }
    else
    wifi2s++;

	/*minute counter */
    if(mcounter<59)
    	mcounter++;
    else
    {
    	mcounter =0;
    	if(netstate==2)
		{

			if(onoff==1)
			  os_sprintf(pub_message,"id:%s status:1",id);
			else
			  os_sprintf(pub_message,"id:%s status:0",id);
			MQTT_Publish(&mqttClient, "notify", pub_message, os_strlen(pub_message), 0, 0);
		}
    }

		 if(netstate !=2)
		 {
			if(offlinetime<errortimebuff)
			 offlinetime++;
			else
			{
				onoff =0;
				offlinetime =0;
				uart0_tx_buffer("#4 12$",6);
			}
		 }
		 else
		 {
			 offlinetime= 0;
			 if(onoff == 0)
			 {
				if(mcuonoff==1)
				{
					 mcuonoffcounter++;
					 if(mcuonoffcounter>6)
					 {
						 mcuonoffcounter=0;
						 uart0_tx_buffer("#4 12$",6);
					 }
				}

			 }
			 else
			mcuonoffcounter =0;
		 }



	if(settimeonoff)
	{
		if(opentimebuff>0)
		{
			opentimebuff--;
		}
		else
		{
   			uart0_tx_buffer("#4 12$",6);
   			settimeonoff = 0 ;
   			onoff = 0;
   			os_sprintf(pub_message,"id:%s t:%d p:on status:201",id,opentime);
   		    MQTT_Publish(&mqttClient, "notify", pub_message,os_strlen(pub_message), 0, 0);
		}

	}

}



void mqttConnectedCb(uint32_t *args)
{
	signed char temp_topic[64];
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Connected\r\n");
	//uart0_tx_buffer("#2 2$",5);
	netstate = 2;

	os_sprintf(temp_topic,"control/%s",id);
	MQTT_Subscribe(&mqttClient, temp_topic, 0);
	os_sprintf(temp_topic,"assist/%s",id);
	MQTT_Subscribe(&mqttClient, temp_topic, 0);
	os_sprintf(temp_topic,"reset/%s",id);
	MQTT_Subscribe(&mqttClient, temp_topic, 0);

	os_sprintf(temp_topic,"id:%s status:200",id);
    MQTT_Publish(&mqttClient, "notify", temp_topic,os_strlen(temp_topic), 0, 0);

	os_sprintf(temp_topic,"id:%s network:wifi",id);
    MQTT_Publish(&mqttClient, "notify", temp_topic,os_strlen(temp_topic), 0, 0);

	os_sprintf(temp_topic,"id:%s status:800",id);
    MQTT_Publish(&mqttClient, "notify", temp_topic,os_strlen(temp_topic), 0, 0);

}

void mqttDisconnectedCb(uint32_t *args)
{
	if(netstate == 2)
	netstate = 1;
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");

}

void mqttPublishedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
}

void mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
	char *topicBuf = (char*)os_zalloc(topic_len+1),
			*dataBuf = (char*)os_zalloc(data_len+1);

	//char *send51=(char*)os_zalloc(50);
	char intervaltime[5] = {0};
	char errorbuff[5] = {0};
	unsigned char i =0;
	char user[50] ={0};
	char sendbuff[50];

	MQTT_Client* client = (MQTT_Client*)args;

	os_memcpy(topicBuf, topic, topic_len);
	topicBuf[topic_len] = 0;

	os_memcpy(dataBuf, data, data_len);
	dataBuf[data_len] = 0;

//	INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);

	if (strstr(dataBuf,"p:on t:"))
	{
	    dataBuf = dataBuf+7;
	    opentime = atoi(dataBuf);
	    opentimebuff = opentime*60;
	    if(mcuonoff==0)
		os_sprintf(sendbuff,"#4 11 %d $",opentime);
		uart0_tx_buffer(sendbuff,os_strlen(sendbuff));
		settimeonoff =1;
	    onoff = 1;
		os_sprintf(sendbuff,"id:%s t:%d p:on status:200",id,opentime);
	    MQTT_Publish(&mqttClient, "notify", sendbuff,os_strlen(sendbuff), 0, 0);

	}

	else if(strstr(dataBuf,"devkey: "))
	{

			str_cut(dataBuf,sendbuff,' ',2);
			system_param_load(ESP_PARAM_START_SEC, 0, &esp_param, sizeof(esp_param));
			os_sprintf(esp_param.devkey,sendbuff);
			system_param_save_with_protect(ESP_PARAM_START_SEC, &esp_param, sizeof(esp_param));

	}

	else if(strstr(dataBuf,"welcomeInterval:"))
	{
		dataBuf = dataBuf+16;
		while(*dataBuf != ' ')
		{
			intervaltime[i++] = *dataBuf++;
		}
		dataBuf = dataBuf+11;
		errortime = atoi(dataBuf);
		os_sprintf(sendbuff,"#4 2 %s %s $",intervaltime,dataBuf);
		uart0_tx_buffer(sendbuff,os_strlen(sendbuff));
		os_sprintf(sendbuff,"id:%s welcomeIntervalCurrent:%s",id,intervaltime);
	    MQTT_Publish(&mqttClient, "notify", sendbuff,os_strlen(sendbuff), 0, 0);
	}


	else if(!strcmp(dataBuf,"p:on"))
	{
		uart0_tx_buffer("#4 11 9999 $",6);
		onoff =1;
		os_sprintf(sendbuff,"id:%s p:on status:200",id);
	    MQTT_Publish(&mqttClient, "notify", sendbuff,os_strlen(sendbuff), 0, 0);

	}
	else if(!strcmp(dataBuf,"p:off"))
	{
		uart0_tx_buffer("#4 12$",6);
		onoff= 0;
		os_sprintf(sendbuff,"id:%s p:off status:200",id);
	    MQTT_Publish(&mqttClient, "notify", sendbuff,os_strlen(sendbuff), 0, 0);
	}

	else if(!strcmp(dataBuf,"excution:pause"))
		uart0_tx_buffer("#4 12$",6);
	else if(!strcmp(dataBuf,"excution:resume"))
		uart0_tx_buffer("#4 11$",6);
	else if(!strcmp(dataBuf,"sound:max"))
	{
		uart0_tx_buffer("#4 1 15 $",9);
		os_sprintf(sendbuff,"id:%s soundCurrent:max",id);
	    MQTT_Publish(&mqttClient, "notify", sendbuff,os_strlen(sendbuff), 0, 0);
	}

	else if(!strcmp(dataBuf,"sound:normal"))
	{
		uart0_tx_buffer("#4 1 7 $",8);
		os_sprintf(sendbuff,"id:%s soundCurrent:normal",id);
	    MQTT_Publish(&mqttClient, "notify", sendbuff,os_strlen(sendbuff), 0, 0);
	}
	else if(!strcmp(dataBuf,"sound:min"))
	{
		uart0_tx_buffer("#4 1 2 $",8);
		os_sprintf(sendbuff,"id:%s soundCurrent:min",id);
	    MQTT_Publish(&mqttClient, "notify", sendbuff,os_strlen(sendbuff), 0, 0);
	}
	else if(!strcmp(dataBuf,"auto:on"))
		uart0_tx_buffer("#4 45$",6);
	else if(!strcmp(dataBuf,"gear:four"))
		uart0_tx_buffer("#4 44$",6);
	else if(!strcmp(dataBuf,"gear:max"))
		uart0_tx_buffer("#4 43$",6);
	else if(!strcmp(dataBuf,"gear:normal"))
		uart0_tx_buffer("#4 42$",6);
	else if(!strcmp(dataBuf,"gear:min"))
		uart0_tx_buffer("#4 41$",6);
	else if(!strcmp(dataBuf,"sleep:on"))
		uart0_tx_buffer("#4 51$",6);
	else if(!strcmp(dataBuf,"sleep:off"))
		uart0_tx_buffer("#4 52$",6);
	else if(!strcmp(dataBuf,"ion:on"))
		uart0_tx_buffer("#4 61$",6);
	else if(!strcmp(dataBuf,"ion:off"))
		uart0_tx_buffer("#4 62$",6);
	else if(!strcmp(dataBuf,"sterilize:on"))
		uart0_tx_buffer("#4 71$",6);
	else if(!strcmp(dataBuf,"sterilize:off"))
		uart0_tx_buffer("#4 72$",6);
	else if(!strcmp(dataBuf,"lock:on"))
		uart0_tx_buffer("#4 81$",6);
	else if(!strcmp(dataBuf,"lock:off"))
		uart0_tx_buffer("#4 82$",6);
	else if(!strcmp(dataBuf,"welcome:on"))
	{
		uart0_tx_buffer("#4 91$",6);
		os_sprintf(sendbuff,"id:%s welcomeCurrent:on",id);
	    MQTT_Publish(&mqttClient, "notify", sendbuff,os_strlen(sendbuff), 0, 0);
	}
	else if(!strcmp(dataBuf,"welcome:off"))
	{
		uart0_tx_buffer("#4 92$",6);
		os_sprintf(sendbuff,"id:%s welcomeCurrent:off",id);
	    MQTT_Publish(&mqttClient, "notify", sendbuff,os_strlen(sendbuff), 0, 0);
	}
	else if(!strcmp(dataBuf,"act:reset"))
		uart0_tx_buffer("#4 a1$",6);
	else if(!strcmp(dataBuf,"wifioff:1"))
		uart0_tx_buffer("#4 z1$",6);

	os_free(topicBuf);
	os_free(dataBuf);
}


/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
 *******************************************************************************/
uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}


void user_init(void)
{

	UART_SetPrintPort(1);
	uart_init(9600,9600);
	os_delay_us(60000);

//	WIFI_Connect(STA_SSID, STA_PASS, wifiConnectCb);
//  MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
    netstate =1;

	MQTT_InitConnection(&mqttClient, MQTT_HOST, MQTT_PORT, DEFAULT_SECURITY);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);
	//MQTT_Disconnect(&mqttClient);


	user_esp_platform_init();
    user_webserver_init(SERVER_PORT);


    os_timer_disarm(&wifi_timer);
    os_timer_setfn(&wifi_timer, (os_timer_func_t *)wifi_status, NULL);
    os_timer_arm(&wifi_timer, 1000, 1);//1s


	struct station_config sta_conf;
	uint32 bitbuff[16];
	spi_flash_read(1018*4*1024, (uint32*)sta_conf.ssid, 32);
	spi_flash_read(1017*4*1024, (uint32*)sta_conf.password, 64);
	spi_flash_read(1016*4*1024, (uint32*)id, 16);

	if(os_strncmp(id,"\xff\xff",2)!=0)
	{

		MQTT_InitClient(&mqttClient, id, MQTT_USER, MQTT_PASS, MQTT_KEEPALIVE, 1);  //

	}

    wifi_station_disconnect();
	os_delay_us(60000);

	if(os_strncmp(sta_conf.ssid,"\xff\xff",2)!=0)
	{

		WIFI_Connect(sta_conf.ssid, sta_conf.password, wifiConnectCb);

	}



}
