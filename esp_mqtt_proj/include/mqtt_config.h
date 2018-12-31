#ifndef __MQTT_CONFIG_H__
#define __MQTT_CONFIG_H__

#define CFG_HOLDER	0x00FF55A4	/* Change this value to load default configurations */
#define CFG_LOCATION	0x79	/* Please don't change or if you know what you doing */
#define MQTT_SSL_ENABLE

/*DEFAULT CONFIGURATIONS*/

#define MQTT_HOST			"www.it-rayko.com" //or "mqtt.yourdomain.com"
#define MQTT_PORT			1883
#define MQTT_BUF_SIZE		1024
#define MQTT_KEEPALIVE		130	 /*second*/

#define MQTT_CLIENT_ID		"867086701234567"
#define MQTT_USER			"sedevice"
#define MQTT_PASS			"956497e59024554d38d14456984d8551dec8254d"

#define STA_SSID "ruike"
#define STA_PASS "123456789"

#define STA_TYPE AUTH_WPA2_PSK

#define MQTT_RECONNECT_TIMEOUT 	5	/*second*/

#define DEFAULT_SECURITY	0   //0��1��2�ֱ����û��SSL��SSL������֤��SSL˫����֤
#define QUEUE_BUFFER_SIZE		 		2048

#define PROTOCOL_NAMEv31	/*MQTT version 3.1 compatible with Mosquitto v0.15*/
//PROTOCOL_NAMEv311			/*MQTT version 3.11 compatible with https://eclipse.org/paho/clients/testing/*/

#define SERVER_PORT 80
#endif // __MQTT_CONFIG_H__
