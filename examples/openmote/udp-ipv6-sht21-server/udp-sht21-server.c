/*
 * Copyright (c) 2012, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** \addtogroup cc2538-examples
 * @{
 *
 * \defgroup cc2538-echo-server cc2538dk UDP Echo Server Project
 *
 *  Tests that a node can correctly join an RPL network and also tests UDP
 *  functionality
 * @{
 *
 * \file
 *  An example of a simple UDP echo server for the cc2538dk platform
 */
#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include <string.h>

#include "sht21.h"  // related sht21 readings
#include "i2c.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"
#include "dev/watchdog.h"
#include "dev/leds.h"
#include "net/rpl/rpl.h"
/*---------------------------------------------------------------------------*/
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[uip_l2_l3_hdr_len])

#define MAX_PAYLOAD_LEN 120
#define MEASURE_INTERVAL_TIME_MINUTE	30   // humidity related
/*---------------------------------------------------------------------------*/
static struct uip_udp_conn *server_conn;
static char buf[MAX_PAYLOAD_LEN];
static uint16_t len;

static char shtMsg[20];
/*---------------------------------------------------------------------------*/
PROCESS(udp_sht21_server_process, "UDP echo server process");
PROCESS(humidity, "Relative humidity measurement");
AUTOSTART_PROCESSES(&humidity, &udp_sht21_server_process);
/*---------------------------------------------------------------------------*/
static void
prepSht21Msg(float num, uint8_t preci)
{
    int integer=(int)num;
    int decimal = 0;

    memset(shtMsg, 0, sizeof(shtMsg));
    preci = preci > 10 ? 10 : preci;
    num -= integer;
    while( (num != 0) && (preci-- > 0) ){
        decimal *= 10;
        num *= 10;
        decimal += (int)num;
        num -= (int)num;
    }
    snprintf(shtMsg, sizeof(shtMsg), "%d.%d", integer, decimal);
    return;
}

/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  memset(buf, 0, MAX_PAYLOAD_LEN);
  if(uip_newdata()) {
    leds_on(LEDS_RED);
    //len = uip_datalen();
    //memcpy(buf, uip_appdata, len);
    len = sizeof(shtMsg);
    memcpy(buf, shtMsg, len);
    PRINTF("%u bytes from [", len);
    PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
    PRINTF("]:%u\n", UIP_HTONS(UIP_UDP_BUF->srcport));
    uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
    server_conn->rport = UIP_UDP_BUF->srcport;

    uip_udp_packet_send(server_conn, buf, len);
    uip_create_unspecified(&server_conn->ripaddr);
    server_conn->rport = 0;
  }
  leds_off(LEDS_RED);
  return;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_sht21_server_process, ev, data)
{

  PROCESS_BEGIN();
  PRINTF("Starting UDP echo server\n");

  server_conn = udp_new(NULL, UIP_HTONS(0), NULL);
  udp_bind(server_conn, UIP_HTONS(3000));

  PRINTF("Listen port: 3000, TTL=%u\n", server_conn->ttl);

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(humidity, ev, data)
{
    static struct etimer timer;             
    static uint16_t sht21Measurement;
    PROCESS_BEGIN();

    while(1){
        //etimer_set(&timer, CLOCK_CONF_SECOND * 20 * MEASURE_INTERVAL_TIME_MINUTE);
        etimer_set(&timer, CLOCK_CONF_SECOND * 20 );
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
        /* We must init I2C each time, since the module loses its state upon enterin PM2 */

        printf("temp/hum\n");
        leds_on(LEDS_GREEN);
        i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_FAST_BUS_SPEED);

        if(read_SHT21(&sht21Measurement, SHT21_TEMP_REGISTER) == I2C_MASTER_ERR_NONE){
            sht21Measurement &= ~0x0003;
            printf("\n");
		    prepSht21Msg(((((float)sht21Measurement)/65536)*175.72-46.85), SHT21_DECIMAL_PRECISION);
        } else {
            printf("\n T Error");
        }
        if( read_SHT21(&sht21Measurement, SHT21_HUMI_REGISTER) == I2C_MASTER_ERR_NONE){
            sht21Measurement &= ~0x000F;
            printf("\n");
			prepSht21Msg(((((float)sht21Measurement)/65536)*125-6), SHT21_DECIMAL_PRECISION);
        } else {
            printf("\n H Error");
        }
        leds_off(LEDS_GREEN);
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
