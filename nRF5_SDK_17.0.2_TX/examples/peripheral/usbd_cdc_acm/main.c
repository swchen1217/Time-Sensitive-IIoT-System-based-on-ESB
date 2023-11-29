/**
 * Copyright (c) 2017 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"
#include "nrf_esb.h"

#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "boards.h"
#include "bsp.h"
#include "bsp_cli.h"
#include "nrf_cli.h"
#include "nrf_cli_uart.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_gpiote.h"

#if NRF_CLI_ENABLED
/**
 * @brief CLI interface over UART
 */
NRF_CLI_UART_DEF(m_cli_uart_transport, 0, 64, 16);
NRF_CLI_DEF(m_cli_uart,
            "uart_cli:~$ ",
            &m_cli_uart_transport.transport,
            '\r',
            4);
#endif

APP_TIMER_DEF(sync_pkt_send_timer); //timer udddddddsed for triggering sending sync pkt
APP_TIMER_DEF(chn_pkt_send_timer);  //timer used for triggering sending channel hopping pkt

/**@file
 * @defgroup usbd_cdc_acm_example main.c
 * @{
 * @ingroup usbd_cdc_acm_example
 * @brief USBD CDC ACM example
 *
 */

#define LED_USB_RESUME      (BSP_BOARD_LED_0)
#define LED_CDC_ACM_OPEN    (BSP_BOARD_LED_1)
#define LED_CDC_ACM_RX      (BSP_BOARD_LED_2)
#define LED_CDC_ACM_TX      (BSP_BOARD_LED_3)

#define BTN_CDC_DATA_SEND       0
#define BTN_CDC_NOTIFY_SEND     1

#define BTN_CDC_DATA_KEY_RELEASE        (bsp_event_t)(BSP_EVENT_KEY_LAST + 1)

/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

#define SYNC_INTERVAL     700 //(ms)
#define CH_TRIAL_INTERVAL 50 //(ms)
#define CH_PKT_ATTEMP     100 //(times)

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1


/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);
#define TLV_TYPE_INDEX   0
#define TLV_LENGTH_INDEX 1
#define CDC_ACM_DATA 0
#define CDC_ACM_CHN_MAP_UPDATE 1
#define TLV_HEADER_LEN    2  //cdc acm should read two bytes first,type and length each with one byte
#define READ_SIZE 1
#define ENDLINE_STRING "\r\n"
#define CHANNEL_MAP_SIZE 37

static char m_rx_buffer[READ_SIZE];
static char m_tx_buffer[NRF_DRV_USBD_EPSIZE];
static char m_test_buffer[NRF_DRV_USBD_EPSIZE];
static bool m_send_flag = 0;
//static uint32_t seq_num = 0;

//esb param
static nrf_esb_payload_t        tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00);

static nrf_esb_payload_t        rx_payload;

static bool m_gpio_trigger_enabled;
static void ts_evt_callback(const ts_evt_t* evt);
static void ts_gpio_trigger_enable(void);

static nrf_atomic_flag_t urllc_pkt_in_progress = false; //this flag is set when urllc pkt is processing
static uint8_t   ch_pkt_count = 0;
static uint32_t  first_ch_target;
static bool first_ch_target_set = false;                //this flag will be true when first channel hopping tick target is set

typedef struct {
    uint32_t seq;
    uint8_t data[32];
}acmDataPacket;

 
/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const *p_inst,
                                    app_usbd_cdc_acm_user_event_t event) {

  app_usbd_cdc_acm_t const *p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

  switch (event) {
  case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN: {
    bsp_board_led_on(LED_CDC_ACM_OPEN);

    /*Setup first transfer*/
    ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
        m_rx_buffer,
        READ_SIZE);
    UNUSED_VARIABLE(ret);
    break;
  }
  case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
    bsp_board_led_off(LED_CDC_ACM_OPEN);
    break;
  case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
    bsp_board_led_invert(LED_CDC_ACM_TX);
    break;

 case APP_USBD_CDC_ACM_USER_EVT_RX_DONE: {
    ret_code_t ret;
    static uint8_t index = 0;
    static uint8_t type = 0;
    static uint8_t val_len = 0;
    static uint32_t seq_num = 0;
    static uint8_t cdc_acm_data[32];    //buffer for receiving cdc_acm_data
    index++;                            //the first byte is read in case : APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN
    do {

        ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, &m_tx_buffer[index], READ_SIZE);

        if (ret == NRF_SUCCESS) {
            index++;
        }

        if(index >= TLV_HEADER_LEN){
          type = m_tx_buffer[TLV_TYPE_INDEX];
          val_len = m_tx_buffer[TLV_LENGTH_INDEX];
        }

        if(index >= TLV_HEADER_LEN + val_len){
          switch(type){
            case CDC_ACM_DATA:{

                NRF_LOG_INFO("USBD CDC ACM RX.");

                urllc_payload urllc_pkt;
                urllc_pkt.header.message_id = URLLC_DATA_PKT;
                memcpy(&urllc_pkt.seq_num,&m_tx_buffer[2],sizeof(uint32_t));
                memcpy(&urllc_pkt.data,&m_tx_buffer[6],val_len - sizeof(uint32_t));
                size_t size = sprintf(m_test_buffer,"%d,%s\n",urllc_pkt.seq_num,urllc_pkt.data); 
                app_usbd_cdc_acm_write(&m_app_cdc_acm, m_test_buffer,size);
                urllc_pkt.time_stamp = TIME_SYNC_TIMESTAMP_TO_USEC(ts_timestamp_get_ticks_u64());

                tx_payload.noack = false;
                tx_payload.length = sizeof(urllc_pkt);
                nrf_esb_set_retransmit_count(5);
                //current counter val + 0.5sec is the ch_tick_target             
                memcpy(tx_payload.data,&urllc_pkt,sizeof(urllc_pkt));

                if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS) {
                      
                      NRF_LOG_INFO("Sending channel map update pkt succeed");
                      bsp_board_led_invert(LED_CDC_ACM_RX);
                  }
                  else {
                      NRF_LOG_WARNING("Sending packet failed");
                  }

                index = 0;
       
            }
            break;
            case CDC_ACM_CHN_MAP_UPDATE:{
                if(val_len <= CHANNEL_MAP_SIZE){
                  
                  uint32_t err_code;
                  uint8_t* chn_map = (uint8_t*)malloc(val_len*sizeof(uint8_t));
                  memcpy(chn_map,&m_tx_buffer[2],val_len);
                  chn_map_update_req chn_map_update_pkt;
                  /*
                  if(!first_ch_target_set){
                      first_ch_target = ts_timestamp_get_counter_val_u32() + 1000;
                      err_code = ch_set_trigger(first_ch_target);
                      APP_ERROR_CHECK(err_code);
                      ch_set_chn_map_change_target(first_ch_target);
                      ch_update_channel_map(chn_map);
                      chn_map_update_pkt.ch_target = first_ch_target;
                      memcpy(&chn_map_update_pkt.chn_map,chn_map,val_len*sizeof(uint8_t));
                      first_ch_target_set = true;
                  }*/
                  first_ch_target = ts_timestamp_get_counter_val_u32() + 1000;
                  chn_map_update_pkt.ch_target = first_ch_target;
                  memcpy(&chn_map_update_pkt.chn_map,chn_map,val_len*sizeof(uint8_t));
                  //size_t size = sprintf(m_test_buffer,"%d\n",chn_map);
                  chn_map_update_pkt.header.message_id = CH_PKT;
                  tx_payload.noack = false;
                  tx_payload.length = sizeof(chn_map_update_pkt);
                  nrf_esb_set_retransmit_count(5);
                  //current counter val + 0.5sec is the ch_tick_target             
                  memcpy(tx_payload.data,&chn_map_update_pkt,sizeof(chn_map_update_pkt));

                  if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS) {
                        
                        NRF_LOG_INFO("Sending channel map update pkt succeed");
                        bsp_board_led_invert(LED_CDC_ACM_RX);
                    }
                    else {
                        NRF_LOG_WARNING("Sending packet failed");
                    }
                  //memcpy(m_test_buffer,chn_map,val_len);
                  //app_usbd_cdc_acm_write(&m_app_cdc_acm, m_test_buffer,val_len);
                  index = 0;
                  free(chn_map);
                }                  
            }
            break;
          }
        }
    } while (ret == NRF_SUCCESS);
    break;
  }
 }
}

    static void usbd_user_ev_handler(app_usbd_event_type_t event) {
      switch (event) {
      case APP_USBD_EVT_DRV_SUSPEND:
        bsp_board_led_off(LED_USB_RESUME);
        break;
      case APP_USBD_EVT_DRV_RESUME:
        bsp_board_led_on(LED_USB_RESUME);
        break;
      case APP_USBD_EVT_STARTED:
        break;
      case APP_USBD_EVT_STOPPED:
        app_usbd_disable();
        bsp_board_leds_off();
        break;
      case APP_USBD_EVT_POWER_DETECTED:
        NRF_LOG_INFO("USB power detected");

        if (!nrf_drv_usbd_is_enabled()) {
          app_usbd_enable();
        }
        break;
      case APP_USBD_EVT_POWER_REMOVED:
        NRF_LOG_INFO("USB power removed");
        app_usbd_stop();
        break;
      case APP_USBD_EVT_POWER_READY:
        NRF_LOG_INFO("USB ready");
        app_usbd_start();
        break;
      default:
        break;
      }
    }

    static void bsp_event_callback(bsp_event_t ev) {
      ret_code_t ret;
      switch ((unsigned int)ev) {
      case CONCAT_2(BSP_EVENT_KEY_, BTN_CDC_DATA_SEND): {
        m_send_flag = 1;
        break;
      }

      case BTN_CDC_DATA_KEY_RELEASE: {
        m_send_flag = 0;
        break;
      }

      case CONCAT_2(BSP_EVENT_KEY_, BTN_CDC_NOTIFY_SEND): {
        ret = app_usbd_cdc_acm_serial_state_notify(&m_app_cdc_acm,
            APP_USBD_CDC_ACM_SERIAL_STATE_BREAK,
            false);
        UNUSED_VARIABLE(ret);
        break;
      }

      default:
        return; // no implementation needed
      }
    }

static void init_bsp(void) {
  ret_code_t ret;
  ret = bsp_init(BSP_INIT_BUTTONS, bsp_event_callback);
  APP_ERROR_CHECK(ret);

  UNUSED_RETURN_VALUE(bsp_event_to_button_action_assign(BTN_CDC_DATA_SEND,
      BSP_BUTTON_ACTION_RELEASE,
      BTN_CDC_DATA_KEY_RELEASE));

  /* Configure LEDs */
  bsp_board_init(BSP_INIT_LEDS);
}

#if NRF_CLI_ENABLED
static void init_cli(void)
{
    ret_code_t ret;
    ret = bsp_cli_init(bsp_event_callback);
    APP_ERROR_CHECK(ret);
    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    uart_config.pseltxd = TX_PIN_NUMBER;
    uart_config.pselrxd = RX_PIN_NUMBER;
    uart_config.hwfc    = NRF_UART_HWFC_DISABLED;
    ret = nrf_cli_init(&m_cli_uart, &uart_config, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(ret);
    ret = nrf_cli_start(&m_cli_uart);
    APP_ERROR_CHECK(ret);
}
#endif

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            NRF_LOG_DEBUG("TX SUCCESS EVENT");
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            NRF_LOG_DEBUG("TX FAILED EVENT");
            (void) nrf_esb_flush_tx();
            (void) nrf_esb_start_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            NRF_LOG_DEBUG("RX RECEIVED EVENT");
            while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
                if (rx_payload.length > 0)
                {
                    NRF_LOG_DEBUG("RX RECEIVED PAYLOAD");
                }
            }
            break;
    }
}


void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


void gpio_init( void )
{
    nrf_gpio_range_cfg_output(8, 15);
    bsp_board_init(BSP_INIT_LEDS);
}

static void ts_gpio_trigger_enable(void)
{
    uint64_t time_now_ticks;
    uint32_t time_now_msec;
    uint32_t time_target;
    uint32_t err_code;

    if (m_gpio_trigger_enabled)
    {
        return;
    }
    // Round up to nearest second to next 250 ms to start toggling.
    // If the receiver has received a valid sync packet within this time, the GPIO toggling polarity will be the same.

    time_now_ticks = ts_timestamp_get_ticks_u64();
    time_now_msec = TIME_SYNC_TIMESTAMP_TO_USEC(time_now_ticks) / 1000;

    time_target = TIME_SYNC_MSEC_TO_TICK(time_now_msec) + (250 * 2);
    time_target = (time_target / 250) * 250;

    err_code = ts_set_trigger(time_target, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
    APP_ERROR_CHECK(err_code);

    nrf_gpiote_task_set(NRF_GPIOTE_TASKS_CLR_3);

    m_gpio_trigger_enabled = true;
}
static void ts_gpio_trigger_disable(void)
{
    m_gpio_trigger_enabled = false;
}

static void ts_evt_callback(const ts_evt_t* evt)
{
    APP_ERROR_CHECK_BOOL(evt != NULL);

    switch (evt->type)
    {

        case TS_EVT_DESYNCHRONIZED:
            ts_gpio_trigger_disable();
            break;
        case TS_EVT_SYNCHRONIZED:
            ts_gpio_trigger_enable();
            break;
        case TS_EVT_TRIGGERED:
            
            if (m_gpio_trigger_enabled)
            {
                uint32_t tick_target;

                tick_target = evt->params.triggered.tick_target + 2;
            
                uint32_t err_code = ts_set_trigger(tick_target, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // Ensure pin is low when triggering is stopped
                nrf_gpiote_task_set(NRF_GPIOTE_TASKS_CLR_3);
            }
            break;
        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
}

uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xF8, 0xF8, 0xF8, 0xF8};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.retransmit_delay         = 600;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack       = false;

    err_code = nrf_esb_init(&nrf_esb_config);

    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, NRF_ESB_PIPE_COUNT);
    VERIFY_SUCCESS(err_code);

    return err_code;
}

static void sync_timer_init(void)
{
    uint32_t err_code;
    m_gpio_trigger_enabled = false; // in bsp_event_handler

    #if defined(BOARD_PCA10040)
    nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(0, 24), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
    nrf_gpiote_task_enable(3);
#elif defined(BOARD_PCA10056)
    nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(1, 11), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
    nrf_gpiote_task_enable(3);
#else
#warning Debug pin not set
#endif

     ts_init_t init_ts =
    {
        .high_freq_timer[0] = NRF_TIMER3,
        .high_freq_timer[1] = NRF_TIMER4,
        .egu                = NRF_EGU3,
        .egu_irq_type       = SWI3_EGU3_IRQn,
        .evt_handler        = ts_evt_callback,
    };
    err_code = ts_init(&init_ts);
    APP_ERROR_CHECK(err_code);

    err_code = ts_enable();
    APP_ERROR_CHECK(err_code);
    //ts_gpio_trigger_enable();
}

//send sync packet periodically (Period : SYNC_INTERVAL)
static void sync_pkt_send_timer_handler(void * p_context)
{
    if(!urllc_pkt_in_progress){
     
      tx_payload.length = sizeof(sync_pkt_t);               //set tx_payload length packet length
      tx_payload.noack = true;                              //sync packet is not required ack
      sync_pkt_t *p_pkt;
      p_pkt = (sync_pkt_t*)malloc(sizeof(sync_pkt_t));
      p_pkt->header.message_id = SYNC_PKT;
      take_sync_timer_val(p_pkt);
      //nrf_gpio_pin_set(TEST_PIN); 
      memcpy(tx_payload.data,p_pkt,sizeof(sync_pkt_t));
      if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
      {
           //NRF_LOG_INFO("Sending sync packet succeed");
      }        
      else
      {
           //NRF_LOG_WARNING("Sending sync failed");
      }
      free(p_pkt);
    }
    //nrf_gpio_pin_clear(TEST_PIN);
}

static void ch_pkt_send_timer_handler(void * p_context){
    uint32_t err_code;
    if(!first_ch_target_set){
      
      //current counter val + 5 sec is the ch_tick_target 
      first_ch_target = ts_timestamp_get_counter_val_u32() + 10000; 
      err_code = ch_set_trigger(first_ch_target);
      APP_ERROR_CHECK(err_code);
      first_ch_target_set = true;
    }
    
    if(ch_pkt_count <= CH_PKT_ATTEMP){
      tx_payload.length = sizeof(ch_pkt);
      tx_payload.noack = true;
      ch_pkt *p_ch_pkt;
      p_ch_pkt = (ch_pkt*)malloc(sizeof(ch_pkt));
      p_ch_pkt->header.message_id = CH_PKT;
      p_ch_pkt -> ch_tick = first_ch_target;
      memcpy(tx_payload.data,p_ch_pkt,sizeof(ch_pkt));
     if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
      {
           NRF_LOG_INFO("Sending ch packet succeed");
      }        
      else
      {
           NRF_LOG_WARNING("Sending ch failed");
      }
      free(p_ch_pkt);
      ch_pkt_count++;
      err_code = app_timer_start(chn_pkt_send_timer, APP_TIMER_TICKS(CH_TRIAL_INTERVAL), NULL);
      APP_ERROR_CHECK(err_code);
    }
    else{
      err_code = app_timer_stop(chn_pkt_send_timer);
      APP_ERROR_CHECK(err_code);
    }
    //restart this single shot timer for CH_PKT_ATTEMP(times)       
}
/*
 *@brief Create timers for sending SYNC pkt and CH packet
 * */
static void create_timers()
{
    ret_code_t err_code;

    // Create timers
    err_code = app_timer_create(&sync_pkt_send_timer,
                                APP_TIMER_MODE_REPEATED,
                                sync_pkt_send_timer_handler);
    err_code = app_timer_create(&chn_pkt_send_timer,
                                 APP_TIMER_MODE_SINGLE_SHOT,
                                 ch_pkt_send_timer_handler);

    APP_ERROR_CHECK(err_code);
}
/**@brief Function starting the internal LFCLK oscillator.
 *
 * @details This is needed by RTC1 which is used by the Application Timer
 *          (When SoftDevice is enabled the LFCLK is always running and this is not needed).
 */
static void lfclk_request(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}
int main(void)
{
    ret_code_t ret;

    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };
    ret = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(ret);
    lfclk_request(); 

    while(!nrf_drv_clock_lfclk_is_running())
    {
        /* Just waiting */
    }

    ret = app_timer_init();
    APP_ERROR_CHECK(ret);

    init_bsp();

#if NRF_CLI_ENABLED
    init_cli();
#endif

    app_usbd_serial_num_generate();
    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);
    NRF_LOG_INFO("USBD CDC ACM example started.");

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");
        app_usbd_enable();
        app_usbd_start();
    }
    /*esb tx*/
    ret_code_t err_code;
    //gpio_init();
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    //NRF_LOG_DEFAULT_BACKENDS_INIT();
    
    clocks_start();   
     

    err_code = esb_init();
    APP_ERROR_CHECK(err_code);
    sync_timer_init();
    //err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    create_timers();
    //send sync packet each (SYNC_INTERVAL)ms using app timer
    err_code = app_timer_start(sync_pkt_send_timer, APP_TIMER_TICKS(SYNC_INTERVAL), NULL);
    APP_ERROR_CHECK(err_code);
    //send CH packet each CH_TRIAL_INTERVAL(ms) using app timer
    //err_code = app_timer_start(chn_pkt_send_timer, APP_TIMER_TICKS(CH_TRIAL_INTERVAL), NULL);
    //APP_ERROR_CHECK(err_code);

    
    NRF_LOG_INFO("Enhanced ShockBurst Transmitter Example started.");
    NRF_LOG_FLUSH();
   
    while (true)
    {
        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
        /*if(m_send_flag)
        {
            static int  frame_counter;

            size_t size = sprintf(m_tx_buffer, "Hello USB CDC FA demo: %u\r\n", frame_counter);

            ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, m_tx_buffer, size);
            if (ret == NRF_SUCCESS)
            {
                ++frame_counter;
            }*/
         
         //nrf_delay_us(50000);
        
        
#if NRF_CLI_ENABLED
        nrf_cli_process(&m_cli_uart);
#endif

        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
        /* Sleep CPU only if there was no interrupt since last loop processing */
        __WFE();
    }
}

/** @} */