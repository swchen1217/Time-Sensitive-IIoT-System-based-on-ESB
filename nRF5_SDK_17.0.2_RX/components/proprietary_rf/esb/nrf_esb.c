/**
 * Copyright (c) 2016 - 2020, Nordic Semiconductor ASA
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

#include "nrf_error.h"
#include "nrf_esb.h"
#include "nrf_esb_error_codes.h"
#include "nrf_gpio.h"
#include <string.h>
#include <stddef.h>
#include "sdk_common.h"
#include "sdk_macros.h"
#include "app_util.h"
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrfx_ppi.h"
#include "nrf_atomic.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define BIT_MASK_UINT_8(x) (0xFF >> (8 - (x)))

// Constant parameters
#define RX_WAIT_FOR_ACK_TIMEOUT_US_2MBPS        (48)        /**< 2 Mb RX wait for acknowledgment time-out value. Smallest reliable value - 43. */
#define RX_WAIT_FOR_ACK_TIMEOUT_US_1MBPS        (73)        /**< 1 Mb RX wait for acknowledgment time-out value. Smallest reliable value - 68. */
#define RX_WAIT_FOR_ACK_TIMEOUT_US_250KBPS      (250)       /**< 250 Kb RX wait for acknowledgment time-out value. */
#define RX_WAIT_FOR_ACK_TIMEOUT_US_1MBPS_BLE    (73)        /**< 1 Mb RX wait for acknowledgment time-out (combined with BLE). Smallest reliable value - 68.*/
#define RETRANSMIT_DELAY_US_OFFSET              (62)        /**< Never retransmit before the wait for ack time plus this offset. */

// Interrupt flags
#define     NRF_ESB_INT_TX_SUCCESS_MSK          0x01        /**< Interrupt mask value for TX success. */
#define     NRF_ESB_INT_TX_FAILED_MSK           0x02        /**< Interrupt mask value for TX failure. */
#define     NRF_ESB_INT_RX_DATA_RECEIVED_MSK    0x04        /**< Interrupt mask value for RX_DR. */

#define     NRF_ESB_PID_RESET_VALUE             0xFF        /**< Invalid PID value which is guaranteed to not collide with any valid PID value. */
#define     NRF_ESB_PID_MAX                     3           /**< Maximum value for PID. */
#define     NRF_ESB_CRC_RESET_VALUE             0xFFFF      /**< CRC reset value. */




// Internal Enhanced ShockBurst module state.
typedef enum {
    NRF_ESB_STATE_IDLE,                                     /**< Module idle. */
    NRF_ESB_STATE_PTX_TX,                                   /**< Module transmitting without acknowledgment. */
    NRF_ESB_STATE_PTX_TX_ACK,                               /**< Module transmitting with acknowledgment. */
    NRF_ESB_STATE_PTX_RX_ACK,                               /**< Module transmitting with acknowledgment and reception of payload with the acknowledgment response. */
    NRF_ESB_STATE_PRX,                                      /**< Module receiving packets without acknowledgment. */
    NRF_ESB_STATE_PRX_SEND_ACK,                             /**< Module transmitting acknowledgment in RX mode. */
} nrf_esb_mainstate_t;


#define DISABLE_RF_IRQ()      NVIC_DisableIRQ(RADIO_IRQn)
#define ENABLE_RF_IRQ()       NVIC_EnableIRQ(RADIO_IRQn)

#define _RADIO_SHORTS_COMMON ( RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | \
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk | RADIO_SHORTS_DISABLED_RSSISTOP_Msk )

#define VERIFY_PAYLOAD_LENGTH(p)                            \
do                                                          \
{                                                           \
    if (p->length == 0 ||                                   \
       p->length > NRF_ESB_MAX_PAYLOAD_LENGTH ||            \
       (m_config_local.protocol == NRF_ESB_PROTOCOL_ESB &&  \
        p->length > m_config_local.payload_length))         \
    {                                                       \
        return NRF_ERROR_INVALID_LENGTH;                    \
    }                                                       \
}while (0)


/* @brief Structure holding pipe info PID and CRC and acknowledgment payload. */
typedef struct
{
    uint16_t    crc;                                      /**< CRC value of the last received packet (Used to detect retransmits). */
    uint8_t     pid;                                      /**< Packet ID of the last received packet (Used to detect retransmits). */
    bool        ack_payload;                              /**< Flag indicating the state of the transmission of acknowledgment payloads. */
} pipe_info_t;


/* @brief Structure used by the PRX to organize ACK payloads for multiple pipes. */ 
typedef struct
{
    nrf_esb_payload_t * p_payload;                        /**< Pointer to the ACK payload. */
    bool                in_use;                           /**< Value used to determine if the current payload pointer is used. */
    struct nrf_esb_payload_random_access_buf_wrapper_t * p_next; /**< Pointer to the next ACK payload queued on the same pipe. */
} nrf_esb_payload_random_access_buf_wrapper_t;


/* @brief  First-in, first-out queue of payloads to be transmitted. */
typedef struct
{
    nrf_esb_payload_t * p_payload[NRF_ESB_TX_FIFO_SIZE];  /**< Pointer to the actual queue. */
    uint32_t            entry_point;                      /**< Current start of queue. */
    uint32_t            exit_point;                       /**< Current end of queue. */
    uint32_t            count;                            /**< Current number of elements in the queue. */
} nrf_esb_payload_tx_fifo_t;


/* @brief First-in, first-out queue of received payloads. */
typedef struct
{
    nrf_esb_payload_t * p_payload[NRF_ESB_RX_FIFO_SIZE];  /**< Pointer to the actual queue. */
    uint32_t            entry_point;                      /**< Current start of queue. */
    uint32_t            exit_point;                       /**< Current end of queue. */
    uint32_t            count;                            /**< Current number of elements in the queue. */
    
} nrf_esb_payload_rx_fifo_t;


static uint8_t          m_sync_pkt_ringbuf[10][sizeof(sync_pkt_t)];
static nrf_atomic_u32_t m_sync_pkt_ringbuf_idx;
static uint32_t cur_counter_val;

typedef struct
{
    uint8_t                 rf_chn;          /** RF Channel [0-80] */
    uint8_t                 rf_addr[5];      /** 5-byte RF address */
    //nrf_ppi_channel_t       ppi_chns[5];     /** PPI channels */
    nrf_ppi_channel_t       ppi_chns[6];
    nrf_ppi_channel_group_t ppi_chg;        /** PPI Channel Group */
    NRF_TIMER_Type *        high_freq_timer[2]; /** 16 MHz timer (e.g. NRF_TIMER2). NOTE: debug toggling only available if TIMER3 or TIMER4 is used for high_freq_timer[0]*/
    NRF_EGU_Type   *        egu;
    IRQn_Type               egu_irq_type;

} ts_params_t;


static ts_params_t       m_params;
static bool m_synchronized = false;



#if   defined ( __CC_ARM )
#define TX_CHAIN_DELAY_PRESCALER_0 (699 - 235)
#elif defined ( __ICCARM__ )
#define TX_CHAIN_DELAY_PRESCALER_0 (703 - 235)
#elif defined ( __GNUC__ )
#define TX_CHAIN_DELAY_PRESCALER_0 (704 - 237)
#endif


#if SYNC_TIMER_PRESCALER == 0
#define TX_CHAIN_DELAY TX_CHAIN_DELAY_PRESCALER_0
#else
#error Invalid prescaler value
#endif


#if TIME_SYNC_TX_OFFSET_REALIGN_TIMEOUT == 0
/*
 * If the master time is already > TIME_SYNC_TIMER_MAX_VAL, or the time it takes
 * to call ppi_sync_timer_adjust_enable() for the slave will cause it to become greater than TIME_SYNC_TIMER_MAX_VAL,
 * the counter in the master will be increased and therefore the current value is no longer correct.
 *
 * This is a "safety zone" in order to avoid this behavior
 */
#define CHECK_INVALID_TIMING(timer_val) ((timer_val + TX_CHAIN_DELAY) > (TIME_SYNC_TIMER_MAX_VAL - 10000))
#else
// Not needed when TX interval is actively adjusted
#define CHECK_INVALID_TIMING(timer_val) (false)
#endif

/**@brief Enhanced ShockBurst address.
 *
 * Enhanced ShockBurst addresses consist of a base address and a prefix
 *          that is unique for each pipe. See @ref esb_addressing in the ESB user
 *          guide for more information.
*/
typedef struct
{
    uint8_t base_addr_p0[4];        /**< Base address for pipe 0 encoded in big endian. */
    uint8_t base_addr_p1[4];        /**< Base address for pipe 1-7 encoded in big endian. */
    uint8_t pipe_prefixes[8];       /**< Address prefix for pipe 0 to 7. */
    uint8_t num_pipes;              /**< Number of pipes available. */
    uint8_t addr_length;            /**< Length of the address including the prefix. */
    uint8_t rx_pipes_enabled;       /**< Bitfield for enabled pipes. */
    uint8_t rf_channel;             /**< Channel to use (must be between 0 and 100). */
} nrf_esb_address_t;


// Module state
static bool                         m_esb_initialized           = false;
static volatile nrf_esb_mainstate_t m_nrf_esb_mainstate         = NRF_ESB_STATE_IDLE;
static nrf_esb_payload_t          * mp_current_payload;

static nrf_esb_event_handler_t      m_event_handler;

// Address parameters
__ALIGN(4) static nrf_esb_address_t m_esb_addr = NRF_ESB_ADDR_DEFAULT;

// RF parameters
static nrf_esb_config_t             m_config_local;

// TX FIFO
static nrf_esb_payload_t            m_tx_fifo_payload[NRF_ESB_TX_FIFO_SIZE];
static nrf_esb_payload_tx_fifo_t    m_tx_fifo;

// RX FIFO
static nrf_esb_payload_t            m_rx_fifo_payload[NRF_ESB_RX_FIFO_SIZE];
static nrf_esb_payload_rx_fifo_t    m_rx_fifo;

// Payload buffers
static  uint8_t                     m_tx_payload_buffer[NRF_ESB_MAX_PAYLOAD_LENGTH + 3];
static  uint8_t                     m_rx_payload_buffer[NRF_ESB_MAX_PAYLOAD_LENGTH + 3];

// Random access buffer variables for better ACK payload handling
nrf_esb_payload_random_access_buf_wrapper_t m_ack_pl_container[NRF_ESB_TX_FIFO_SIZE];
nrf_esb_payload_random_access_buf_wrapper_t * m_ack_pl_container_entry_point_pr_pipe[NRF_ESB_PIPE_COUNT];

// Run time variables
static volatile uint32_t            m_interrupt_flags = 0;
static uint8_t                      m_pids[NRF_ESB_PIPE_COUNT];
static pipe_info_t                  m_rx_pipe_info[NRF_ESB_PIPE_COUNT];
static volatile uint32_t            m_retransmits_remaining;
static volatile uint32_t            m_last_tx_attempts;
static volatile uint32_t            m_wait_for_ack_timeout_us;
static volatile uint32_t            m_radio_shorts_common = _RADIO_SHORTS_COMMON;

// These function pointers are changed dynamically, depending on protocol configuration and state.
static void (*on_radio_disabled)(void) = 0;
static void (*on_radio_end)(void) = 0;
static void (*update_rf_payload_format)(uint32_t payload_length) = 0;


// The following functions are assigned to the function pointers above.
static void on_radio_disabled_tx_noack(void);
static void on_radio_disabled_tx(void);
static void on_radio_disabled_tx_wait_for_ack(void);
static void on_radio_disabled_rx(void);
static void on_radio_disabled_rx_ack(void);


//time sync related parameters
static ts_evt_t m_trigger_evt = {.type = TS_EVT_TRIGGERED};
static nrf_atomic_flag_t m_timer_update_in_progress = false;
static nrf_atomic_u32_t  mp_curr_adj_pkt;
//static nrf_atomic_u32_t  m_master_counter = 0;
static uint32_t  m_master_counter = 0;

static ts_evt_handler_t m_callback;
static nrf_atomic_u32_t m_tick_target;
static nrf_atomic_u32_t m_sync_packet_count = 0;
static nrf_atomic_u32_t m_used_packet_count = 0;
static nrf_atomic_u32_t m_last_sync = 0;
// The chn hopping related function and parameters ¡@
static bool first_ch_target_set = false;                //this flag will be true when first channel hopping tick target is set
static uint32_t ch_current_target;
//static uint32_t chn_map[16] = {75, 78, 28, 47, 73, 71, 56, 15, 58, 13, 12, 52, 24, 27, 54, 35};
//static uint32_t chn_map[16] = {42, 67, 70, 20, 22, 53, 32, 39, 50, 46, 5, 18, 66, 74, 64, 59};
static uint32_t chn_map[16] = {25, 68, 30, 55, 80, 43, 14, 63, 79, 57, 2, 8, 21, 10, 41, 61};
//static uint32_t chn_map[16] = {31, 3, 44, 19, 7, 26, 72, 62, 60, 29, 69, 48, 34, 4, 49, 36};
//static uint32_t chn_map[15] = {45, 37, 17, 40, 16, 51, 38, 9, 11, 23, 77, 76, 33, 65, 6};
static uint32_t chn_index = 0;        //channel map index
static uint32_t ch_set_trigger(uint32_t ch_tick_target);
static void ch_set_next_target(void);        



//the following is time sync related function
static bool sync_timer_offset_compensate(sync_pkt_t * p_pkt);
static void ppi_sync_timer_adjust_enable(void);
static void ppi_radio_rx_configure(void);


#define NRF_ESB_ADDR_UPDATE_MASK_BASE0          (1 << 0)    /*< Mask value to signal updating BASE0 radio address. */
#define NRF_ESB_ADDR_UPDATE_MASK_BASE1          (1 << 1)    /*< Mask value to signal updating BASE1 radio address. */
#define NRF_ESB_ADDR_UPDATE_MASK_PREFIX         (1 << 2)    /*< Mask value to signal updating radio prefixes. */


// Function to do bytewise bit-swap on an unsigned 32-bit value
static uint32_t bytewise_bit_swap(uint8_t const * p_inp)
{
#if __CORTEX_M == (0x04U)
    uint32_t inp = (*(uint32_t*)p_inp);
    return __REV((uint32_t)__RBIT(inp)); //lint -esym(628, __rev) -esym(526, __rev) -esym(628, __rbit) -esym(526, __rbit) */
#else
    uint32_t inp = (p_inp[3] << 24) | (p_inp[2] << 16) | (p_inp[1] << 8) | (p_inp[0]);
    inp = (inp & 0xF0F0F0F0) >> 4 | (inp & 0x0F0F0F0F) << 4;
    inp = (inp & 0xCCCCCCCC) >> 2 | (inp & 0x33333333) << 2;
    inp = (inp & 0xAAAAAAAA) >> 1 | (inp & 0x55555555) << 1;
    return inp;
#endif
}


// Convert a base address from nRF24L format to nRF5 format
static uint32_t addr_conv(uint8_t const* p_addr)
{
    return __REV(bytewise_bit_swap(p_addr)); //lint -esym(628, __rev) -esym(526, __rev) */
}

#ifdef NRF52832_XXAA
static ret_code_t apply_address_workarounds()
{
    if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004200) //Check if the device is an nRF52832 Rev. 1.
    {
        // Workaround for nRF52832 Rev 1 erratas
        //  Set up radio parameters.
        NRF_RADIO->MODECNF0 = (NRF_RADIO->MODECNF0 & ~RADIO_MODECNF0_RU_Msk) | RADIO_MODECNF0_RU_Default << RADIO_MODECNF0_RU_Pos;

        // Workaround for nRF52832 Rev 1 Errata 102 and nRF52832 Rev 1 Errata 106. This will reduce sensitivity by 3dB.
        *((volatile uint32_t *)0x40001774) = (*((volatile uint32_t *)0x40001774) & 0xFFFFFFFE) | 0x01000000;
    }

    if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004500)//Check if the device is an nRF52832 Rev. 2.
    {
        /*
        Workaround for nRF52832 Rev 2 Errata 143
        Check if the most significant bytes of address 0 (including prefix) match those of another address.
        It's recommended to use a unique address 0 since this will avoid the 3dBm penalty incurred from the workaround.
        */
        uint32_t base_address_mask = m_esb_addr.addr_length == 5 ? 0xFFFF0000 : 0xFF000000;

        // Load the two addresses before comparing them to ensure defined ordering of volatile accesses.
        uint32_t addr0 = NRF_RADIO->BASE0 & base_address_mask;
        uint32_t addr1 = NRF_RADIO->BASE1 & base_address_mask;
        if (addr0 == addr1)
        {
            uint32_t prefix0 = NRF_RADIO->PREFIX0 & 0x000000FF;
            uint32_t prefix1 = (NRF_RADIO->PREFIX0 & 0x0000FF00) >> 8;
            uint32_t prefix2 = (NRF_RADIO->PREFIX0 & 0x00FF0000) >> 16;
            uint32_t prefix3 = (NRF_RADIO->PREFIX0 & 0xFF000000) >> 24;
            uint32_t prefix4 = NRF_RADIO->PREFIX1 & 0x000000FF;
            uint32_t prefix5 = (NRF_RADIO->PREFIX1 & 0x0000FF00) >> 8;
            uint32_t prefix6 = (NRF_RADIO->PREFIX1 & 0x00FF0000) >> 16;
            uint32_t prefix7 = (NRF_RADIO->PREFIX1 & 0xFF000000) >> 24;

            if (prefix0 == prefix1 || prefix0 == prefix2 || prefix0 == prefix3 || prefix0 == prefix4 ||
                prefix0 == prefix5 || prefix0 == prefix6 || prefix0 == prefix7)
            {
                // This will cause a 3dBm sensitivity loss, avoid using such address combinations if possible.
                *(volatile uint32_t *) 0x40001774 = ((*(volatile uint32_t *) 0x40001774) & 0xfffffffe) | 0x01000000;
            }
        }
    }
    return NRF_SUCCESS;
}
#endif


static void update_rf_payload_format_esb_dpl(uint32_t payload_length)
{
#if (NRF_ESB_MAX_PAYLOAD_LENGTH <= 32)
    // Using 6 bits for length
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) |
                       (6 << RADIO_PCNF0_LFLEN_Pos) |
                       (3 << RADIO_PCNF0_S1LEN_Pos) ;
#else
    // Using 8 bits for length
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) |
                       (8 << RADIO_PCNF0_LFLEN_Pos) |
                       (3 << RADIO_PCNF0_S1LEN_Pos) ;
#endif
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled    << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big          << RADIO_PCNF1_ENDIAN_Pos)  |
                       ((m_esb_addr.addr_length - 1)    << RADIO_PCNF1_BALEN_Pos)   |
                       (0                               << RADIO_PCNF1_STATLEN_Pos) |
                       (NRF_ESB_MAX_PAYLOAD_LENGTH      << RADIO_PCNF1_MAXLEN_Pos);
}


static void update_rf_payload_format_esb(uint32_t payload_length)
{
    NRF_RADIO->PCNF0 = (1 << RADIO_PCNF0_S0LEN_Pos) |
                       (0 << RADIO_PCNF0_LFLEN_Pos) |
                       (1 << RADIO_PCNF0_S1LEN_Pos);

    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled    << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big          << RADIO_PCNF1_ENDIAN_Pos)  |
                       ((m_esb_addr.addr_length - 1)    << RADIO_PCNF1_BALEN_Pos)   |
                       (payload_length                  << RADIO_PCNF1_STATLEN_Pos) |
                       (payload_length                  << RADIO_PCNF1_MAXLEN_Pos);
}


static void update_radio_addresses(uint8_t update_mask)
{
    if ((update_mask & NRF_ESB_ADDR_UPDATE_MASK_BASE0) != 0)
    {
        NRF_RADIO->BASE0 = addr_conv(m_esb_addr.base_addr_p0);
    }

    if ((update_mask & NRF_ESB_ADDR_UPDATE_MASK_BASE1) != 0)
    {
        NRF_RADIO->BASE1 = addr_conv(m_esb_addr.base_addr_p1);
    }

    if ((update_mask & NRF_ESB_ADDR_UPDATE_MASK_PREFIX) != 0)
    {
        NRF_RADIO->PREFIX0 = bytewise_bit_swap(&m_esb_addr.pipe_prefixes[0]);
        NRF_RADIO->PREFIX1 = bytewise_bit_swap(&m_esb_addr.pipe_prefixes[4]);
    }
}


static void update_radio_tx_power()
{
    NRF_RADIO->TXPOWER = m_config_local.tx_output_power << RADIO_TXPOWER_TXPOWER_Pos;
}


static bool update_radio_bitrate()
{
    NRF_RADIO->MODE = m_config_local.bitrate << RADIO_MODE_MODE_Pos;

    switch (m_config_local.bitrate)
    {
        case NRF_ESB_BITRATE_2MBPS:
#ifdef RADIO_MODE_MODE_Ble_2Mbit
        case NRF_ESB_BITRATE_2MBPS_BLE:
#endif
            m_wait_for_ack_timeout_us = RX_WAIT_FOR_ACK_TIMEOUT_US_2MBPS;
            break;

        case NRF_ESB_BITRATE_1MBPS:
            m_wait_for_ack_timeout_us = RX_WAIT_FOR_ACK_TIMEOUT_US_1MBPS;
            break;

#ifdef RADIO_MODE_MODE_Nrf_250Kbit
        case NRF_ESB_BITRATE_250KBPS:
            m_wait_for_ack_timeout_us = RX_WAIT_FOR_ACK_TIMEOUT_US_250KBPS;
            break;
#endif

        case NRF_ESB_BITRATE_1MBPS_BLE:
            m_wait_for_ack_timeout_us = RX_WAIT_FOR_ACK_TIMEOUT_US_1MBPS_BLE;
            break;

        default:
            // Should not be reached
            return false;
    }

    // Ensure that we do not attempt retransmitting before ack timeout.
    if (m_config_local.retransmit_delay < m_wait_for_ack_timeout_us + RETRANSMIT_DELAY_US_OFFSET)
    {
        m_config_local.retransmit_delay = m_wait_for_ack_timeout_us + RETRANSMIT_DELAY_US_OFFSET;
    }

    return true;
}


static bool update_radio_protocol()
{
    switch (m_config_local.protocol)
    {
        case NRF_ESB_PROTOCOL_ESB_DPL:
            update_rf_payload_format = update_rf_payload_format_esb_dpl;
            break;

        case NRF_ESB_PROTOCOL_ESB:
            update_rf_payload_format = update_rf_payload_format_esb;
            break;

        default:
            // Should not be reached
            return false;
    }
    return true;
}


static bool update_radio_crc()
{
    switch(m_config_local.crc)
    {
        case NRF_ESB_CRC_16BIT:
            NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value
            NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1
            break;

        case NRF_ESB_CRC_8BIT:
            NRF_RADIO->CRCINIT = 0xFFUL;        // Initial value
            NRF_RADIO->CRCPOLY = 0x107UL;       // CRC poly: x^8+x^2^x^1+1
            break;

        case NRF_ESB_CRC_OFF:
            NRF_RADIO->CRCINIT = 0x00UL;
            NRF_RADIO->CRCPOLY = 0x00UL;
            break;

        default:
            return false;
    }
    NRF_RADIO->CRCCNF = m_config_local.crc << RADIO_CRCCNF_LEN_Pos;
    return true;
}


static bool update_radio_parameters()
{
    bool params_valid = true;
    update_radio_tx_power();
    params_valid &= update_radio_bitrate();
    params_valid &= update_radio_protocol();
    params_valid &= update_radio_crc();
    update_rf_payload_format(m_config_local.payload_length);
    return params_valid;
}


static void reset_fifos()
{
    m_tx_fifo.entry_point = 0;
    m_tx_fifo.exit_point  = 0;
    m_tx_fifo.count       = 0;

    m_rx_fifo.entry_point = 0;
    m_rx_fifo.exit_point  = 0;
    m_rx_fifo.count       = 0;
}


static void initialize_fifos()
{
    reset_fifos();

    for (int i = 0; i < NRF_ESB_TX_FIFO_SIZE; i++)
    {
        m_tx_fifo.p_payload[i] = &m_tx_fifo_payload[i];
    }

    for (int i = 0; i < NRF_ESB_RX_FIFO_SIZE; i++)
    {
        m_rx_fifo.p_payload[i] = &m_rx_fifo_payload[i];
    }
    
    for (int i = 0; i < NRF_ESB_TX_FIFO_SIZE; i++)
    {
        m_ack_pl_container[i].p_payload = &m_tx_fifo_payload[i];
        m_ack_pl_container[i].in_use = false;
        m_ack_pl_container[i].p_next = 0;
    }
    for (int i = 0; i < NRF_ESB_PIPE_COUNT; i++)
    {
        m_ack_pl_container_entry_point_pr_pipe[i] = 0;
    }
}


uint32_t nrf_esb_skip_tx()
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);
    VERIFY_TRUE(m_tx_fifo.count > 0, NRF_ERROR_BUFFER_EMPTY);

    DISABLE_RF_IRQ();

    m_tx_fifo.count--;
    if (++m_tx_fifo.exit_point >= NRF_ESB_TX_FIFO_SIZE)
    {
        m_tx_fifo.exit_point = 0;
    }

    ENABLE_RF_IRQ();

    return NRF_SUCCESS;
}

/** @brief  Function to push the content of the rx_buffer to the RX FIFO.
 *
 *  The module will point the register NRF_RADIO->PACKETPTR to a buffer for receiving packets.
 *  After receiving a packet the module will call this function to copy the received data to
 *  the RX FIFO.
 *
 *  @param  pipe Pipe number to set for the packet.
 *  @param  pid  Packet ID.
 *
 *  @retval true   Operation successful.
 *  @retval false  Operation failed.
 */
static bool rx_fifo_push_rfbuf(uint8_t pipe, uint8_t pid)
{
    //isr_debug.p2++;

    if (m_rx_fifo.count < NRF_ESB_RX_FIFO_SIZE)
    {
        if (m_config_local.protocol == NRF_ESB_PROTOCOL_ESB_DPL)
        {
            if (m_rx_payload_buffer[0] > NRF_ESB_MAX_PAYLOAD_LENGTH)
            {
                return false;
            }

            m_rx_fifo.p_payload[m_rx_fifo.entry_point]->length = m_rx_payload_buffer[0];
        }
        else if (m_config_local.mode == NRF_ESB_MODE_PTX)
        {
            // Received packet is an acknowledgment
            m_rx_fifo.p_payload[m_rx_fifo.entry_point]->length = 0;
        }
        else
        {
            m_rx_fifo.p_payload[m_rx_fifo.entry_point]->length = m_config_local.payload_length;
        }

        memcpy(m_rx_fifo.p_payload[m_rx_fifo.entry_point]->data, &m_rx_payload_buffer[2],
               m_rx_fifo.p_payload[m_rx_fifo.entry_point]->length);

        m_rx_fifo.p_payload[m_rx_fifo.entry_point]->pipe  = pipe;
        m_rx_fifo.p_payload[m_rx_fifo.entry_point]->rssi  = NRF_RADIO->RSSISAMPLE;
        m_rx_fifo.p_payload[m_rx_fifo.entry_point]->pid   = pid;
        m_rx_fifo.p_payload[m_rx_fifo.entry_point]->noack = !(m_rx_payload_buffer[1] & 0x01);
        if (++m_rx_fifo.entry_point >= NRF_ESB_RX_FIFO_SIZE)
        {
            m_rx_fifo.entry_point = 0;
        }
        m_rx_fifo.count++;
        //isr_debug.fifo_count =  m_rx_fifo.count;
        //isr_debug.p3++;

        return true;
    }
    //exit(0);
    return false;
}


static void sys_timer_init()
{
    // Configure the system timer with a 1 MHz base frequency
    NRF_ESB_SYS_TIMER->PRESCALER = 4;
    NRF_ESB_SYS_TIMER->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;
    NRF_ESB_SYS_TIMER->SHORTS    = TIMER_SHORTS_COMPARE1_CLEAR_Msk | TIMER_SHORTS_COMPARE1_STOP_Msk;
}


static void ppi_init()
{
    NRF_PPI->CH[NRF_ESB_PPI_TIMER_START].EEP = (uint32_t)&NRF_RADIO->EVENTS_READY;
    NRF_PPI->CH[NRF_ESB_PPI_TIMER_START].TEP = (uint32_t)&NRF_ESB_SYS_TIMER->TASKS_START;

    NRF_PPI->CH[NRF_ESB_PPI_TIMER_STOP].EEP  = (uint32_t)&NRF_RADIO->EVENTS_ADDRESS;
    NRF_PPI->CH[NRF_ESB_PPI_TIMER_STOP].TEP  = (uint32_t)&NRF_ESB_SYS_TIMER->TASKS_SHUTDOWN;

    NRF_PPI->CH[NRF_ESB_PPI_RX_TIMEOUT].EEP  = (uint32_t)&NRF_ESB_SYS_TIMER->EVENTS_COMPARE[0];
    NRF_PPI->CH[NRF_ESB_PPI_RX_TIMEOUT].TEP  = (uint32_t)&NRF_RADIO->TASKS_DISABLE;

    NRF_PPI->CH[NRF_ESB_PPI_TX_START].EEP    = (uint32_t)&NRF_ESB_SYS_TIMER->EVENTS_COMPARE[1];
    NRF_PPI->CH[NRF_ESB_PPI_TX_START].TEP    = (uint32_t)&NRF_RADIO->TASKS_TXEN;
}


static void start_tx_transaction()
{
    bool ack;

    m_last_tx_attempts = 1;
    // Prepare the payload
    mp_current_payload = m_tx_fifo.p_payload[m_tx_fifo.exit_point];


    switch (m_config_local.protocol)
    {
        case NRF_ESB_PROTOCOL_ESB:
            update_rf_payload_format(mp_current_payload->length);
            m_tx_payload_buffer[0] = mp_current_payload->pid;
            m_tx_payload_buffer[1] = 0;
            
            memcpy(&m_tx_payload_buffer[2], mp_current_payload->data, mp_current_payload->length);

            NRF_RADIO->SHORTS   = m_radio_shorts_common | RADIO_SHORTS_DISABLED_RXEN_Msk;
            NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk | RADIO_INTENSET_READY_Msk;

            // Configure the retransmit counter
            m_retransmits_remaining = m_config_local.retransmit_count;
            on_radio_disabled = on_radio_disabled_tx;
            m_nrf_esb_mainstate = NRF_ESB_STATE_PTX_TX_ACK;
            break;

        case NRF_ESB_PROTOCOL_ESB_DPL:
           ack = !mp_current_payload->noack || !m_config_local.selective_auto_ack;
            m_tx_payload_buffer[0] = mp_current_payload->length;
            m_tx_payload_buffer[1] = mp_current_payload->pid << 1;
            m_tx_payload_buffer[1] |= mp_current_payload->noack ? 0x00 : 0x01;

            memcpy(&m_tx_payload_buffer[2], mp_current_payload->data, mp_current_payload->length);


            // Handling ack if noack is set to false or if selective auto ack is turned off
            if (ack)
            {
                NRF_RADIO->SHORTS   = m_radio_shorts_common | RADIO_SHORTS_DISABLED_RXEN_Msk;
                NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk | RADIO_INTENSET_READY_Msk;

                // Configure the retransmit counter
                m_retransmits_remaining = m_config_local.retransmit_count;
                on_radio_disabled = on_radio_disabled_tx;
                m_nrf_esb_mainstate = NRF_ESB_STATE_PTX_TX_ACK;
            }
            else
            {
                NRF_RADIO->SHORTS   = m_radio_shorts_common;
                NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;
                on_radio_disabled   = on_radio_disabled_tx_noack;
                m_nrf_esb_mainstate = NRF_ESB_STATE_PTX_TX;
            }
            break;

        default:
            // Should not be reached
            break;
    }

    NRF_RADIO->TXADDRESS    = mp_current_payload->pipe;
    NRF_RADIO->RXADDRESSES  = 1 << mp_current_payload->pipe;

    NRF_RADIO->FREQUENCY    = m_esb_addr.rf_channel;
    NRF_RADIO->PACKETPTR    = (uint32_t)m_tx_payload_buffer;

    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);

    NRF_RADIO->EVENTS_ADDRESS = 0;
    NRF_RADIO->EVENTS_PAYLOAD = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;

    DEBUG_PIN_SET(DEBUGPIN4);
    NRF_RADIO->TASKS_TXEN  = 1;
}


static void on_radio_disabled_tx_noack()
{
    m_interrupt_flags |= NRF_ESB_INT_TX_SUCCESS_MSK;
    (void) nrf_esb_skip_tx();

    if (m_tx_fifo.count == 0)
    {
        m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;
        NVIC_SetPendingIRQ(ESB_EVT_IRQ);
    }
    else
    {
        NVIC_SetPendingIRQ(ESB_EVT_IRQ);
        start_tx_transaction();
    }
}


static void on_radio_disabled_tx()
{
    // Remove the DISABLED -> RXEN shortcut, to make sure the radio stays
    // disabled after the RX window
    NRF_RADIO->SHORTS           = m_radio_shorts_common;

    // Make sure the timer is started the next time the radio is ready,
    // and that it will disable the radio automatically if no packet is
    // received by the time defined in m_wait_for_ack_timeout_us
    NRF_ESB_SYS_TIMER->CC[0]    = m_wait_for_ack_timeout_us;
    NRF_ESB_SYS_TIMER->CC[1]    = m_config_local.retransmit_delay - 130;
    NRF_ESB_SYS_TIMER->TASKS_CLEAR = 1;
    NRF_ESB_SYS_TIMER->EVENTS_COMPARE[0] = 0;
    NRF_ESB_SYS_TIMER->EVENTS_COMPARE[1] = 0;

    NRF_PPI->CHENSET            = (1 << NRF_ESB_PPI_TIMER_START) |
                                  (1 << NRF_ESB_PPI_RX_TIMEOUT) |
                                  (1 << NRF_ESB_PPI_TIMER_STOP);
    NRF_PPI->CHENCLR            = (1 << NRF_ESB_PPI_TX_START);
    NRF_RADIO->EVENTS_END       = 0;

    if (m_config_local.protocol == NRF_ESB_PROTOCOL_ESB)
    {
        update_rf_payload_format(0);
    }

    NRF_RADIO->PACKETPTR        = (uint32_t)m_rx_payload_buffer;
    on_radio_disabled           = on_radio_disabled_tx_wait_for_ack;
    m_nrf_esb_mainstate         = NRF_ESB_STATE_PTX_RX_ACK;
}


static void on_radio_disabled_tx_wait_for_ack()
{
    // This marks the completion of a TX_RX sequence (TX with ACK)

    // Make sure the timer will not deactivate the radio if a packet is received
    NRF_PPI->CHENCLR = (1 << NRF_ESB_PPI_TIMER_START) |
                       (1 << NRF_ESB_PPI_RX_TIMEOUT)  |
                       (1 << NRF_ESB_PPI_TIMER_STOP);

    // If the radio has received a packet and the CRC status is OK
    if (NRF_RADIO->EVENTS_END && NRF_RADIO->CRCSTATUS != 0)
    {
        NRF_ESB_SYS_TIMER->TASKS_SHUTDOWN = 1;
        NRF_PPI->CHENCLR = (1 << NRF_ESB_PPI_TX_START);
        m_interrupt_flags |= NRF_ESB_INT_TX_SUCCESS_MSK;
        m_last_tx_attempts = m_config_local.retransmit_count - m_retransmits_remaining + 1;

        (void) nrf_esb_skip_tx();

        if (m_config_local.protocol != NRF_ESB_PROTOCOL_ESB && m_rx_payload_buffer[0] > 0)
        {
            if (rx_fifo_push_rfbuf((uint8_t)NRF_RADIO->TXADDRESS, m_rx_payload_buffer[1] >> 1))
            {
                m_interrupt_flags |= NRF_ESB_INT_RX_DATA_RECEIVED_MSK;
            }
        }

        if ((m_tx_fifo.count == 0) || (m_config_local.tx_mode == NRF_ESB_TXMODE_MANUAL))
        {
            m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;
            NVIC_SetPendingIRQ(ESB_EVT_IRQ);
        }
        else
        {
            NVIC_SetPendingIRQ(ESB_EVT_IRQ);
            start_tx_transaction();
        }
    }
    else
    {
        if (m_retransmits_remaining-- == 0)
        {
            NRF_ESB_SYS_TIMER->TASKS_SHUTDOWN = 1;
            NRF_PPI->CHENCLR = (1 << NRF_ESB_PPI_TX_START);
            // All retransmits are expended, and the TX operation is suspended
            m_last_tx_attempts = m_config_local.retransmit_count + 1;
            m_interrupt_flags |= NRF_ESB_INT_TX_FAILED_MSK;

            m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;
            NVIC_SetPendingIRQ(ESB_EVT_IRQ);
        }
        else
        {
            // There are still more retransmits left, TX mode should be
            // entered again as soon as the system timer reaches CC[1].
            NRF_RADIO->SHORTS = m_radio_shorts_common | RADIO_SHORTS_DISABLED_RXEN_Msk;
            update_rf_payload_format(mp_current_payload->length);
            NRF_RADIO->PACKETPTR = (uint32_t)m_tx_payload_buffer;
            on_radio_disabled = on_radio_disabled_tx;
            m_nrf_esb_mainstate = NRF_ESB_STATE_PTX_TX_ACK;
            NRF_ESB_SYS_TIMER->TASKS_START = 1;
            NRF_PPI->CHENSET = (1 << NRF_ESB_PPI_TX_START);
            if (NRF_ESB_SYS_TIMER->EVENTS_COMPARE[1])
            {
                NRF_RADIO->TASKS_TXEN = 1;
            }
        }
    }
}

static void clear_events_restart_rx(void)
{
    NRF_RADIO->SHORTS = m_radio_shorts_common;
    update_rf_payload_format(m_config_local.payload_length);
    NRF_RADIO->PACKETPTR = (uint32_t)m_rx_payload_buffer;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;

    while (NRF_RADIO->EVENTS_DISABLED == 0);
    //NRF_LOG_INFO("%d/n",counter++);

    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->SHORTS = m_radio_shorts_common | RADIO_SHORTS_DISABLED_TXEN_Msk;
    NRF_RADIO->TASKS_RXEN = 1;
}


static void on_radio_disabled_rx(void)
{
    bool            ack                = false;
    bool            retransmit_payload = false;
    bool            send_rx_event      = true;
    pipe_info_t *   p_pipe_info;

    /*if (NRF_RADIO->CRCSTATUS == 0)
    {
        clear_events_restart_rx();
        return;
    }*/

    if (m_rx_fifo.count >= NRF_ESB_RX_FIFO_SIZE)
    {
        clear_events_restart_rx();
        return;
    }

    p_pipe_info = &m_rx_pipe_info[NRF_RADIO->RXMATCH];
    if (NRF_RADIO->RXCRC             == p_pipe_info->crc &&
        (m_rx_payload_buffer[1] >> 1) == p_pipe_info->pid
       )
    {
        retransmit_payload = true;
        send_rx_event = false;
    }

    p_pipe_info->pid = m_rx_payload_buffer[1] >> 1;
    p_pipe_info->crc = NRF_RADIO->RXCRC;

    if ((m_config_local.selective_auto_ack == false) || ((m_rx_payload_buffer[1] & 0x01) == 1))
    {
        ack = true;
    }

    if (ack)
    {
        NRF_RADIO->SHORTS = m_radio_shorts_common | RADIO_SHORTS_DISABLED_RXEN_Msk;

        switch (m_config_local.protocol)
        {
            case NRF_ESB_PROTOCOL_ESB_DPL:
                {
                    if (m_tx_fifo.count > 0 && m_ack_pl_container_entry_point_pr_pipe[NRF_RADIO->RXMATCH] != 0)
                    {
                        mp_current_payload = m_ack_pl_container_entry_point_pr_pipe[NRF_RADIO->RXMATCH]->p_payload;

                        // Pipe stays in ACK with payload until TX FIFO is empty
                        // Do not report TX success on first ack payload or retransmit
                        if (p_pipe_info->ack_payload == true && !retransmit_payload)
                        {
                            uint32_t pipe = NRF_RADIO->RXMATCH;
                            m_ack_pl_container_entry_point_pr_pipe[pipe]->in_use = false;
                            m_ack_pl_container_entry_point_pr_pipe[pipe] = (nrf_esb_payload_random_access_buf_wrapper_t *)m_ack_pl_container_entry_point_pr_pipe[pipe]->p_next;
                            m_tx_fifo.count--;
                            if (m_tx_fifo.count > 0 && m_ack_pl_container_entry_point_pr_pipe[pipe] != 0)
                            {
                                 mp_current_payload = m_ack_pl_container_entry_point_pr_pipe[pipe]->p_payload;
                            }
                            else mp_current_payload = 0;

                            // ACK payloads also require TX_DS
                            // (page 40 of the 'nRF24LE1_Product_Specification_rev1_6.pdf').
                            m_interrupt_flags |= NRF_ESB_INT_TX_SUCCESS_MSK;
                        }

                        if(mp_current_payload != 0)
                        {
                            p_pipe_info->ack_payload = true;
                            update_rf_payload_format(mp_current_payload->length);
                            m_tx_payload_buffer[0] = mp_current_payload->length;
                            memcpy(&m_tx_payload_buffer[2],
                                   mp_current_payload->data,
                                   mp_current_payload->length);
                        }
                        else
                        {
                            p_pipe_info->ack_payload = false;
                            update_rf_payload_format(0);
                            m_tx_payload_buffer[0] = 0;
                        }
                    }
                    else
                    {
                        p_pipe_info->ack_payload = false;
                        update_rf_payload_format(0);
                        m_tx_payload_buffer[0] = 0;
                    }

                    m_tx_payload_buffer[1] = m_rx_payload_buffer[1];
                }
                break;

            case NRF_ESB_PROTOCOL_ESB:
                {
                    update_rf_payload_format(0);
                    m_tx_payload_buffer[0] = m_rx_payload_buffer[0];
                    m_tx_payload_buffer[1] = 0;
                }
                break;
        }

        m_nrf_esb_mainstate = NRF_ESB_STATE_PRX_SEND_ACK;
        NRF_RADIO->TXADDRESS = NRF_RADIO->RXMATCH;
        NRF_RADIO->PACKETPTR = (uint32_t)m_tx_payload_buffer;
        on_radio_disabled = on_radio_disabled_rx_ack;
    }
    else
    {
        clear_events_restart_rx();
    }

    //isr_debug.p1++;

    if (send_rx_event)
    {
        // Push the new packet to the RX buffer and trigger a received event if the operation was
        // successful.

        if (rx_fifo_push_rfbuf(NRF_RADIO->RXMATCH, p_pipe_info->pid))
        {
            
            m_interrupt_flags |= NRF_ESB_INT_RX_DATA_RECEIVED_MSK;
            NVIC_SetPendingIRQ(ESB_EVT_IRQ);

        }
    }
}


static void on_radio_disabled_rx_ack(void)
{
    NRF_RADIO->SHORTS = m_radio_shorts_common | RADIO_SHORTS_DISABLED_TXEN_Msk;
    update_rf_payload_format(m_config_local.payload_length);

    NRF_RADIO->PACKETPTR = (uint32_t)m_rx_payload_buffer;
    on_radio_disabled = on_radio_disabled_rx;

    m_nrf_esb_mainstate = NRF_ESB_STATE_PRX;
}


/**@brief Function for clearing pending interrupts.
 *
 * @param[in,out]   p_interrupts        Pointer to the value that holds the current interrupts.
 *
 * @retval  NRF_SUCCESS                     If the interrupts were cleared successfully.
 * @retval  NRF_ERROR_NULL                  If the required parameter was NULL.
 * @retval  NRF_INVALID_STATE               If the module is not initialized.
 */
static uint32_t nrf_esb_get_clear_interrupts(uint32_t * p_interrupts)
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);
    VERIFY_PARAM_NOT_NULL(p_interrupts);

    DISABLE_RF_IRQ();

    *p_interrupts = m_interrupt_flags;
    m_interrupt_flags = 0;

    ENABLE_RF_IRQ();

    return NRF_SUCCESS;
}

static sync_pkt_t * tx_buf_get(void)
{
    uint32_t idx;

    idx = nrf_atomic_u32_fetch_add(&m_sync_pkt_ringbuf_idx, 1) % ARRAY_SIZE(m_sync_pkt_ringbuf);

    return (sync_pkt_t *) m_sync_pkt_ringbuf[idx];
}


void RADIO_IRQHandler() {

  
    if (NRF_RADIO->EVENTS_READY && (NRF_RADIO->INTENSET & RADIO_INTENSET_READY_Msk)) {
      NRF_RADIO->EVENTS_READY = 0;
       //DEBUG_PIN_SET(DEBUGPIN1);
    }

    if (NRF_RADIO->EVENTS_END && (NRF_RADIO->INTENSET & RADIO_INTENSET_END_Msk)) {
       NRF_RADIO->EVENTS_END = 0;
       //DEBUG_PIN_SET(DEBUGPIN2);
       // Call the correct on_radio_end function, depending on the current protocol state
      if (on_radio_end) {
        on_radio_end();
      }
    }

    if (NRF_RADIO->EVENTS_DISABLED && (NRF_RADIO->INTENSET & RADIO_INTENSET_DISABLED_Msk)) {

      NRF_RADIO->EVENTS_DISABLED = 0;
      if (NRF_RADIO->CRCSTATUS == 0) {
        clear_events_restart_rx();
        return;
      }
      uint8_t message_id;
      message_id = m_rx_payload_buffer[2];
      //isr_debug.msg = message_id;
      //isr_debug.r0++;
      switch (message_id){
      case SYNC_PKT:
            {
              //isr_debug.r1++;
              //nrf_gpio_pin_set(TEST_PIN);
              bool adjustment_procedure_started;
              sync_pkt_t *p_pkt;
              p_pkt = (sync_pkt_t*)&m_rx_payload_buffer[2];
              //NRF_LOG_INFO("%u",p_pkt -> counter_val);
              adjustment_procedure_started = sync_timer_offset_compensate(p_pkt);
              //nrf_gpio_pin_clear(TEST_PIN);
              
              if (adjustment_procedure_started) {
                p_pkt = tx_buf_get();
              }
              
              
              clear_events_restart_rx();
            }
            break;

      case URLLC_DATA_PKT:
            //isr_debug.r2++;
            
            if (on_radio_disabled) 
            {
              //isr_debug.r3++; 
              on_radio_disabled();
            }
           
            break;
      case CH_PKT:
             
             {
            
                if(m_synchronized){
                  if (on_radio_disabled) 
                    {
                      //isr_debug.r3++; 
                      on_radio_disabled();
                    }

                    /*uint32_t err;
                    uint32_t ch_tick_target;
                    ch_pkt *p_pkt;
                    p_pkt = (ch_pkt*)&m_rx_payload_buffer[2];
                    ch_tick_target = p_pkt->ch_tick;
                    err = ch_set_trigger(ch_tick_target);
                    if(err == NRF_SUCCESS){
                        first_ch_target_set = true;
                    }*/  
                     
                 }
              //clear_events_restart_rx();
      
             }

            break;
      }
      //isr_debug.radio_isr++;
    }
  }


uint32_t nrf_esb_init(nrf_esb_config_t const * p_config)
{
    uint32_t err_code;

    VERIFY_PARAM_NOT_NULL(p_config);

    if (m_esb_initialized)
    {
        err_code = nrf_esb_disable();
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    m_event_handler = p_config->event_handler;

    memcpy(&m_config_local, p_config, sizeof(nrf_esb_config_t));

    m_interrupt_flags    = 0;

    memset(m_rx_pipe_info, 0, sizeof(m_rx_pipe_info));
    memset(m_pids, 0, sizeof(m_pids));

    VERIFY_TRUE(update_radio_parameters(), NRF_ERROR_INVALID_PARAM);

    // Configure radio address registers according to ESB default values
    NRF_RADIO->BASE0   = 0xE7E7E7E7;
    NRF_RADIO->BASE1   = 0x43434343;
    NRF_RADIO->PREFIX0 = 0x23C343E7;
    NRF_RADIO->PREFIX1 = 0x13E363A3;

    initialize_fifos();

    sys_timer_init();

    ppi_init();
    m_config_local.radio_irq_priority = 1;
    m_config_local.event_irq_priority = 3;
    NVIC_SetPriority(RADIO_IRQn, m_config_local.radio_irq_priority & ESB_IRQ_PRIORITY_MSK);
    NVIC_SetPriority(ESB_EVT_IRQ, m_config_local.event_irq_priority & ESB_IRQ_PRIORITY_MSK);
    NVIC_EnableIRQ(ESB_EVT_IRQ);

    m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;
    m_esb_initialized = true;

    nrf_gpio_cfg_output(TEST_PIN);//set pin as output pin 




#ifdef NRF52832_XXAA
if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004500) //Check if the device is an nRF52832 Rev. 2.
    //Workaround for nRF52832 rev 2 errata 182
    *(volatile uint32_t *) 0x4000173C |= (1 << 10);
#endif

    return NRF_SUCCESS;
}


uint32_t nrf_esb_suspend(void)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);

    // Clear PPI
    NRF_PPI->CHENCLR = (1 << NRF_ESB_PPI_TIMER_START) |
                       (1 << NRF_ESB_PPI_TIMER_STOP)  |
                       (1 << NRF_ESB_PPI_RX_TIMEOUT)  |
                       (1 << NRF_ESB_PPI_TX_START);

    m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;

    return NRF_SUCCESS;
}


uint32_t nrf_esb_disable(void)
{
    // Clear PPI
    NRF_PPI->CHENCLR = (1 << NRF_ESB_PPI_TIMER_START) |
                       (1 << NRF_ESB_PPI_TIMER_STOP)  |
                       (1 << NRF_ESB_PPI_RX_TIMEOUT)  |
                       (1 << NRF_ESB_PPI_TX_START);

    m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;
    m_esb_initialized = false;

    reset_fifos();

    memset(m_rx_pipe_info, 0, sizeof(m_rx_pipe_info));
    memset(m_pids, 0, sizeof(m_pids));

    // Disable the radio
    NVIC_DisableIRQ(ESB_EVT_IRQ);
    NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos |
                        RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos;

    return NRF_SUCCESS;
}


bool nrf_esb_is_idle(void)
{
    return m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE;
}


void ESB_EVT_IRQHandler(void)
{
    
    //isr_debug.esb_isr++;
    ret_code_t      err_code;
    uint32_t        interrupts;
    nrf_esb_evt_t   event;

    event.tx_attempts = m_last_tx_attempts;

    err_code = nrf_esb_get_clear_interrupts(&interrupts);
    if (err_code == NRF_SUCCESS && m_event_handler != 0)
    {
        if (interrupts & NRF_ESB_INT_TX_SUCCESS_MSK)
        {
            event.evt_id = NRF_ESB_EVENT_TX_SUCCESS;
            m_event_handler(&event);
        }
        if (interrupts & NRF_ESB_INT_TX_FAILED_MSK)
        {
            event.evt_id = NRF_ESB_EVENT_TX_FAILED;
            m_event_handler(&event);
        }
        if (interrupts & NRF_ESB_INT_RX_DATA_RECEIVED_MSK)
        {
            event.evt_id = NRF_ESB_EVENT_RX_RECEIVED;
            m_event_handler(&event);
        }
    }
    //isr_debug.esb_isr++;
}


static nrf_esb_payload_random_access_buf_wrapper_t *find_free_payload_cont(void)
{
    for (int i = 0; i < NRF_ESB_TX_FIFO_SIZE; i++)
    {
        if(!m_ack_pl_container[i].in_use) return &m_ack_pl_container[i];
    }
    return 0;
}


uint32_t nrf_esb_write_payload(nrf_esb_payload_t const * p_payload)
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);
    VERIFY_PARAM_NOT_NULL(p_payload);
    VERIFY_PAYLOAD_LENGTH(p_payload);
    VERIFY_FALSE(m_tx_fifo.count >= NRF_ESB_TX_FIFO_SIZE, NRF_ERROR_NO_MEM);
    VERIFY_TRUE(p_payload->pipe < NRF_ESB_PIPE_COUNT, NRF_ERROR_INVALID_PARAM);

    DISABLE_RF_IRQ();

    if (m_config_local.mode == NRF_ESB_MODE_PTX)
    {
        memcpy(m_tx_fifo.p_payload[m_tx_fifo.entry_point], p_payload, sizeof(nrf_esb_payload_t));

        m_pids[p_payload->pipe] = (m_pids[p_payload->pipe] + 1) % (NRF_ESB_PID_MAX + 1);
        m_tx_fifo.p_payload[m_tx_fifo.entry_point]->pid = m_pids[p_payload->pipe];

        if (++m_tx_fifo.entry_point >= NRF_ESB_TX_FIFO_SIZE)
        {
            m_tx_fifo.entry_point = 0;
        }

        m_tx_fifo.count++;
    }
    else
    {
        nrf_esb_payload_random_access_buf_wrapper_t *new_ack_payload;
        if((new_ack_payload = find_free_payload_cont()) != 0)
        {
            new_ack_payload->in_use = true;
            new_ack_payload->p_next = 0;
            memcpy(new_ack_payload->p_payload, p_payload, sizeof(nrf_esb_payload_t));

            m_pids[p_payload->pipe] = (m_pids[p_payload->pipe] + 1) % (NRF_ESB_PID_MAX + 1);
            new_ack_payload->p_payload->pid = m_pids[p_payload->pipe];

            if(m_ack_pl_container_entry_point_pr_pipe[p_payload->pipe] == 0)
            {
                m_ack_pl_container_entry_point_pr_pipe[p_payload->pipe] = new_ack_payload;
            }
            else
            {
                nrf_esb_payload_random_access_buf_wrapper_t *list_iterator = m_ack_pl_container_entry_point_pr_pipe[p_payload->pipe];
                while(list_iterator->p_next != 0)
                {
                    list_iterator = (nrf_esb_payload_random_access_buf_wrapper_t *)list_iterator->p_next;
                }
                list_iterator->p_next = (struct nrf_esb_payload_random_access_buf_wrapper_t *)new_ack_payload;
            }
            m_tx_fifo.count++;
        }
    }

    ENABLE_RF_IRQ();


    if (m_config_local.mode == NRF_ESB_MODE_PTX &&
        m_config_local.tx_mode == NRF_ESB_TXMODE_AUTO &&
        m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE)
    {
        start_tx_transaction();
    }

    return NRF_SUCCESS;
}


uint32_t nrf_esb_read_rx_payload(nrf_esb_payload_t * p_payload)
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);
    VERIFY_PARAM_NOT_NULL(p_payload);

    if (m_rx_fifo.count == 0)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    DISABLE_RF_IRQ();

    p_payload->length = m_rx_fifo.p_payload[m_rx_fifo.exit_point]->length;
    p_payload->pipe   = m_rx_fifo.p_payload[m_rx_fifo.exit_point]->pipe;
    p_payload->rssi   = m_rx_fifo.p_payload[m_rx_fifo.exit_point]->rssi;
    p_payload->pid    = m_rx_fifo.p_payload[m_rx_fifo.exit_point]->pid;
    p_payload->noack  = m_rx_fifo.p_payload[m_rx_fifo.exit_point]->noack;
    memcpy(p_payload->data, m_rx_fifo.p_payload[m_rx_fifo.exit_point]->data, p_payload->length);

    if (++m_rx_fifo.exit_point >= NRF_ESB_RX_FIFO_SIZE)
    {
        m_rx_fifo.exit_point = 0;
    }

    m_rx_fifo.count--;

    ENABLE_RF_IRQ();

    return NRF_SUCCESS;
}


uint32_t nrf_esb_start_tx(void)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);

    if (m_tx_fifo.count == 0)
    {
        return NRF_ERROR_BUFFER_EMPTY;
    }

    start_tx_transaction();

    return NRF_SUCCESS;
}


uint32_t nrf_esb_start_rx(void)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);

    NRF_RADIO->INTENCLR = 0xFFFFFFFF;
    NRF_RADIO->EVENTS_DISABLED = 0;
    on_radio_disabled = on_radio_disabled_rx;

    NRF_RADIO->SHORTS      = m_radio_shorts_common | RADIO_SHORTS_DISABLED_TXEN_Msk;
    NRF_RADIO->INTENSET    = RADIO_INTENSET_DISABLED_Msk;
    m_nrf_esb_mainstate    = NRF_ESB_STATE_PRX;

    NRF_RADIO->RXADDRESSES  = m_esb_addr.rx_pipes_enabled;
    NRF_RADIO->FREQUENCY    = m_esb_addr.rf_channel;
    NRF_RADIO->PACKETPTR    = (uint32_t)m_rx_payload_buffer;

    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);

    NRF_RADIO->EVENTS_ADDRESS = 0;
    NRF_RADIO->EVENTS_PAYLOAD = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;

    NRF_RADIO->TASKS_RXEN  = 1;

    ppi_radio_rx_configure();

    return NRF_SUCCESS;
}


uint32_t nrf_esb_stop_rx(void)
{
    if (m_nrf_esb_mainstate == NRF_ESB_STATE_PRX ||
        m_nrf_esb_mainstate == NRF_ESB_STATE_PRX_SEND_ACK)
    {
        NRF_RADIO->SHORTS = 0;
        NRF_RADIO->INTENCLR = 0xFFFFFFFF;
        on_radio_disabled = NULL;
        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_DISABLE = 1;
        while (NRF_RADIO->EVENTS_DISABLED == 0);
        m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;

        return NRF_SUCCESS;
    }

    return NRF_ESB_ERROR_NOT_IN_RX_MODE;
}


uint32_t nrf_esb_flush_tx(void)
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);

    DISABLE_RF_IRQ();

    m_tx_fifo.count = 0;
    m_tx_fifo.entry_point = 0;
    m_tx_fifo.exit_point = 0;

    ENABLE_RF_IRQ();

    return NRF_SUCCESS;
}


uint32_t nrf_esb_pop_tx(void)
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);
    VERIFY_TRUE(m_tx_fifo.count > 0, NRF_ERROR_BUFFER_EMPTY);

    DISABLE_RF_IRQ();

    if (m_tx_fifo.entry_point == 0)
    {
        m_tx_fifo.entry_point = (NRF_ESB_TX_FIFO_SIZE-1);
    }
    else
    {
        m_tx_fifo.entry_point--;
    }
    m_tx_fifo.count--;

    ENABLE_RF_IRQ();

    return NRF_SUCCESS;
}


uint32_t nrf_esb_flush_rx(void)
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);

    DISABLE_RF_IRQ();

    m_rx_fifo.count = 0;
    m_rx_fifo.entry_point = 0;
    m_rx_fifo.exit_point = 0;

    memset(m_rx_pipe_info, 0, sizeof(m_rx_pipe_info));

    ENABLE_RF_IRQ();

    return NRF_SUCCESS;
}


uint32_t nrf_esb_set_address_length(uint8_t length)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_TRUE(length > 2 && length < 6, NRF_ERROR_INVALID_PARAM);

#ifdef NRF52832_XXAA
    uint32_t base_address_mask = length == 5 ? 0xFFFF0000 : 0xFF000000;
    if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004200)  //Check if the device is an nRF52832 Rev. 1.
    {
        /*
        Workaround for nRF52832 Rev 1 Errata 107
        Check if pipe 0 or pipe 1-7 has a 'zero address'.
        Avoid using access addresses in the following pattern (where X is don't care):
        ADDRLEN=5
        BASE0 = 0x0000XXXX, PREFIX0 = 0xXXXXXX00
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXXXX00XX
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXX00XXXX
        BASE1 = 0x0000XXXX, PREFIX0 = 0x00XXXXXX
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXXXX00
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXX00XX
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXX00XXXX
        BASE1 = 0x0000XXXX, PREFIX1 = 0x00XXXXXX

        ADDRLEN=4
        BASE0 = 0x00XXXXXX, PREFIX0 = 0xXXXXXX00
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXXXX00XX
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXX00XXXX
        BASE1 = 0x00XXXXXX, PREFIX0 = 0x00XXXXXX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXXXX00
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXX00XX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXX00XXXX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0x00XXXXXX
        */
        if ((NRF_RADIO->BASE0 & base_address_mask) == 0 && (NRF_RADIO->PREFIX0 & 0x000000FF) == 0)
        {
            return NRF_ERROR_INVALID_PARAM;
        }
        if ((NRF_RADIO->BASE1 & base_address_mask) == 0 && ((NRF_RADIO->PREFIX0 & 0x0000FF00) == 0 ||(NRF_RADIO->PREFIX0 & 0x00FF0000) == 0 || (NRF_RADIO->PREFIX0 & 0xFF000000) == 0 ||
           (NRF_RADIO->PREFIX1 & 0xFF000000) == 0 || (NRF_RADIO->PREFIX1 & 0x00FF0000) == 0 ||(NRF_RADIO->PREFIX1 & 0x0000FF00) == 0 || (NRF_RADIO->PREFIX1 & 0x000000FF) == 0))
        {
            return NRF_ERROR_INVALID_PARAM;
        }
    }
#endif

    m_esb_addr.addr_length = length;

    update_rf_payload_format(m_config_local.payload_length);

#ifdef NRF52832_XXAA
    if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004500)  //Check if the device is an nRF52832 Rev. 2.
    {
        return apply_address_workarounds();
    }
    else
    {
        return NRF_SUCCESS;
    }
#else
    return NRF_SUCCESS;
#endif
}


uint32_t nrf_esb_set_base_address_0(uint8_t const * p_addr)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_PARAM_NOT_NULL(p_addr);

#ifdef NRF52832_XXAA
    if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004200)  //Check if the device is an nRF52832 Rev. 1.
    {
        /*
        Workaround for nRF52832 Rev 1 Errata 107
        Check if pipe 0 or pipe 1-7 has a 'zero address'.
        Avoid using access addresses in the following pattern (where X is don't care):
        ADDRLEN=5
        BASE0 = 0x0000XXXX, PREFIX0 = 0xXXXXXX00
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXXXX00XX
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXX00XXXX
        BASE1 = 0x0000XXXX, PREFIX0 = 0x00XXXXXX
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXXXX00
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXX00XX
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXX00XXXX
        BASE1 = 0x0000XXXX, PREFIX1 = 0x00XXXXXX

        ADDRLEN=4
        BASE0 = 0x00XXXXXX, PREFIX0 = 0xXXXXXX00
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXXXX00XX
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXX00XXXX
        BASE1 = 0x00XXXXXX, PREFIX0 = 0x00XXXXXX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXXXX00
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXX00XX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXX00XXXX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0x00XXXXXX
        */
        uint32_t base_address_mask = m_esb_addr.addr_length == 5 ? 0xFFFF0000 : 0xFF000000;
        if ((addr_conv(p_addr) & base_address_mask) == 0 && (NRF_RADIO->PREFIX0 & 0x000000FF) == 0)
        {
            return NRF_ERROR_INVALID_PARAM;
        }
    }
#endif



    memcpy(m_esb_addr.base_addr_p0, p_addr, 4);

    update_radio_addresses(NRF_ESB_ADDR_UPDATE_MASK_BASE0);
#ifdef NRF52832_XXAA
    return apply_address_workarounds();
#else
    return NRF_SUCCESS;
#endif
}


uint32_t nrf_esb_set_base_address_1(uint8_t const * p_addr)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_PARAM_NOT_NULL(p_addr);

#ifdef NRF52832_XXAA
    if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004200)  //Check if the device is an nRF52832 Rev. 1.
    {
        /*
        Workaround for nRF52832 Rev 1 Errata 107
        Check if pipe 0 or pipe 1-7 has a 'zero address'.
        Avoid using access addresses in the following pattern (where X is don't care):
        ADDRLEN=5
        BASE0 = 0x0000XXXX, PREFIX0 = 0xXXXXXX00
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXXXX00XX
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXX00XXXX
        BASE1 = 0x0000XXXX, PREFIX0 = 0x00XXXXXX
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXXXX00
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXX00XX
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXX00XXXX
        BASE1 = 0x0000XXXX, PREFIX1 = 0x00XXXXXX

        ADDRLEN=4
        BASE0 = 0x00XXXXXX, PREFIX0 = 0xXXXXXX00
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXXXX00XX
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXX00XXXX
        BASE1 = 0x00XXXXXX, PREFIX0 = 0x00XXXXXX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXXXX00
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXX00XX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXX00XXXX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0x00XXXXXX
        */
        uint32_t base_address_mask = m_esb_addr.addr_length == 5 ? 0xFFFF0000 : 0xFF000000;
        if ((addr_conv(p_addr) & base_address_mask) == 0 &&
            ((NRF_RADIO->PREFIX0 & 0x0000FF00) == 0 ||(NRF_RADIO->PREFIX0 & 0x00FF0000) == 0 ||
            (NRF_RADIO->PREFIX0 & 0xFF000000) == 0 || (NRF_RADIO->PREFIX1 & 0xFF000000) == 0 ||
            (NRF_RADIO->PREFIX1 & 0x00FF0000) == 0 ||(NRF_RADIO->PREFIX1 & 0x0000FF00) == 0 ||
            (NRF_RADIO->PREFIX1 & 0x000000FF) == 0))
        {
            return NRF_ERROR_INVALID_PARAM;
        }
    }
#endif

    memcpy(m_esb_addr.base_addr_p1, p_addr, 4);

    update_radio_addresses(NRF_ESB_ADDR_UPDATE_MASK_BASE1);

#ifdef NRF52832_XXAA
    return apply_address_workarounds();
#else
    return NRF_SUCCESS;
#endif
}


uint32_t nrf_esb_set_prefixes(uint8_t const * p_prefixes, uint8_t num_pipes)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_PARAM_NOT_NULL(p_prefixes);
    VERIFY_TRUE(num_pipes <= NRF_ESB_PIPE_COUNT, NRF_ERROR_INVALID_PARAM);

#ifdef NRF52832_XXAA
    if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004200)  //Check if the device is an nRF52832 Rev. 1.
    {
        /*
        Workaround for nRF52832 Rev 1 Errata 107
        Check if pipe 0 or pipe 1-7 has a 'zero address'.
        Avoid using access addresses in the following pattern (where X is don't care):
        ADDRLEN=5
        BASE0 = 0x0000XXXX, PREFIX0 = 0xXXXXXX00
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXXXX00XX
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXX00XXXX
        BASE1 = 0x0000XXXX, PREFIX0 = 0x00XXXXXX
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXXXX00
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXX00XX
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXX00XXXX
        BASE1 = 0x0000XXXX, PREFIX1 = 0x00XXXXXX

        ADDRLEN=4
        BASE0 = 0x00XXXXXX, PREFIX0 = 0xXXXXXX00
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXXXX00XX
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXX00XXXX
        BASE1 = 0x00XXXXXX, PREFIX0 = 0x00XXXXXX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXXXX00
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXX00XX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXX00XXXX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0x00XXXXXX
        */
        uint32_t base_address_mask = m_esb_addr.addr_length == 5 ? 0xFFFF0000 : 0xFF000000;
        if (num_pipes >= 1 && (NRF_RADIO->BASE0 & base_address_mask) == 0 && p_prefixes[0] == 0)
        {
            return NRF_ERROR_INVALID_PARAM;
        }

        if ((NRF_RADIO->BASE1 & base_address_mask) == 0)
        {
            for (uint8_t i = 1; i < num_pipes; i++)
            {
                if (p_prefixes[i] == 0)
                {
                    return NRF_ERROR_INVALID_PARAM;
                }
            }
        }
    }
#endif

    memcpy(m_esb_addr.pipe_prefixes, p_prefixes, num_pipes);
    m_esb_addr.num_pipes = num_pipes;
    m_esb_addr.rx_pipes_enabled = BIT_MASK_UINT_8(num_pipes);

    update_radio_addresses(NRF_ESB_ADDR_UPDATE_MASK_PREFIX);

#ifdef NRF52832_XXAA
    return apply_address_workarounds();
#else
    return NRF_SUCCESS;
#endif
}


uint32_t nrf_esb_update_prefix(uint8_t pipe, uint8_t prefix)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_TRUE(pipe < NRF_ESB_PIPE_COUNT, NRF_ERROR_INVALID_PARAM);

#ifdef NRF52832_XXAA
    if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004200)  //Check if the device is an nRF52832 Rev. 1.
    {
        /*
        Workaround for nRF52832 Rev 1 Errata 107
        Check if pipe 0 or pipe 1-7 has a 'zero address'.
        Avoid using access addresses in the following pattern (where X is don't care):
        ADDRLEN=5
        BASE0 = 0x0000XXXX, PREFIX0 = 0xXXXXXX00
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXXXX00XX
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXX00XXXX
        BASE1 = 0x0000XXXX, PREFIX0 = 0x00XXXXXX
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXXXX00
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXX00XX
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXX00XXXX
        BASE1 = 0x0000XXXX, PREFIX1 = 0x00XXXXXX

        ADDRLEN=4
        BASE0 = 0x00XXXXXX, PREFIX0 = 0xXXXXXX00
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXXXX00XX
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXX00XXXX
        BASE1 = 0x00XXXXXX, PREFIX0 = 0x00XXXXXX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXXXX00
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXX00XX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXX00XXXX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0x00XXXXXX
        */
        uint32_t base_address_mask = m_esb_addr.addr_length == 5 ? 0xFFFF0000 : 0xFF000000;
        if (pipe == 0)
        {
            if ((NRF_RADIO->BASE0 & base_address_mask) == 0 && prefix == 0)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
        }
        else
        {
            if ((NRF_RADIO->BASE1 & base_address_mask) == 0 && prefix == 0)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
        }
    }
#endif
    m_esb_addr.pipe_prefixes[pipe] = prefix;

    update_radio_addresses(NRF_ESB_ADDR_UPDATE_MASK_PREFIX);

#ifdef NRF52832_XXAA
    return apply_address_workarounds();
#else
    return NRF_SUCCESS;
#endif
}


uint32_t nrf_esb_enable_pipes(uint8_t enable_mask)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_TRUE((enable_mask | BIT_MASK_UINT_8(NRF_ESB_PIPE_COUNT)) == BIT_MASK_UINT_8(NRF_ESB_PIPE_COUNT), NRF_ERROR_INVALID_PARAM);

    m_esb_addr.rx_pipes_enabled = enable_mask;

#ifdef NRF52832_XXAA
    return apply_address_workarounds();
#else
    return NRF_SUCCESS;
#endif
}


uint32_t nrf_esb_set_rf_channel(uint32_t channel)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_TRUE(channel <= 100, NRF_ERROR_INVALID_PARAM);

    m_esb_addr.rf_channel = channel;

    return NRF_SUCCESS;
}


uint32_t nrf_esb_get_rf_channel(uint32_t * p_channel)
{
    VERIFY_PARAM_NOT_NULL(p_channel);

    *p_channel = m_esb_addr.rf_channel;

    return NRF_SUCCESS;
}


uint32_t nrf_esb_set_tx_power(nrf_esb_tx_power_t tx_output_power)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);

    if ( m_config_local.tx_output_power != tx_output_power )
    {
        m_config_local.tx_output_power = tx_output_power;
        update_radio_tx_power();
    }

    return NRF_SUCCESS;
}


uint32_t nrf_esb_set_retransmit_delay(uint16_t delay)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_TRUE(delay >= m_wait_for_ack_timeout_us + RETRANSMIT_DELAY_US_OFFSET, NRF_ERROR_INVALID_PARAM);

    m_config_local.retransmit_delay = delay;
    return NRF_SUCCESS;
}


uint32_t nrf_esb_set_retransmit_count(uint16_t count)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);

    m_config_local.retransmit_count = count;
    return NRF_SUCCESS;
}


uint32_t nrf_esb_set_bitrate(nrf_esb_bitrate_t bitrate)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);

    m_config_local.bitrate = bitrate;
    return update_radio_bitrate() ? NRF_SUCCESS : NRF_ERROR_INVALID_PARAM;
}


uint32_t nrf_esb_reuse_pid(uint8_t pipe)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_TRUE(pipe < NRF_ESB_PIPE_COUNT, NRF_ERROR_INVALID_PARAM);

    m_pids[pipe] = (m_pids[pipe] + NRF_ESB_PID_MAX) % (NRF_ESB_PID_MAX + 1);
    return NRF_SUCCESS;
}



void SWI3_EGU3_IRQHandler(void)
{

    if (NRF_EGU3->EVENTS_TRIGGERED[0] != 0)
    { 
        NRF_EGU3->EVENTS_TRIGGERED[0] = 0;
        //NRF_LOG_INFO("3:%u",((sync_pkt_t *) mp_curr_adj_pkt)->counter_val)
        //nrf_atomic_u32_fetch_store(&m_master_counter, ((sync_pkt_t *) mp_curr_adj_pkt)->counter_val);
        m_master_counter = cur_counter_val; //cal new capture
        NRF_LOG_INFO("%d",cur_counter_val);
        cur_counter_val = 0;
        //NRF_LOG_INFO("4:%u\n",m_master_counter);
        //m_params.high_freq_timer[1]->CC[4] = m_trigger_evt.params.triggered.tick_target - m_master_counter;	// calculate new capture
       
        m_params.high_freq_timer[1]->CC[5] = ch_current_target - m_master_counter;	// calculate new capture
        
        
        //m_trigger_evt.params.triggered.used_packet_count++;
        //m_trigger_evt.params.triggered.last_sync = m_master_counter;

        nrf_atomic_flag_clear(&m_timer_update_in_progress);

        if (!m_synchronized)
        {
           
            m_synchronized = true;
            /*if (m_callback)
            {
                ts_evt_t evt =
                {
                    .type = TS_EVT_SYNCHRONIZED,
                };
        

                m_callback(&evt);
            }*/
        }
    }

    /*
    if (NRF_EGU3->EVENTS_TRIGGERED[2] != 0)
    {
        NRF_EGU3->EVENTS_TRIGGERED[2] = 0;
        nrf_ppi_channel_disable(m_params.ppi_chns[4]);

        if (m_callback)
        {  
            m_callback(&m_trigger_evt);
        }
    }
    */
     if (NRF_EGU3->EVENTS_TRIGGERED[3] != 0)
    {
        static uint32_t test_count;
        uint32_t err;
        NRF_EGU3->EVENTS_TRIGGERED[3] = 0;
        /*channel hopping process start*/
        nrf_ppi_channel_disable(m_params.ppi_chns[5]);
        nrf_esb_stop_rx();
        if(chn_index >= sizeof(chn_map)/sizeof(uint32_t)){
          chn_index = 0; 
        }
        err = nrf_esb_set_rf_channel(chn_map[chn_index]);
        //NRF_LOG_INFO("%d,%d",chn_index,test_count);
        if(err != NRF_SUCCESS){
            NRF_LOG_DEBUG("%d",err);          
        }
        nrf_esb_start_rx();
        chn_index++;//update channel index
        test_count++;
        ch_set_next_target();   
    }
    

}

static inline bool sync_timer_offset_compensate(sync_pkt_t *p_pkt)
{
    uint32_t peer_timer;
    uint32_t local_timer;
    uint32_t timer_offset;

    if (m_timer_update_in_progress)
    {
        return false;
    }

    if (CHECK_INVALID_TIMING(p_pkt->timer_val))
    {
        return false;
    }
    
    if(p_pkt -> counter_val <= 0 && m_synchronized == true){
      return false;
    }
    if(p_pkt -> counter_val > ch_current_target && m_synchronized == true){
      return false;
    }
    if(ch_current_target - (p_pkt -> counter_val) <= 2 && m_synchronized == true ){
        return false;
    }
    

    peer_timer  = p_pkt->timer_val;
    //peer_timer += TX_CHAIN_DELAY;
    peer_timer += 4000; //measured TX_CHAIN_DELAY is 4000
    local_timer = m_params.high_freq_timer[0]->CC[1];//take timer0 value

    if (local_timer > peer_timer)
    {
        timer_offset = TIME_SYNC_TIMER_MAX_VAL - local_timer + peer_timer;
    }
    else
    {
        timer_offset = peer_timer - local_timer;
    }
    //NRF_LOG_INFO("timer_offset %d ",timer_offset);

    /*
     * FIXME:
     * This is a PPI / timer timing issue:
     * If the the CC[2] for the timer reset is too close to the timer overflow,
     * the timer behavior becomes fuzzy...
     * Probably because of the PPI + timer task delays, it is possible that the
     * timestamp PPI task is executed after the timer was reset.
     */
    if (timer_offset < 5)
    {
        m_trigger_evt.params.triggered.used_packet_count++;
        m_trigger_evt.params.triggered.last_sync = p_pkt->counter_val;
        return false;
    }

    if (timer_offset == 0 || timer_offset == TIME_SYNC_TIMER_MAX_VAL)
    {
        // Already in sync
        m_trigger_evt.params.triggered.used_packet_count++;
        m_trigger_evt.params.triggered.last_sync = p_pkt->counter_val;
        return false;
    }
    else if (timer_offset > TIME_SYNC_TIMER_MAX_VAL)
    {
        m_params.high_freq_timer[0]->CC[2] = (timer_offset - TIME_SYNC_TIMER_MAX_VAL);
    }
    else
    {
        m_params.high_freq_timer[0]->CC[2] = (TIME_SYNC_TIMER_MAX_VAL - timer_offset);
    }

    /*
     * This is necessary, because the following timer reset will not be counted as step
     */
    p_pkt->counter_val++;

    // resetting the timer to the trigger target will skip the trigger -> avoid this
    if (p_pkt->counter_val == m_trigger_evt.params.triggered.tick_target)
    {
        return false;
    }

    //ASSERT(m_params.high_freq_timer[0]->CC[2] < TIME_SYNC_TIMER_MAX_VAL);

    nrf_atomic_flag_set(&m_timer_update_in_progress);
    //NRF_LOG_INFO("1:%u",p_pkt -> counter_val);
    //nrf_atomic_u32_fetch_store(&mp_curr_adj_pkt, (uint32_t)p_pkt);
    //NRF_LOG_INFO("2:%u",((sync_pkt_t *)mp_curr_adj_pkt) -> counter_val);
    cur_counter_val = p_pkt->counter_val;
    ppi_sync_timer_adjust_enable();
    return true;
}


static void ppi_sync_timer_adjust_enable(void)
{
    NRF_PPI->TASKS_CHG[m_params.ppi_chg].EN = 1;
}

static void ppi_timestamp_timer_configure(void)
{
    uint32_t chn;
    //ppi channel 3
    chn = m_params.ppi_chns[3];
  
    NRF_PPI->CH[chn].EEP   = (uint32_t) &m_params.high_freq_timer[0]->EVENTS_COMPARE[0]; //when timer0 value equals to cc[0] compare event is generated
    NRF_PPI->CH[chn].TEP   = (uint32_t) &m_params.high_freq_timer[1]->TASKS_COUNT;       //timer 1 Increment Timer (counter mode) 
    NRF_PPI->FORK[chn].TEP = 0;
    NRF_PPI->CHENSET       = (1 << chn);
}
static void ppi_counter_timer_capture_configure(uint32_t chn)
{
    NRF_PPI->CH[chn].EEP    = (uint32_t) &m_params.egu->EVENTS_TRIGGERED[1];
    NRF_PPI->CH[chn].TEP    = (uint32_t) &m_params.high_freq_timer[0]->TASKS_CAPTURE[3];
    NRF_PPI->FORK[chn].TEP  = (uint32_t) &m_params.high_freq_timer[1]->TASKS_CAPTURE[0];
    NRF_PPI->CHENSET        = (1 << chn);
}

static void ppi_counter_timer_capture_disable(uint32_t chn)
{
    NRF_PPI->CHENCLR = (1 << chn);

    NRF_PPI->CH[chn].EEP    = 0;
    NRF_PPI->CH[chn].TEP    = 0;
    NRF_PPI->FORK[chn].TEP  = 0;
}

static void ppi_sync_timer_adjust_configure(void)
{
    uint32_t chn0, chn1, chg;

    chn0 = m_params.ppi_chns[0];
    chn1 = m_params.ppi_chns[1];
    chg  = m_params.ppi_chg;

    // PPI channel 0: clear timer when compare[2] value is reached
    NRF_PPI->CHENCLR        = (1 << chn0);
    NRF_PPI->CH[chn0].EEP   = (uint32_t) &m_params.high_freq_timer[0]->EVENTS_COMPARE[2];
    NRF_PPI->CH[chn0].TEP   = (uint32_t) &m_params.high_freq_timer[0]->TASKS_CLEAR;
    NRF_PPI->FORK[chn0].TEP = (uint32_t) &m_params.high_freq_timer[1]->TASKS_CLEAR;

    // PPI channel 1: disable PPI channel 0 such that the timer is only reset once, and trigger software interrupt
    NRF_PPI->CHENCLR        = (1 << chn1);
    NRF_PPI->CH[chn1].EEP   = (uint32_t) &m_params.high_freq_timer[0]->EVENTS_COMPARE[2];
    NRF_PPI->CH[chn1].TEP   = (uint32_t) &NRF_PPI->TASKS_CHG[chg].DIS;	// disable PPI group
    NRF_PPI->FORK[chn1].TEP = (uint32_t) &m_params.egu->TASKS_TRIGGER[0]; // trigger EGU interrupt

    NRF_PPI->TASKS_CHG[chg].DIS = 1;
    NRF_PPI->CHG[chg]           = (1 << chn0) | (1 << chn1);
}
static void timestamp_counter_start(void)
{
    // m_params.high_freq_timer[1] (NRF_TIMER) is used in counter mode to count the number of sync timer overflows/resets (m_params.high_freq_timer[0])
    // When timestamp API is used, the number of overflows/resets + current value of sync timer must be added up to give accurate timestamp information
    m_params.high_freq_timer[1]->TASKS_STOP  = 1;
    m_params.high_freq_timer[1]->TASKS_CLEAR = 1;
    m_params.high_freq_timer[1]->PRESCALER   = 0;
    m_params.high_freq_timer[1]->MODE        = TIMER_MODE_MODE_Counter << TIMER_MODE_MODE_Pos;;
    m_params.high_freq_timer[1]->BITMODE     = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
    m_params.high_freq_timer[1]->TASKS_START = 1;
}
static void sync_timer_start(void)
{
    // m_params.high_freq_timer[0] (NRF_TIMER) is the always-running sync timer
    // The timing master never adjusts this timer
    // The timing slave(s) adjusts this timer whenever a sync packet is received and the logic determines that there is
    m_params.high_freq_timer[0]->TASKS_STOP  = 1;
    m_params.high_freq_timer[0]->TASKS_CLEAR = 1;
    m_params.high_freq_timer[0]->PRESCALER   = 0;
    m_params.high_freq_timer[0]->BITMODE     = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
    m_params.high_freq_timer[0]->CC[0]       = TIME_SYNC_TIMER_MAX_VAL; //timer0 reach this value then timer1 increase
    m_params.high_freq_timer[0]->CC[1]       = 0xFFFFFFFF;
    m_params.high_freq_timer[0]->CC[2]       = 0xFFFFFFFF;
    m_params.high_freq_timer[0]->CC[3]       = 0xFFFFFFFF;

    if (m_params.high_freq_timer[0] == NRF_TIMER3 || m_params.high_freq_timer[0] == NRF_TIMER4)
    {
        // TIMERS 0,1, and 2 only have 4 compare registers
        m_params.high_freq_timer[0]->CC[4]   = TIME_SYNC_TIMER_MAX_VAL / 2; // Only used for debugging purposes such as pin toggling
    }

    m_params.high_freq_timer[0]->SHORTS      = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    m_params.high_freq_timer[0]->TASKS_START = 1;
}
uint32_t ppi_sync_trigger_configure(uint32_t ppi_endpoint)
{
    uint32_t error;

    error = nrfx_ppi_channel_assign(m_params.ppi_chns[4], (uint32_t) &m_params.high_freq_timer[1]->EVENTS_COMPARE[4], ppi_endpoint);
    if (error != NRF_SUCCESS)
    {
        return error;
    }

    return nrfx_ppi_channel_fork_assign(m_params.ppi_chns[4], (uint32_t) &m_params.egu->TASKS_TRIGGER[2]);
}
uint32_t ts_set_trigger(uint32_t target_tick, uint32_t ppi_endpoint)
{


    if (ppi_sync_trigger_configure(ppi_endpoint) != NRF_SUCCESS)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    m_trigger_evt.params.triggered.sync_packet_count = 0;
    m_trigger_evt.params.triggered.used_packet_count = 0;


    // TODO: is there a way to check if the target value is plausible?
    m_trigger_evt.params.triggered.tick_start = ts_timestamp_get_ticks_u64()/TIME_SYNC_TIMER_MAX_VAL;

    m_params.high_freq_timer[1]->CC[4] = target_tick - m_master_counter;
    m_trigger_evt.params.triggered.tick_target = target_tick;

    nrf_ppi_channel_enable(m_params.ppi_chns[4]); // activate trigger
    return NRF_SUCCESS;
}
uint32_t ppi_ch_trigger_configure(void)
{
    uint32_t error;
    error = nrfx_ppi_channel_assign(m_params.ppi_chns[5], (uint32_t) &m_params.high_freq_timer[1]->EVENTS_COMPARE[5], (uint32_t)&m_params.egu->TASKS_TRIGGER[3]);
    if (error != NRF_SUCCESS)
    {
        return error;
    }
    return NRF_SUCCESS;
}
uint32_t ch_set_trigger(uint32_t ch_tick_target)
{

    if (ppi_ch_trigger_configure() != NRF_SUCCESS)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    // TODO: is there a way to check if the target value is plausible?
    m_params.high_freq_timer[1]->CC[5] = ch_tick_target - m_master_counter;
    ch_current_target = ch_tick_target;
    //NRF_LOG_INFO("%d",ch_current_target - m_master_counter);
    nrf_ppi_channel_enable(m_params.ppi_chns[5]); // activate channel hopping 
    return NRF_SUCCESS;

}
void ch_set_next_target(void){

    uint32_t ch_tick_target;
    ch_tick_target = ch_current_target + 2000;
    uint32_t err_code =  ch_set_trigger(ch_tick_target);
    if(err_code != NRF_SUCCESS){
        NRF_LOG_DEBUG("fail to set next ch target")
    }
}


uint32_t ts_init(const ts_init_t * p_init)
{
    if (p_init->high_freq_timer[0] == 0 ||
        p_init->high_freq_timer[1] == 0 ||
        p_init->egu                == 0 ||          
        p_init->evt_handler        == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memcpy(m_params.high_freq_timer, p_init->high_freq_timer, sizeof(m_params.high_freq_timer));
    m_params.egu = p_init->egu;
    m_params.egu_irq_type = p_init->egu_irq_type;
    m_callback = p_init->evt_handler;

    nrfx_err_t ret = nrfx_ppi_group_alloc(&m_params.ppi_chg);
    if (ret != NRFX_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    for (size_t i = 0; i < sizeof(m_params.ppi_chns) / sizeof(m_params.ppi_chns[0]); i++)
    {
        ret = nrfx_ppi_channel_alloc(&m_params.ppi_chns[i]);
        if (ret != NRFX_SUCCESS)
        {
            return NRF_ERROR_INTERNAL;
        }
    }

    if (m_params.egu != NRF_EGU3)
    {
        // TODO: Remove hardcoded use of SWI3_EGU3_IRQHandler()
        return NRF_ERROR_INVALID_PARAM;
    }

    // TODO: Implement use of RTC as a low-power (and lower accuracy)
    // alternative to 16 MHz TIMER
    // if (SYNC_RTC_PRESCALER != m_params.rtc->PRESCALER)
    // {
    //     // TODO: Handle this
    //     return NRF_ERROR_INVALID_STATE;
    // }

    return NRF_SUCCESS;
}
static void ppi_radio_rx_configure(void)
{
    uint32_t chn;
    chn = m_params.ppi_chns[2];
    NRF_PPI->CH[chn].EEP   = (uint32_t) &NRF_RADIO->EVENTS_ADDRESS;
    NRF_PPI->CH[chn].TEP   = (uint32_t) &m_params.high_freq_timer[0]->TASKS_CAPTURE[1];  //capture high_freq_timer[0] value to cc[1] register
    NRF_PPI->FORK[chn].TEP = 0;
    NRF_PPI->CHENSET       = (1 << chn); // enable channel
}

uint32_t ts_enable(void)
{
    uint32_t err_code;

    ppi_timestamp_timer_configure();
    ppi_sync_timer_adjust_configure();
    NVIC_ClearPendingIRQ(m_params.egu_irq_type);
    NVIC_SetPriority(m_params.egu_irq_type, TIME_SYNC_EVT_HANDLER_IRQ_PRIORITY);
    NVIC_EnableIRQ(m_params.egu_irq_type);

    m_params.egu->INTENCLR = 0xFFFFFFFF;
    m_params.egu->INTENSET = EGU_INTENSET_TRIGGERED0_Msk | EGU_INTENSET_TRIGGERED2_Msk | EGU_INTENSET_TRIGGERED3_Msk | EGU_INTENSET_TRIGGERED4_Msk;

    //m_blocked_cancelled_count  = 0;
    //m_radio_state              = RADIO_STATE_IDLE;
    //nrf_atomic_flag_clear(&m_send_sync_pkt);

    timestamp_counter_start();
    sync_timer_start();
    //nrf_atomic_flag_set(&m_timeslot_session_open);

    return NRF_SUCCESS;
}



static void timers_capture(uint32_t * p_sync_timer_val, uint32_t * p_count_timer_val, uint32_t * p_peer_counter)
{
    static nrf_atomic_flag_t m_timestamp_capture_flag = 0;

    uint32_t peer_counter;
    //bool     timeslot_active;

    if (nrf_atomic_flag_set_fetch(&m_timestamp_capture_flag) != 0)
    {
        // Not thread-safe
        APP_ERROR_CHECK_BOOL(false);
    }

    nrf_ppi_channel_t ppi_chn;
    nrfx_err_t ret = nrfx_ppi_channel_alloc(&ppi_chn);
    //ASSERT(ret == NRFX_SUCCESS);

    ppi_counter_timer_capture_configure(ppi_chn);

    // Use loop in case radio state changes from inactive to active during value copy
    // Value copy needs to be atomic to get accurate timestamp
   
   do
    {
        // Do critical section during timeslot
        NVIC_DisableIRQ(SWI3_EGU3_IRQn);
        m_params.egu->EVENTS_TRIGGERED[1] = 0;
        m_params.egu->TASKS_TRIGGER[1] = 1;
        while (m_params.egu->EVENTS_TRIGGERED[1] == 0)
        {
            __NOP();
        }
    
        peer_counter = m_master_counter;
        NVIC_EnableIRQ(SWI3_EGU3_IRQn);

    } while (m_params.high_freq_timer[0]->CC[3] < 2);
    

    ppi_counter_timer_capture_disable(ppi_chn);
    nrfx_ppi_channel_free(ppi_chn);

    *p_sync_timer_val  = m_params.high_freq_timer[0]->CC[3];
    *p_count_timer_val = (m_params.high_freq_timer[1]->CC[0]);
    *p_peer_counter    = peer_counter;

    nrf_atomic_flag_clear(&m_timestamp_capture_flag);
}


uint64_t ts_timestamp_get_ticks_u64(void)
{
    uint32_t sync_timer_val;
    uint32_t count_timer_val;
    uint64_t timestamp;
    uint32_t  peer_count;

    timers_capture(&sync_timer_val, &count_timer_val, &peer_count);

    timestamp  = (uint64_t) count_timer_val;
    timestamp += (uint64_t) peer_count;
    timestamp *= TIME_SYNC_TIMER_MAX_VAL;
    timestamp += (uint64_t) sync_timer_val;

    return timestamp;
}

#ifdef NRF52832_XXAA
// Workaround neccessary on nRF52832 Rev. 1.
void NRF_ESB_BUGFIX_TIMER_IRQHandler(void)
{
    if (NRF_ESB_BUGFIX_TIMER->EVENTS_COMPARE[0])
    {
        NRF_ESB_BUGFIX_TIMER->EVENTS_COMPARE[0] = 0;

        // If the timeout timer fires and we are in the PTX receive ACK state, disable the radio
        if (m_nrf_esb_mainstate == NRF_ESB_STATE_PTX_RX_ACK)
        {
            NRF_RADIO->TASKS_DISABLE = 1;
        }
    }
}
#endif