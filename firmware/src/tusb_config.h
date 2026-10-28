#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_TUD_RHPORT          0
#define CFG_TUSB_RHPORT0_MODE     (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS               OPT_OS_PICO
#endif
#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG            0
#endif

#define CFG_TUD_ENABLED           1
#define CFG_TUD_MAX_SPEED         OPT_MODE_FULL_SPEED

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))

#define CFG_TUD_ENDPOINT0_SIZE    64

#define CFG_TUD_AUDIO             1
#define CFG_TUD_CDC               1
#define CFG_TUD_CDC_RX_BUFSIZE    256
#define CFG_TUD_CDC_TX_BUFSIZE    1024
#define CFG_TUD_MSC               0
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            0

/* ---- Audio: 6 channels, 24-bit in 3-byte subslots, 48 kHz, UAC2 microphone ---- */
#define AUDIO_SAMPLE_RATE                             48000
#define AUDIO_N_CHANNELS                              6
#define AUDIO_BYTES_PER_SAMPLE                        3
#define AUDIO_BITS_PER_SAMPLE                         24

#define CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE              AUDIO_SAMPLE_RATE
#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN                 AUDIO_6CH_MIC_DESC_LEN
#define CFG_TUD_AUDIO_FUNC_1_N_AS_INT                 1
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ              64

#define CFG_TUD_AUDIO_ENABLE_EP_IN                    1
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX    AUDIO_BYTES_PER_SAMPLE
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX            AUDIO_N_CHANNELS
#define CFG_TUD_AUDIO_EP_SZ_IN                        TUD_AUDIO_EP_SIZE(AUDIO_SAMPLE_RATE, AUDIO_BYTES_PER_SAMPLE, AUDIO_N_CHANNELS)   /* 49*18 = 882 */
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX             CFG_TUD_AUDIO_EP_SZ_IN
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ          (16 * CFG_TUD_AUDIO_EP_SZ_IN)   /* 16 ms FIFO (flow control needs >= 4 packets) */
#define CFG_TUD_AUDIO_ENABLE_ENCODING                 0
#define CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL              1   /* v2.5: keeps the 16 ms FIFO half full (47/48/49-frame packets) = ~8 ms cushion against main-loop stalls. sample rate is picked up when Windows GET_CURs the clock; without it TinyUSB falls back to min(fifo, ep size) */

/* descriptor length of our hand-written 6-channel microphone function (no feature unit) */
#define AUDIO_6CH_MIC_DESC_LEN (TUD_AUDIO_DESC_IAD_LEN\
  + TUD_AUDIO_DESC_STD_AC_LEN\
  + TUD_AUDIO_DESC_CS_AC_LEN\
  + TUD_AUDIO_DESC_CLK_SRC_LEN\
  + TUD_AUDIO_DESC_INPUT_TERM_LEN\
  + (6 + (6 + 1) * 4) /* 6-channel feature unit */\
  + TUD_AUDIO_DESC_OUTPUT_TERM_LEN\
  + TUD_AUDIO_DESC_STD_AS_INT_LEN\
  + TUD_AUDIO_DESC_STD_AS_INT_LEN\
  + TUD_AUDIO_DESC_CS_AS_INT_LEN\
  + TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN\
  + TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN\
  + TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN)

#ifdef __cplusplus
}
#endif

#endif
