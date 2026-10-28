/* USB descriptors: UAC2 6-channel 24-bit/48 kHz microphone ("HU Audio USB Analyzer") */
#include "tusb.h"
#include "pico/unique_id.h"

#define USB_VID   0xCafe   /* TinyUSB test VID - for personal use only */
#define USB_PID   0x4A06
#define USB_BCD   0x0200

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *) &desc_device;
}

enum {
    ITF_NUM_AUDIO_CONTROL = 0,
    ITF_NUM_AUDIO_STREAMING,
    ITF_NUM_CDC,            /* debug / status serial port (virtual COM) */
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + AUDIO_6CH_MIC_DESC_LEN + TUD_CDC_DESC_LEN)
#define EPNUM_AUDIO_IN   0x01
#define EPNUM_CDC_NOTIF  0x82
#define EPNUM_CDC_OUT    0x03
#define EPNUM_CDC_IN     0x83

/* entity IDs */
#define UAC2_ENTITY_CLOCK          0x04
#define UAC2_ENTITY_INPUT_TERMINAL 0x01
#define UAC2_ENTITY_OUTPUT_TERMINAL 0x03

uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    /* Interface Association */
    TUD_AUDIO_DESC_IAD(ITF_NUM_AUDIO_CONTROL, 0x02, 0x00),
    /* Standard AC interface */
    TUD_AUDIO_DESC_STD_AC(ITF_NUM_AUDIO_CONTROL, 0x00, 0x02),
    /* Class-specific AC header */
    TUD_AUDIO_DESC_CS_AC(0x0200, AUDIO_FUNC_MICROPHONE,
                         TUD_AUDIO_DESC_CLK_SRC_LEN + TUD_AUDIO_DESC_INPUT_TERM_LEN + TUD_AUDIO_DESC_OUTPUT_TERM_LEN,
                         AUDIO_CS_AS_INTERFACE_CTRL_LATENCY_POS),
    /* Clock source: internal fixed, frequency read-only */
    TUD_AUDIO_DESC_CLK_SRC(UAC2_ENTITY_CLOCK, AUDIO_CLOCK_SOURCE_ATT_INT_FIX_CLK,
                           (AUDIO_CTRL_R << AUDIO_CLOCK_SOURCE_CTRL_CLK_FRQ_POS), UAC2_ENTITY_INPUT_TERMINAL, 0x00),
    /* Input terminal: 6 logical channels */
    TUD_AUDIO_DESC_INPUT_TERM(UAC2_ENTITY_INPUT_TERMINAL, AUDIO_TERM_TYPE_IN_GENERIC_MIC, UAC2_ENTITY_OUTPUT_TERMINAL,
                              UAC2_ENTITY_CLOCK, AUDIO_N_CHANNELS, AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, 0x00,
                              AUDIO_CTRL_R << AUDIO_IN_TERM_CTRL_CONNECTOR_POS, 0x00),
    /* Output terminal: USB streaming, sourced directly from the input terminal */
    TUD_AUDIO_DESC_OUTPUT_TERM(UAC2_ENTITY_OUTPUT_TERMINAL, AUDIO_TERM_TYPE_USB_STREAMING, UAC2_ENTITY_INPUT_TERMINAL,
                               UAC2_ENTITY_INPUT_TERMINAL, UAC2_ENTITY_CLOCK, 0x0000, 0x00),
    /* AS interface alt 0 (zero bandwidth) */
    TUD_AUDIO_DESC_STD_AS_INT(ITF_NUM_AUDIO_STREAMING, 0x00, 0x00, 0x00),
    /* AS interface alt 1 (streaming) */
    TUD_AUDIO_DESC_STD_AS_INT(ITF_NUM_AUDIO_STREAMING, 0x01, 0x01, 0x00),
    TUD_AUDIO_DESC_CS_AS_INT(UAC2_ENTITY_OUTPUT_TERMINAL, AUDIO_CTRL_NONE, AUDIO_FORMAT_TYPE_I, AUDIO_DATA_FORMAT_TYPE_I_PCM,
                             AUDIO_N_CHANNELS, AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, 0x00),
    TUD_AUDIO_DESC_TYPE_I_FORMAT(AUDIO_BYTES_PER_SAMPLE, AUDIO_BITS_PER_SAMPLE),
    TUD_AUDIO_DESC_STD_AS_ISO_EP(0x80 | EPNUM_AUDIO_IN,
                                 (uint8_t) ((uint8_t) TUSB_XFER_ISOCHRONOUS | (uint8_t) TUSB_ISO_EP_ATT_ASYNCHRONOUS | (uint8_t) TUSB_ISO_EP_ATT_DATA),
                                 CFG_TUD_AUDIO_EP_SZ_IN, 0x01),
    TUD_AUDIO_DESC_CS_AS_ISO_EP(AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, AUDIO_CTRL_NONE,
                                AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, 0x0000),

    /* CDC ACM: "HU Analyzer debug" serial port over the same USB-C */
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 0x04, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64)
};

TU_VERIFY_STATIC(sizeof(desc_configuration) == CONFIG_TOTAL_LEN, "configuration descriptor length mismatch");

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void) index;
    return desc_configuration;
}

/* ---- strings ---- */
static char serial_str[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];

char const *string_desc_arr[] = {
    (const char[]) {0x09, 0x04},        /* 0: English (0x0409) */
    "siddu",                            /* 1: manufacturer */
    "HU Audio USB Analyzer 6ch",        /* 2: product */
    serial_str,                         /* 3: serial */
    "HU Analyzer debug port",           /* 4: CDC interface */
};

static uint16_t _desc_str[48];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) return NULL;
        if (index == 3 && serial_str[0] == 0) pico_get_unique_board_id_string(serial_str, sizeof(serial_str));
        const char *str = string_desc_arr[index];
        chr_count = (uint8_t) strlen(str);
        if (chr_count > 46) chr_count = 46;
        for (uint8_t i = 0; i < chr_count; i++) _desc_str[1 + i] = (uint16_t) str[i];
    }
    _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
