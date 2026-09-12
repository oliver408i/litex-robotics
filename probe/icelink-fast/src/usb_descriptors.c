/* USB descriptors: two CDC ACM interfaces.
 *
 *   if00 -- the probe console (IDCODE report + b/r/s/z/a/m/c commands)
 *   if02 -- transparent USART2 bridge to the target's serial console
 *
 * Interface order is load-bearing: the console stays first so the existing
 * /dev/serial/by-id/...-if00 path keeps pointing at it.
 *
 * VID/PID are the TinyUSB example pair -- fine for a bench tool that never
 * leaves this desk, and deliberately NOT the stock 0d28:0204 so this build is
 * never mistaken for the DAPLink probe it replaced.
 */
#include "tusb.h"

#define USB_VID 0xCAFE
#define USB_PID 0x4001

static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    /* Misc/IAD: required so the CDC control+data interfaces are grouped. */
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

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

enum { ITF_NUM_CDC0 = 0, ITF_NUM_CDC0_DATA,
       ITF_NUM_CDC1,     ITF_NUM_CDC1_DATA, ITF_NUM_TOTAL };

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + 2 * TUD_CDC_DESC_LEN)

/* Endpoint numbers must not collide in the F103's 512-byte PMA. Budget with
 * both CDCs: 2x64 (EP0) + 2x(8 notif + 64 out + 64 in) + 40 BTABLE = 440/512.
 * Notifications get their own EP numbers because the stm32_fsdev driver
 * allocates PMA per endpoint number, not per direction. */
#define EPNUM_CDC0_NOTIF 0x81
#define EPNUM_CDC0_OUT   0x02
#define EPNUM_CDC0_IN    0x82
#define EPNUM_CDC1_NOTIF 0x83
#define EPNUM_CDC1_OUT   0x04
#define EPNUM_CDC1_IN    0x84

static const uint8_t desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC0, 4, EPNUM_CDC0_NOTIF, 8,
                       EPNUM_CDC0_OUT, EPNUM_CDC0_IN, 64),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC1, 5, EPNUM_CDC1_NOTIF, 8,
                       EPNUM_CDC1_OUT, EPNUM_CDC1_IN, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_configuration;
}

static const char *string_desc_arr[] = {
    (const char[]){0x09, 0x04},   /* 0: en-US */
    "icelink-fast",               /* 1: manufacturer */
    "ECP5 JTAG probe",            /* 2: product */
    "000001",                     /* 3: serial */
    "icelink console",            /* 4: CDC 0 -- probe console */
    "icelink target UART",        /* 5: CDC 1 -- USART2 bridge */
};

static uint16_t _desc_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    uint8_t chr_count;

    if (index == 0) {
        _desc_str[1] = 0x0409;
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))
            return NULL;

        const char *str = string_desc_arr[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31) chr_count = 31;

        for (uint8_t i = 0; i < chr_count; i++)
            _desc_str[1 + i] = str[i];
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
