/* TinyUSB configuration -- CDC only, device only, single FS port.
 *
 * The point of USB in this build is to have *any* output channel: the i9 SODIMM
 * physically covers the LED once inserted, so a blink code is unreadable and
 * the grabbers are off. CDC gets us a serial port to print IDCODE to.
 */
#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#define CFG_TUSB_MCU              OPT_MCU_STM32F1
#define CFG_TUSB_OS               OPT_OS_NONE
#define CFG_TUSB_RHPORT0_MODE     (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

#define CFG_TUD_ENABLED           1
#define CFG_TUD_MAX_SPEED         OPT_MODE_FULL_SPEED

/* The F103's USB peripheral copies to/from a 512-byte packet memory area, so
 * endpoint budget is tight -- CDC and nothing else. */
#define CFG_TUD_CDC               1
#define CFG_TUD_MSC               0
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            0

#define CFG_TUD_CDC_RX_BUFSIZE    64
#define CFG_TUD_CDC_TX_BUFSIZE    256
#define CFG_TUD_CDC_EP_BUFSIZE    64

#define CFG_TUD_ENDPOINT0_SIZE    64

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif
#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))
#endif

#endif
