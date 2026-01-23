LiteX UART Robotics Protocol v1.0

Overview
- Transport: UART, 750000 8N1.
- Framing: preamble + length + command + payload + checksum.
- Endianness: little-endian for multi-byte fields.
- Responses: command | 0x80 on success, 0x7F on error.

Frame Format
```
Byte 0   : 0xAA
Byte 1   : 0x55
Byte 2   : LEN (number of bytes in CMD+PAYLOAD)
Byte 3   : CMD
Byte 4.. : PAYLOAD (LEN-1 bytes)
Last     : CHECKSUM (xor of LEN, CMD, PAYLOAD bytes)
```

Error Response
- CMD: 0x7F
- PAYLOAD: [orig_cmd, error_code]
- error_code:
  - 1: bad length
  - 2: bad checksum
  - 3: unknown command
  - 4: bad index

Commands
- 0x01 PING
  - req payload: none
  - resp payload: ASCII "PONG"
- 0x02 GET_VERSION
  - req payload: none
  - resp payload: [major, minor]
- 0x10 SET_MOTOR
  - req payload: [index, speed_lo, speed_hi] (int16)
  - resp payload: [index]
- 0x11 GET_MOTOR
  - req payload: [index]
  - resp payload: [index, speed_lo, speed_hi]
- 0x12 SET_SERVO
  - req payload: [index, pulse_lo, pulse_hi] (uint16, microseconds)
  - resp payload: [index]
- 0x13 GET_SERVO
  - req payload: [index]
  - resp payload: [index, pulse_lo, pulse_hi]
- 0x14 SET_GPIO
  - req payload: [mask_u32, value_u32]
  - resp payload: none
- 0x15 GET_GPIO
  - req payload: none
  - resp payload: [mask_u32, value_u32]
- 0x16 ESTOP
  - req payload: none
  - resp payload: none (clears all motor speeds)
- 0x20 GET_STATUS
  - req payload: none
  - resp payload: [uptime_ms_u32, last_error_u8]
- 0x30 SET_NEOPIXEL
  - req payload: [en, brightness, g, r, b]
  - resp payload: none
- 0x31 GET_NEOPIXEL
  - req payload: none
  - resp payload: [en, brightness, g, r, b]
- 0x32 SET_STRIP
  - req payload: [index_lo, index_hi, g, r, b] (index 0 = LED1, 298 = LED299)
  - resp payload: none
- 0x33 SET_STRIP_BRI
  - req payload: [index_lo, index_hi, g, r, b, brightness] (brightness 0-255)
  - resp payload: none
- 0x34 SET_STRIP_BULK
  - req payload: [start_lo, start_hi, count, g0, r0, b0, g1, r1, b1, ...]
  - resp payload: none
- 0x35 SET_STRIP_INTERP
  - req payload: [color_step, brightness_step]
  - resp payload: none
- 0x40 GET_ADC
  - req payload: none
  - resp payload: [ch0_mv_u16..ch7_mv_u16, update_mask_u8, last_channel_u8]
- 0x41 SET_ADC_CFG
  - req payload: [enable, channel_mask, interval_ticks_u32]
  - resp payload: none
- 0x42 CLR_ADC_UPD
  - req payload: [update_mask]
  - resp payload: none
- 0x50 GET_ESTOP
  - req payload: none
  - resp payload: [estop_active, debounced_level, raw_active]
- 0x60 GET_AS5600
  - req payload: none
  - resp payload: [present, ok, status, angle_lo, angle_hi, magnitude_lo, magnitude_hi]

Notes
- SET_MOTOR/SET_SERVO only update internal state in firmware.
- NeoPixel control is enabled by default in this build.
- Strip interpolation steps are per-millisecond; set both to 0 for immediate updates.
- ADC control requires MCP3008 module enabled (default in this build).
- E-STOP is latched on press; hold for 2 seconds to release.
- Integrate your motor/servo drivers by consuming motor_speed and servo_pulse_us.
