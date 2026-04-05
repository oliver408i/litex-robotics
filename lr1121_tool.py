#!/usr/bin/env python3
import argparse
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

# Mock/Import Logic
try:
    from uart_client import UartProtocol
except ImportError:
    class UartProtocol:
        def __init__(self, port, baud): pass
        def spi_xfer(self, bits, val): return 0x00
        def spi_set_cs(self, mask, val): pass
        def get_radio_gpio(self): return (False, False)
        def set_radio_reset(self, val): pass
        def close(self): pass

LR1121_CS_MASK = 0x02

@dataclass
class StatusDecoded:
    stat1: int
    stat2: int

    def is_cmd_ok(self) -> bool:
        return ((self.stat1 >> 1) & 0x07) == 0x02

    def cmd_index(self) -> int:
        return (self.stat1 >> 1) & 0x07

    def is_cmd_error(self) -> bool:
        return self.cmd_index() in (3, 4, 5)

    def __repr__(self):
        cmd_names = ["res", "pending", "CMD_OK", "CMD_TIMEOUT", "CMD_ERROR", "EXEC_FAIL", "TX_DONE", "res"]
        mode_names = ["sleep", "stby_rc", "stby_xosc", "fs", "rx", "tx", "wifi/gnss"]
        cmd_idx = (self.stat1 >> 1) & 0x07
        mode_idx = (self.stat2 >> 1) & 0x07
        cmd_stat = cmd_names[cmd_idx] if cmd_idx < len(cmd_names) else f"0x{cmd_idx:x}"
        mode = mode_names[mode_idx] if mode_idx < len(mode_names) else f"0x{mode_idx:x}"
        return f"<Status: {cmd_stat}, Mode: {mode}>"

class LR1121:
    def __init__(self, client: UartProtocol, timeout: float = 1.0):
        self.client = client
        self.timeout = timeout
        self.strict_busy = True 
        self.invert_busy = False

    def _get_busy_state(self) -> bool:
        busy_raw, _ = self.client.get_radio_gpio()
        return not busy_raw if self.invert_busy else busy_raw

    def _wait_not_busy(self, custom_timeout: Optional[float] = None):
        if not self.strict_busy:
            time.sleep(0.01)
            return
        t = custom_timeout if custom_timeout is not None else self.timeout
        deadline = time.time() + t
        while time.time() < deadline:
            if not self._get_busy_state():
                return
            time.sleep(0.001)
        raise TimeoutError("LR1121 BUSY timeout")

    def _cs_set(self, state: bool):
        self.client.spi_set_cs(LR1121_CS_MASK if state else 0, 1 if state else 0)

    def wakeup(self):
        self._cs_set(True)
        time.sleep(0.05)
        self._cs_set(False)
        time.sleep(0.02)
        try:
            self._wait_not_busy(0.3)
        except: pass

    def reset(self):
        self.client.set_radio_reset(0)
        time.sleep(0.05)
        self.client.set_radio_reset(1)
        time.sleep(0.05)
        self._wait_not_busy(0.5)

    def write_command(self, opcode: int, args: bytes = b"", check_busy: bool = True) -> StatusDecoded:
        if check_busy: self._wait_not_busy()
        tx = bytes([(opcode >> 8) & 0xFF, opcode & 0xFF]) + args
        self._cs_set(True)
        rx = [self.client.spi_xfer(8, b) & 0xFF for b in tx]
        self._cs_set(False)
        return StatusDecoded(stat1=rx[0], stat2=rx[1])

    def read_command(self, opcode: int, args: bytes = b"", read_len: int = 0, check_busy: bool = True) -> Tuple[StatusDecoded, bytes]:
        if check_busy: self._wait_not_busy()
        # FIX: The LR11xx requires 1 Dummy Byte (0x00) after args but before data
        tx = bytes([(opcode >> 8) & 0xFF, opcode & 0xFF]) + args + b"\x00" + bytes(read_len)
        self._cs_set(True)
        rx = [self.client.spi_xfer(8, b) & 0xFF for b in tx]
        self._cs_set(False)
        
        status = StatusDecoded(stat1=rx[0], stat2=rx[1])
        data_start = 2 + len(args) + 1 # Opcode + Args + Dummy
        return status, bytes(rx[data_start:])

    # --- Commands ---

    def get_version(self):
        status, data = self.read_command(0x0101, read_len=4)
        if len(data) >= 4:
            return {"use": data[0], "hw": data[1], "fw": f"{data[2]}.{data[3]}"}
        return None

    def calibrate(self, param: int = 0x3F):
        return self.write_command(0x010F, bytes([param]))

    def calibrate_image(self, freq1: int, freq2: int):
        return self.write_command(0x0110, bytes([freq1, freq2]))

    def clear_irq(self, mask: int = 0xFFFFFFFF):
        return self.write_command(0x0103, mask.to_bytes(4, 'big'))

    def get_irq_status(self) -> int:
        status, data = self.read_command(0x0102, read_len=4)
        return int.from_bytes(data, 'big')

    def set_standby(self, mode: int = 0x00):
        return self.write_command(0x0105, bytes([mode]))

    def set_rf_frequency(self, freq_hz: int):
        return self.write_command(0x020B, freq_hz.to_bytes(4, 'big'))

    def set_packet_type(self, p_type: int):
        return self.write_command(0x0200, bytes([p_type]))

    def set_pa_config(self, pa_sel: int, reg_mode: int, duty_cycle: int, hp_sel: int):
        return self.write_command(0x0213, bytes([pa_sel, reg_mode, duty_cycle, hp_sel]))

    def set_tx_params(self, power: int, ramp_time: int = 0x04):
        return self.write_command(0x020F, bytes([power & 0xFF, ramp_time]))

    def set_lora_mod_params(self, sf: int, bw: int, cr: int, ldro: int = 0):
        return self.write_command(0x0202, bytes([sf, bw, cr, ldro]))

    def set_lora_pkt_params(self, pre: int, hdr: int, pay: int, crc: int, inv: int = 0):
        return self.write_command(0x0203, pre.to_bytes(2, 'big') + bytes([hdr, pay, crc, inv]))

    def set_tx(self, timeout_ms: int = 0):
        return self.write_command(0x020A, timeout_ms.to_bytes(3, 'big'))

    def write_buffer(self, offset: int, data: bytes):
        return self.write_command(0x0109, bytes([offset]) + data)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    args = parser.parse_args()

    client = UartProtocol(args.port, 1000000)
    radio = LR1121(client)
    
    try:
        print("--- Initializing LR1121 ---")
        radio.reset()
        radio.wakeup()
        
        # 1. Essential Calibration (PLL/Image)
        print("Calibrating...")
        radio.calibrate(0x3F) 
        radio.calibrate_image(0xD1, 0xD2) # Calibration for 915MHz band
        radio.set_standby(0x00)
        
        print(f"Chip Version: {radio.get_version()}")
        
        # 2. Config
        radio.set_packet_type(1) # LoRa
        radio.set_rf_frequency(915000000)
        radio.set_pa_config(0x01, 0x01, 0x04, 0x00) # HP PA
        radio.set_tx_params(14, 0x04)
        
        # 3. LoRa Params (SF7, BW125kHz, CR4/5)
        radio.set_lora_mod_params(0x07, 0x04, 0x01, 0x00)
        radio.set_lora_pkt_params(8, 0x00, 12, 0x01, 0x00)
        
        # 4. Data
        radio.clear_irq()
        radio.write_buffer(0, b"HELLO_LR1121")
        
        print("Starting TX...")
        radio.set_tx(0) # Infinite/Zero timeout
        
        # 5. Poll for TX Done
        start = time.time()
        while time.time() - start < 5:
            irq = radio.get_irq_status()
            if irq & 0x08:
                print("TX DONE!")
                break
            time.sleep(0.1)
        else:
            print("TX Timed out.")

    finally:
        client.close()

if __name__ == "__main__":
    main()