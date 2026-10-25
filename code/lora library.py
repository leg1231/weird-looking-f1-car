# lora.py  – SX1276/SX1278 driver for MicroPython (Raspberry Pi Pico)
# Fixes:
#   - DIO0 explicitly mapped to TX Done / RX Done via REG_DIO_MAPPING1 (0x40)
#   - PA_BOOST enabled by default (suits almost every common breakout board)
#   - TX wait has a hard 3-second timeout; radio self-recovers on timeout
#   - Explicit LoRa mode bit set in every set_mode() call so the mode byte
#     can never accidentally clear the LoRa flag
#   - Receive returns (payload, rssi) tuple so callers can inspect signal quality
#   - All register constants are named at the top for readability

from machine import Pin, SPI
import time

# ── Register map (SX1276) ────────────────────────────────────────────────────
REG_FIFO            = 0x00
REG_OP_MODE         = 0x01
REG_FRF_MSB         = 0x06
REG_FRF_MID         = 0x07
REG_FRF_LSB         = 0x08
REG_PA_CONFIG       = 0x09
REG_LNA             = 0x0C
REG_FIFO_ADDR_PTR   = 0x0D
REG_FIFO_TX_BASE    = 0x0E
REG_FIFO_RX_BASE    = 0x0F
REG_FIFO_RX_CURRENT = 0x10
REG_IRQ_FLAGS       = 0x12
REG_RX_NB_BYTES     = 0x13
REG_PKT_RSSI        = 0x1A
REG_MODEM_CONFIG1   = 0x1D
REG_MODEM_CONFIG2   = 0x1E
REG_MODEM_CONFIG3   = 0x26
REG_PAYLOAD_LENGTH  = 0x22
REG_DIO_MAPPING1    = 0x40
REG_VERSION         = 0x42

# ── Mode constants (bit 7 = LoRa, bits 2:0 = mode) ──────────────────────────
MODE_SLEEP      = 0x80   # LoRa | sleep
MODE_STDBY      = 0x81   # LoRa | standby
MODE_TX         = 0x83   # LoRa | TX
MODE_RX_CONT    = 0x85   # LoRa | RX continuous

# ── IRQ flag masks ────────────────────────────────────────────────────────────
IRQ_TX_DONE     = 0x08
IRQ_RX_DONE     = 0x40
IRQ_CRC_ERROR   = 0x20

TX_TIMEOUT_MS   = 3000   # ms before send() gives up and self-recovers


class LoRa:
    def __init__(self, spi, cs, rst, dio0, freq=915e6,
                 pa_boost=True, tx_power=17):
        """
        spi       – configured SPI object
        cs        – chip-select Pin
        rst       – reset Pin
        dio0      – DIO0 Pin (used for TX-done and RX-done interrupts)
        freq      – centre frequency in Hz  (default 915 MHz)
        pa_boost  – True for PA_BOOST pin modules (most common breakouts)
        tx_power  – output power in dBm (2–17 for PA_BOOST, 0–14 for RFO)
        """
        self.spi      = spi
        self.cs       = cs
        self.rst      = rst
        self.dio0     = dio0
        self.freq     = freq
        self.pa_boost = pa_boost
        self.tx_power = tx_power

        self.cs.init(Pin.OUT,  value=1)
        self.rst.init(Pin.OUT, value=1)
        self.dio0.init(Pin.IN)

        self.reset()
        self._check_version()
        self.init_radio()

    # ── Low-level SPI helpers ────────────────────────────────────────────────

    def _write_reg(self, addr, val):
        self.cs.value(0)
        self.spi.write(bytearray([addr | 0x80, val]))
        self.cs.value(1)

    def _read_reg(self, addr):
        self.cs.value(0)
        self.spi.write(bytearray([addr & 0x7F]))
        val = self.spi.read(1)[0]
        self.cs.value(1)
        return val

    # ── Utility ──────────────────────────────────────────────────────────────

    def reset(self):
        self.rst.value(0)
        time.sleep_ms(10)
        self.rst.value(1)
        time.sleep_ms(10)

    def _check_version(self):
        ver = self._read_reg(REG_VERSION)
        if ver != 0x12:
            raise OSError(
                "SX1276 not found (REG_VERSION=0x{:02X}). "
                "Check wiring / SPI pins.".format(ver)
            )

    def set_mode(self, mode):
        self._write_reg(REG_OP_MODE, mode)
        time.sleep_ms(2)

    def _clear_irq(self):
        self._write_reg(REG_IRQ_FLAGS, 0xFF)

    # ── Radio initialisation ──────────────────────────────────────────────────

    def init_radio(self):
        self.set_mode(MODE_SLEEP)          # must be in sleep to write freq

        # Frequency
        frf = int((self.freq / 32_000_000) * (1 << 19))
        self._write_reg(REG_FRF_MSB, (frf >> 16) & 0xFF)
        self._write_reg(REG_FRF_MID, (frf >>  8) & 0xFF)
        self._write_reg(REG_FRF_LSB,  frf        & 0xFF)

        # PA / TX power
        if self.pa_boost:
            # PA_BOOST pin, power = 2 + (OutputPower)  → max 17 dBm
            pwr = max(2, min(17, self.tx_power))
            self._write_reg(REG_PA_CONFIG, 0x80 | (pwr - 2))
        else:
            # RFO pin, power = -1 + (OutputPower) → max 13 dBm
            pwr = max(0, min(14, self.tx_power))
            self._write_reg(REG_PA_CONFIG, 0x70 | pwr)

        # FIFO base addresses
        self._write_reg(REG_FIFO_TX_BASE, 0x00)
        self._write_reg(REG_FIFO_RX_BASE, 0x00)

        # LNA: max gain + boost
        self._write_reg(REG_LNA, 0x23)

        # Modem config: BW=125 kHz, CR=4/5, explicit header
        self._write_reg(REG_MODEM_CONFIG1, 0x72)
        # SF=7, normal mode, CRC on
        self._write_reg(REG_MODEM_CONFIG2, 0x74)
        # AGC on
        self._write_reg(REG_MODEM_CONFIG3, 0x04)

        # DIO0 → TX Done when transmitting (bits 7:6 = 01)
        # We'll switch this per-operation; start with RX Done (00)
        self._write_reg(REG_DIO_MAPPING1, 0x00)

        self._clear_irq()
        self.set_mode(MODE_STDBY)

    # ── Send ─────────────────────────────────────────────────────────────────

    def send(self, msg):
        """
        Transmit a string.  Returns True on success, False on timeout.
        On timeout the radio is fully re-initialised so it stays usable.
        """
        data = msg.encode() if isinstance(msg, str) else msg

        self.set_mode(MODE_STDBY)

        # Write payload into FIFO
        self._write_reg(REG_FIFO_ADDR_PTR, 0x00)
        for b in data:
            self._write_reg(REG_FIFO, b)
        self._write_reg(REG_PAYLOAD_LENGTH, len(data))

        # Map DIO0 → TX Done (bits 7:6 = 01)
        self._write_reg(REG_DIO_MAPPING1, 0x40)

        self._clear_irq()
        self.set_mode(MODE_TX)

        # Wait for TX Done with a hard timeout
        deadline = time.ticks_add(time.ticks_ms(), TX_TIMEOUT_MS)
        while not self.dio0.value():
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                print("[LoRa] TX timeout – reinitialising radio")
                self.reset()
                self.init_radio()
                return False
            time.sleep_ms(1)

        self._clear_irq()
        # Return DIO0 mapping to RX Done for receive calls
        self._write_reg(REG_DIO_MAPPING1, 0x00)
        self.set_mode(MODE_STDBY)
        time.sleep_ms(10)
        return True

    # ── Receive ───────────────────────────────────────────────────────────────

    def receive(self):
        """
        Non-blocking receive poll.  Call repeatedly in your main loop.

        Returns (payload_str, rssi_dBm) if a valid packet arrived,
        or (None, None) if nothing is ready yet.
        On a CRC error returns (None, None) and prints a warning.
        """
        # Ensure we're in RX continuous and DIO0 → RX Done (bits 7:6 = 00)
        if self._read_reg(REG_OP_MODE) != MODE_RX_CONT:
            self._write_reg(REG_DIO_MAPPING1, 0x00)
            self.set_mode(MODE_RX_CONT)

        if not self.dio0.value():
            return None, None   # nothing yet

        irq = self._read_reg(REG_IRQ_FLAGS)
        self._clear_irq()

        if irq & IRQ_CRC_ERROR:
            print("[LoRa] CRC error – packet dropped")
            return None, None

        if not (irq & IRQ_RX_DONE):
            return None, None

        length   = self._read_reg(REG_RX_NB_BYTES)
        fifo_ptr = self._read_reg(REG_FIFO_RX_CURRENT)
        self._write_reg(REG_FIFO_ADDR_PTR, fifo_ptr)

        data = bytearray()
        for _ in range(length):
            data.append(self._read_reg(REG_FIFO))

        # RSSI = -157 + reg value  (for HF port, 915 MHz)
        rssi = self._read_reg(REG_PKT_RSSI) - 157

        self.set_mode(MODE_STDBY)

        try:
            return data.decode(), rssi
        except Exception:
            return data, rssi   # return raw bytes if UTF-8 decode fails
