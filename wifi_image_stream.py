"""SeekFree WiFi SPI and SeekFree Assistant image protocol for OMV RT5.

Wiring (signal names are from the OMV RT5 SPI-master side):
    P0  MOSI -> WiFi MOSI
    P1  MISO <- WiFi MISO
    P2  SCLK -> WiFi SCK
    P3  GPIO -> WiFi CS
    P6  GPIO <- WiFi INT
    P7  GPIO -> WiFi RST
    5V       -> WiFi 5V
    GND      -> WiFi GND

The transport is ported from zf_device_wifi_spi.c. Image frames use the
standard seekfree_assistant CAMERA header followed by raw grayscale pixels.
"""

import time

try:
    import ustruct as struct
except ImportError:
    import struct

from machine import Pin, SPI


class WifiSPIError(Exception):
    pass


class SeekfreeWifiSPI:
    # Commands sent by the SPI master.
    CMD_RESET = 0x01
    CMD_DATA = 0x02
    CMD_UDP_SEND = 0x03
    CMD_CLOSE_SOCKET = 0x04
    CMD_SET_WIFI = 0x10
    CMD_SET_SOCKET = 0x11
    CMD_GET_VERSION = 0x20
    CMD_GET_MAC = 0x21
    CMD_GET_IP = 0x22

    # Replies returned by the WiFi module.
    REPLY_OK = 0x80
    REPLY_ERROR = 0x81
    REPLY_DATA_START = 0x90
    REPLY_DATA_END = 0x91
    REPLY_VERSION = 0xA0
    REPLY_MAC = 0xA1
    REPLY_IP = 0xA2

    RECEIVE_SIZE = 32
    TRANSFER_SIZE = 4088

    def __init__(self, spi_id=2, baudrate=30_000_000,
                 sck="P2", mosi="P0", miso="P1",
                 cs="P3", int_pin="P6", rst="P7"):
        self.spi = self._create_spi(spi_id, baudrate, sck, mosi, miso)
        self.cs = self._create_output(cs, 1)
        self.rst = self._create_output(rst, 1)
        self.int_pin = self._create_input(int_pin)
        self.version = ""
        self.mac = ""
        self.ip_info = ""

    @staticmethod
    def _create_spi(spi_id, baudrate, sck, mosi, miso):
        firstbit = getattr(SPI, "MSB", 0)

        # Newer OpenMV firmware accepts explicit machine.SPI pins.
        try:
            return SPI(spi_id, baudrate=baudrate, polarity=0, phase=0,
                       bits=8, firstbit=firstbit,
                       sck=Pin(sck), mosi=Pin(mosi), miso=Pin(miso))
        except (TypeError, ValueError):
            pass

        # On OMV RT5, hardware SPI2 is fixed to P0/P1/P2.
        try:
            return SPI(spi_id, baudrate=baudrate, polarity=0, phase=0,
                       bits=8, firstbit=firstbit)
        except (TypeError, ValueError):
            master = getattr(SPI, "MASTER", 0)
            return SPI(spi_id, master, baudrate=baudrate,
                       polarity=0, phase=0)

    @staticmethod
    def _create_output(name, initial):
        mode = getattr(Pin, "OUT", None)
        if mode is None:
            mode = Pin.OUT_PP
        try:
            pin = Pin(name, mode, value=initial)
        except TypeError:
            pin = Pin(name, mode)
            pin.value(initial)
        return pin

    @staticmethod
    def _create_input(name):
        pull_down = getattr(Pin, "PULL_DOWN", None)
        try:
            if pull_down is None:
                return Pin(name, Pin.IN)
            return Pin(name, Pin.IN, pull_down)
        except TypeError:
            return Pin(name, Pin.IN, pull=pull_down)

    @staticmethod
    def _header(command, length=0):
        # MSPM0 is little-endian: command, reserve, uint16 length.
        return struct.pack("<BBH", command, 0, length)

    @staticmethod
    def _parse_header(data):
        return struct.unpack("<BBH", bytes(data[:4]))

    def _wait_ready(self, timeout_ms):
        start = time.ticks_ms()
        while self.int_pin.value() == 0:
            if time.ticks_diff(time.ticks_ms(), start) >= timeout_ms:
                return False
            time.sleep_us(10)
        return True

    def _spi_transfer(self, tx_data):
        rx_data = bytearray(len(tx_data))
        if hasattr(self.spi, "write_readinto"):
            self.spi.write_readinto(tx_data, rx_data)
            return rx_data

        try:
            result = self.spi.send_recv(tx_data, timeout=5000)
        except TypeError:
            result = self.spi.send_recv(tx_data)
        rx_data[:] = result
        return rx_data

    def _spi_write(self, data):
        if hasattr(self.spi, "write"):
            self.spi.write(data)
        else:
            try:
                self.spi.send(data, timeout=5000)
            except TypeError:
                self.spi.send(data)

    def _write_packet(self, command, payload=b""):
        self.cs.value(0)
        try:
            self._spi_write(self._header(command, len(payload)))
            if payload:
                self._spi_write(payload)
        finally:
            self.cs.value(1)

    def _transfer_command(self, command, length=0, receive_size=0):
        self.cs.value(0)
        try:
            rx_head = self._spi_transfer(self._header(command, length))
            rx_body = bytearray()
            if receive_size:
                rx_body = self._spi_transfer(bytearray(receive_size))
        finally:
            self.cs.value(1)

        reply, _, reply_length = self._parse_header(rx_head)
        return reply, bytes(rx_body[:min(reply_length, len(rx_body))])

    def _set_parameter(self, command, payload, timeout_ms):
        if not self._wait_ready(timeout_ms):
            return False

        self._write_packet(command, payload)
        if not self._wait_ready(timeout_ms):
            return False

        reply, _ = self._transfer_command(self.CMD_DATA)
        time.sleep_us(20)
        return reply == self.REPLY_OK

    def _get_parameter(self, command, timeout_ms=1000):
        if not self._wait_ready(timeout_ms):
            raise WifiSPIError("WiFi SPI was not ready before command 0x%02X" % command)

        # The original driver clocks 32 bytes for a get request.
        request = self._header(command, 0) + bytes(self.RECEIVE_SIZE - 4)
        self.cs.value(0)
        try:
            self._spi_write(request)
        finally:
            self.cs.value(1)

        if not self._wait_ready(timeout_ms):
            raise WifiSPIError("WiFi SPI response timeout for command 0x%02X" % command)

        return self._transfer_command(self.CMD_DATA, 0, self.RECEIVE_SIZE)

    def _transfer_data(self, data):
        length = len(data)
        first_length = min(length, self.RECEIVE_SIZE)
        first_block = bytearray(self.RECEIVE_SIZE)
        first_block[:first_length] = data[:first_length]

        self.cs.value(0)
        try:
            rx_head = self._spi_transfer(self._header(self.CMD_DATA, length))
            rx_body = self._spi_transfer(first_block)
            if length > self.RECEIVE_SIZE:
                self._spi_write(data[self.RECEIVE_SIZE:])
        finally:
            self.cs.value(1)

        reply, _, reply_length = self._parse_header(rx_head)
        return reply, bytes(rx_body[:min(reply_length, self.RECEIVE_SIZE)])

    def hard_reset(self):
        self.rst.value(0)
        time.sleep_ms(10)
        self.rst.value(1)
        time.sleep_ms(100)

    def initialize(self, ssid=None, password=None):
        self.cs.value(1)
        self.hard_reset()

        reply, payload = self._get_parameter(self.CMD_GET_VERSION)
        if reply != self.REPLY_VERSION:
            raise WifiSPIError("WiFi module version query failed: 0x%02X" % reply)
        self.version = payload.rstrip(b"\x00").decode("ascii", "ignore")

        reply, payload = self._get_parameter(self.CMD_GET_MAC)
        if reply == self.REPLY_MAC:
            self.mac = payload.rstrip(b"\x00").decode("ascii", "ignore")

        if ssid:
            self.connect_wifi(ssid, password)
        return True

    def connect_wifi(self, ssid, password=None):
        if password:
            text = "%s\r\n%s\r\n" % (ssid, password)
        else:
            text = "%s\r\n" % ssid
        if not self._set_parameter(self.CMD_SET_WIFI, text.encode(), 10_000):
            raise WifiSPIError("Unable to connect to WiFi: %s" % ssid)
        return True

    def connect_socket(self, transport, ip_address, port, local_port="6666"):
        transport = transport.upper()
        if transport not in ("TCP", "UDP"):
            raise ValueError("transport must be TCP or UDP")
        text = "%s\r\n%s\r\n%s\r\n%s\r\n" % (
            transport, ip_address, port, local_port)
        if not self._set_parameter(self.CMD_SET_SOCKET, text.encode(), 50_000):
            raise WifiSPIError("Unable to connect %s socket to %s:%s" %
                               (transport, ip_address, port))
        return True

    def disconnect_socket(self):
        reply, _ = self._get_parameter(self.CMD_CLOSE_SOCKET)
        return reply == self.REPLY_OK

    def send_buffer(self, data, timeout_ms=1000):
        """Send bytes and return the number of bytes not sent."""
        view = memoryview(data)
        offset = 0
        last_reply = self.REPLY_DATA_END

        while offset < len(view):
            if not self._wait_ready(timeout_ms):
                break

            send_length = min(self.TRANSFER_SIZE, len(view) - offset)
            last_reply, _ = self._transfer_data(view[offset:offset + send_length])
            offset += send_length

        # Drain any data that the module reports during full-duplex transfers.
        while last_reply == self.REPLY_DATA_START:
            if not self._wait_ready(timeout_ms):
                break
            last_reply, _ = self._transfer_command(
                self.CMD_DATA, 0, self.RECEIVE_SIZE)

        return len(view) - offset

    def udp_send_now(self):
        if not self._wait_ready(1000):
            return False
        self._transfer_command(self.CMD_UDP_SEND, 0, self.RECEIVE_SIZE)
        if not self._wait_ready(1000):
            return False
        reply, _ = self._transfer_command(self.CMD_DATA)
        return reply == self.REPLY_OK


class WifiImageStreamer:
    """Send OpenMV grayscale images using the SeekFree Assistant protocol."""

    ASSISTANT_HEAD = 0xAA
    ASSISTANT_CAMERA_CMD = 0x02
    CAMERA_TYPE_GRAY = 0x02
    DATA_TYPE_LITTLE_ENDIAN = 0x01

    def __init__(self, wifi, max_fps=5):
        self.wifi = wifi
        self.frame_id = 0
        self.interval_ms = max(1, 1000 // max_fps)
        self.next_frame_ms = 0

    def frame_due(self):
        now = time.ticks_ms()
        if time.ticks_diff(now, self.next_frame_ms) < 0:
            return False
        self.next_frame_ms = time.ticks_add(now, self.interval_ms)
        return True

    def send_image(self, image):
        width = image.width()
        height = image.height()
        try:
            payload = image.bytearray()
        except AttributeError:
            payload = bytes(image)

        expected_size = width * height
        if len(payload) != expected_size:
            raise WifiSPIError(
                "Expected %d grayscale bytes, got %d" %
                (expected_size, len(payload)))

        # This exactly mirrors seekfree_assistant_camera_config() for a
        # little-endian grayscale image. The config block is 12 bytes.
        header = bytearray(12)
        header[0] = self.ASSISTANT_HEAD
        header[2] = self.ASSISTANT_CAMERA_CMD
        header[3] = self.CAMERA_TYPE_GRAY
        header[4] = width & 0xFF
        header[5] = (width >> 8) & 0xFF
        header[6] = height & 0xFF
        header[7] = (height >> 8) & 0xFF
        header[8] = self.DATA_TYPE_LITTLE_ENDIAN
        header[1] = (self.ASSISTANT_HEAD + sum(header[2:])) & 0xFF

        if self.wifi.send_buffer(header) != 0:
            return False
        if self.wifi.send_buffer(payload) != 0:
            return False

        self.frame_id = (self.frame_id + 1) & 0xFFFFFFFF
        return True
