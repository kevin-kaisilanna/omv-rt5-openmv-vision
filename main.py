import sensor
import time
from machine import PWM
from machine import UART


# UART1 is mapped to P4/P5 on the OMV RT5 board.
UART_BAUD = 115200

# Leave WIFI_SSID empty when only UART tracking data is required.
WIFI_SSID = "YOUR_WIFI_SSID"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"
SERVER_IP = "192.168.1.100"
SERVER_PORT = "8086"
LOCAL_PORT = "6666"
IMAGE_MAX_FPS = 5


fill_light_duty = 65535

# UART is initialized before the sensor, PWM and WiFi. This keeps the
# diagnostic heartbeat available even when another peripheral fails.
uart = UART(1, baudrate=UART_BAUD)


def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def send_tracking_packet(line_error, distance_raw, cross_flag):
    line_error = clamp(int(line_error), -120, 120)
    distance_raw = clamp(int(distance_raw), 0, 255)
    cross_flag = 1 if cross_flag else 0

    packet = bytearray(6)
    packet[0] = 0xFF
    packet[1] = 0xFE
    packet[2] = line_error + 120
    packet[3] = distance_raw
    packet[4] = cross_flag
    packet[5] = sum(packet[0:5]) & 0xFF
    uart.write(packet)


def calculate_cross_product(p1, p2, p3):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    return (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2)


def create_image_streamer():
    if not WIFI_SSID:
        return None

    # Import only when WiFi streaming is enabled. A missing WiFi module must
    # never prevent UART tracking data from being sent.
    from wifi_image_stream import SeekfreeWifiSPI, WifiImageStreamer

    wifi = SeekfreeWifiSPI(
        spi_id=2,
        baudrate=30000000,
        sck="P2",
        mosi="P0",
        miso="P1",
        cs="P3",
        int_pin="P6",
        rst="P7")
    wifi.initialize(WIFI_SSID, WIFI_PASSWORD or None)
    wifi.connect_socket("TCP", SERVER_IP, SERVER_PORT, LOCAL_PORT)
    print("WiFi SPI ready:", wifi.version, wifi.mac)
    return WifiImageStreamer(wifi, max_fps=IMAGE_MAX_FPS)


print("OpenMV UART1 ready on P4/P5")
send_tracking_packet(0, 120, 0)

sensor_ready = False
try:
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.QQVGA)
    sensor.skip_frames(time=2000)
    sensor_ready = True
except Exception as error:
    print("Sensor init failed:", error)

clock = time.clock()

pwm1 = None
pwm2 = None
try:
    pwm1 = PWM("P13", freq=200, duty_u16=65535)
    pwm2 = PWM("P14", freq=200, duty_u16=65535)
    pwm1.duty_u16(0)
    pwm2.duty_u16(0)
except Exception as error:
    pwm1 = None
    pwm2 = None
    print("Indicator PWM init failed:", error)



image_streamer = None
try:
    image_streamer = create_image_streamer()
except Exception as error:
    print("WiFi SPI init failed:", error)

edge = [0] * 160
switch_flag = 0
led_duty = 0
led_duty_delta = 2560
debug_frame_count = 0
frame_error_count = 0

# Send several packets after the two-second sensor startup delay. This makes
# the MCU-side frame counter move even before the first processed image.
for _ in range(3):
    send_tracking_packet(0, 120, 0)
    time.sleep_ms(20)


while True:
    if not sensor_ready:
        send_tracking_packet(0, 120, 1)
        time.sleep_ms(100)
        continue

    try:
        clock.tick()
        next_start_y = 0
        img = sensor.snapshot()

        histogram = img.get_histogram()
        threshold = histogram.get_threshold()
        img.binary([(threshold.value(), 255)])

        for x in range(5, 155):
            start_y = clamp(next_start_y, 0, 109)
            for y in range(start_y, 110):
                pixel = img.get_pixel(x, y)
                if pixel == 0 and img.get_pixel(x, y + 1) == 255:
                    next_start_y = clamp(y - 10, 0, 109)
                    edge[x] = y
                    img.draw_line(x, y, x, y + 5, 255)
                    break
                if y == 109:
                    if pixel == 0:
                        next_start_y = 108
                        edge[x] = y
                        img.draw_line(x, y, x, y + 5, 255)
                    else:
                        next_start_y = 0
                        edge[x] = 0
                        img.draw_line(x, 0, x, 5, 255)
                    break

        while uart.any():
            rx_data = uart.read(1)
            if rx_data == b"\xFF":
                switch_flag = 1
            elif rx_data == b"\xFE":
                switch_flag = 0

        if switch_flag == 0:
            cross = calculate_cross_product(
                (90, edge[90]), (110, edge[110]), (130, edge[130]))
            line_error = (
                edge[120] + edge[125] + edge[130] + edge[135]
                - edge[85] - edge[90] - edge[95] - edge[100])
        else:
            cross = calculate_cross_product(
                (60, edge[60]), (80, edge[80]), (100, edge[100]))
            line_error = (
                edge[90] + edge[95] + edge[100] + edge[105]
                - edge[55] - edge[60] - edge[65] - edge[75])

        distance_average = (edge[109] + edge[110] + edge[111]) / 3.0
        distance_raw = int(distance_average) - 72 + 120

        if (edge[100] > 95 and edge[110] > 95 and
                edge[120] > 95 and line_error == 0):
            line_error = 70

        cross_flag = 0 if cross > -30 else 1
        send_tracking_packet(line_error, distance_raw, cross_flag)

        led_duty += led_duty_delta
        if led_duty >= 65535:
            led_duty = 65535
            led_duty_delta = -2560
        elif led_duty <= 1:
            led_duty = 1
            led_duty_delta = 2560

        if pwm1 is not None and pwm2 is not None:
            if cross_flag == 0:
                pwm2.duty_u16(led_duty)
                pwm1.duty_u16(0)
            else:
                pwm1.duty_u16(led_duty)
                pwm2.duty_u16(0)

        fill_light = None
        try:
            fill_light = PWM("LED_LIGHT", freq=100000, duty_u16=fill_light_duty)
        except Exception as error:
            print(error)

        if image_streamer is not None and image_streamer.frame_due():
            try:
                if not image_streamer.send_image(img):
                    raise RuntimeError("incomplete WiFi image frame")
            except Exception as error:
                print("Image stream stopped:", error)
                image_streamer = None

        frame_error_count = 0
        debug_frame_count += 1
        if debug_frame_count >= 30:
            print("UART frame:", line_error, distance_raw,
                  cross_flag, "fps:", clock.fps())
            debug_frame_count = 0

    except Exception as error:
        frame_error_count += 1
        if frame_error_count == 1 or frame_error_count % 20 == 0:
            print("Frame processing error:", error)

        # A valid fallback frame keeps the MCU/OLED link alive and makes a
        # processing failure distinguishable from a UART wiring failure.
        send_tracking_packet(0, 120, 1)
        time.sleep_ms(50)
