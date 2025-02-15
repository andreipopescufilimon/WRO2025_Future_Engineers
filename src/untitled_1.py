import sensor, time
from pyb import UART, LED

# **Initialize Camera**
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time=2000)

# **Disable auto settings for stable color tracking**
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=10000)
clock = time.clock()

# **Setup UART Communication**
uart = UART(3, 19200)

# **Setup LEDs**
red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)

# **Indicate Camera Readiness**
green_led.on()
red_led.on()  # Blink yellow to indicate qualification mode
time.sleep(0.5)
red_led.off()
green_led.off()
blue_led.off()
time.sleep(0.5)

# **Updated Color Thresholds (LAB Space)**
red_threshold = [(18, 80, 5, 60, -10, 65)]
green_threshold = [(20, 85, -60, -5, -5, 55)]
blue_threshold = [(10, 80, -5, 25, -50, -5)]
orange_threshold = [(50, 85, 10, 50, 25, 80)]  # **More distinct from red**

# **Define Regions of Interest (ROI)**
cubes_roi = (10, int(sensor.height() / 2), sensor.width() - 20, int(sensor.height() / 2 - 10))
lines_roi = (5, int(sensor.height() - 60), sensor.width() - 10, 50)

# **Blob Filtering Parameters**
min_cube_size = 50
min_line_size = 300

def get_largest_blob(blobs):
    """Returns the largest blob in a list or None if empty."""
    return max(blobs, key=lambda b: b.area(), default=None)

direction = 0

while True:
    img = sensor.snapshot()

    # **Detect Colored Cubes**
    red_blobs = img.find_blobs(red_threshold, roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)
    green_blobs = img.find_blobs(green_threshold, roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)

    # **Find Largest Red & Green Cube**
    red_cube = get_largest_blob(red_blobs)
    green_cube = get_largest_blob(green_blobs)

    # **Detect Colored Lines**
    blue_blobs = img.find_blobs(blue_threshold, roi=lines_roi, pixels_threshold=min_line_size, area_threshold=min_line_size, merge=True)
    orange_blobs = img.find_blobs(orange_threshold, roi=lines_roi, pixels_threshold=min_line_size, area_threshold=min_line_size, merge=True)

    # **Find Widest Blue & Orange Line**
    blue_line = get_largest_blob(blue_blobs)
    orange_line = get_largest_blob(orange_blobs)

    # **Process Red Cube**
    if red_cube:
        img.draw_rectangle(red_cube.rect(), color=(255, 0, 0))
        img.draw_cross(red_cube.cx(), red_cube.cy(), color=(255, 0, 0))
        uart.write("RED Cube Detected at X:{} Y:{}\n".format(red_cube.cx(), red_cube.cy()))

    # **Process Green Cube**
    if green_cube:
        img.draw_rectangle(green_cube.rect(), color=(0, 255, 0))
        img.draw_cross(green_cube.cx(), green_cube.cy(), color=(0, 255, 0))
        uart.write("GREEN Cube Detected at X:{} Y:{}\n".format(green_cube.cx(), green_cube.cy()))

    # **Process Blue Line**
    if blue_line:
        img.draw_rectangle(blue_line.rect(), color=(0, 0, 255))
        uart.write("BLUE Line Detected at X:{} Width:{}\n".format(blue_line.cx(), blue_line.w()))

    # **Process Orange Line**
    if orange_line:
        img.draw_rectangle(orange_line.rect(), color=(255, 165, 0))
        uart.write("ORANGE Line Detected at X:{} Width:{}\n".format(orange_line.cx(), orange_line.w()))

    # **Determine Direction Based on First Detected Line**
    if direction == 0:
        if orange_line:
            direction = 2  # Orange line first → turn right
        elif blue_line:
            direction = 1  # Blue line first → turn left

    # **Send Direction Command to Arduino**
    has_line = orange_line or blue_line
    uart.write(str(direction) + '\n')
