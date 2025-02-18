import sensor, time
from pyb import UART, LED

# **Initialize Camera**
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time=500)

# **Disable auto settings for stable color tracking**
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=10000)  # Adjusted exposure for better visibility
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

# **Color Thresholds (LAB Space)**
red_threshold = [(19, 80, 6, 75, 80, 29)]
green_threshold = [(20, 85, -60, -16, -2, 54)]
blue_threshold = [(10, 80, -5, 25, -50, -14)]
orange_threshold = [(20, 80, 10, 45, 100, -20)]
pink_threshold =  [(25, 70, 14, 45, -15, 10)]
black_threshold = [(0, 20, 35, -50, 58, -62)]

# **Define Regions of Interest (ROI)**
image_height = sensor.height()
cubes_roi = (10, int(image_height * 0.6), sensor.width() - 20, int(image_height * 0.5))  # Bottom 50%
lines_roi = (5, int(image_height * 0.6), sensor.width() - 10, int(image_height * 0.5))  # Bottom 50%

# **Blob Filtering Parameters**
min_cube_size = 30  # Lowered to capture full objects
min_line_size = 80  # Lowered to detect thinner lines
min_area = 10       # Ignore objects with area < 10
min_valid_cube_area = 800  # NEW: Ignore red/green blobs smaller than 800
pink_wall_min_area = 5000  # Threshold for pink highlighting
black_wall_min_area = 15000  # Threshold for black highlighting

def get_largest_blob(blobs):
    """Returns the largest blob in a list or None if empty."""
    return max(blobs, key=lambda b: b.area(), default=None)

def is_elongated(blob):
    """Check if the blob is at least 5 times taller than its width."""
    return blob.w() >= 5 * blob.h()

def is_invalid_orange(orange_blob, red_blobs):
    if orange_blob.h() > orange_blob.w():
        return "ignore_orange"  # Ignore if taller than wide

    for red_blob in red_blobs:
        if (
            red_blob.x() < orange_blob.x() + orange_blob.w() and
            red_blob.x() + red_blob.w() > orange_blob.x() and
            red_blob.y() < orange_blob.y() + orange_blob.h() and
            red_blob.y() + red_blob.h() > orange_blob.y()
        ):
            red_area = red_blob.area()
            orange_area = orange_blob.area()

            if red_area >= orange_area * 1.5:
                return "ignore_orange"
            elif orange_area >= red_area * 0.1:
                return "ignore_red"
            else:
                return None

    return None

direction = 0

while True:
    img = sensor.snapshot()

    # **Detect Blobs**
    red_blobs = img.find_blobs(red_threshold, roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)
    green_blobs = img.find_blobs(green_threshold, roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)
    blue_blobs = img.find_blobs(blue_threshold, roi=lines_roi, pixels_threshold=min_line_size, area_threshold=min_line_size, merge=True)
    orange_blobs = img.find_blobs(orange_threshold, roi=lines_roi, pixels_threshold=min_line_size, area_threshold=min_line_size, merge=True)
    pink_blobs = img.find_blobs(pink_threshold, roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)
    black_blobs = img.find_blobs(black_threshold, roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)

    # **Find Largest Objects**
    red_cube = get_largest_blob([b for b in red_blobs if b.area() >= min_valid_cube_area])
    green_cube = get_largest_blob([b for b in green_blobs if b.area() >= min_valid_cube_area])
    blue_line = get_largest_blob([b for b in blue_blobs if b.area() >= min_area])
    orange_line = get_largest_blob([b for b in orange_blobs if b.area() >= min_area])
    pink_blob = get_largest_blob(pink_blobs)
    black_blob = get_largest_blob(black_blobs)

    # **Highlight Large Pink Blob**
    if pink_blob and pink_blob.area() >= pink_wall_min_area:
        img.draw_rectangle(pink_blob.rect(), color=(255, 20, 147))
        img.draw_string(pink_blob.x(), pink_blob.y() + pink_blob.h() - 10, str(pink_blob.area()), color=(255, 20, 147))
        uart.write("Pink Wall Detected at X:{} Y:{} Area:{}\n".format(pink_blob.cx(), pink_blob.cy(), pink_blob.area()))

    # **Highlight Large Black Blob**
    if black_blob and black_blob.area() >= black_wall_min_area:
        img.draw_rectangle(black_blob.rect(), color=(10, 10, 10))
        img.draw_string(black_blob.x(), black_blob.y() + black_blob.h() - 10, str(black_blob.area()), color=(255, 255, 255))
        uart.write("Black Wall Detected at X:{} Y:{} Area:{}\n".format(black_blob.cx(), black_blob.cy(), black_blob.area()))

    # **Process Red Cube (Ignore if elongated or too small)**
    if red_cube and not is_elongated(red_cube):
        img.draw_rectangle(red_cube.rect(), color=(255, 0, 0))
        img.draw_cross(red_cube.cx(), red_cube.cy(), color=(255, 0, 0))
        img.draw_string(red_cube.x(), red_cube.y() + red_cube.h() - 10, str(red_cube.area()), color=(255, 0, 0))
        uart.write("RED Cube Detected at X:{} Y:{} Area:{}\n".format(red_cube.cx(), red_cube.cy(), red_cube.area()))

    # **Process Green Cube (Ignore if elongated or too small)**
    if green_cube and not is_elongated(green_cube):
        img.draw_rectangle(green_cube.rect(), color=(0, 255, 0))
        img.draw_cross(green_cube.cx(), green_cube.cy(), color=(0, 255, 0))
        img.draw_string(green_cube.x(), green_cube.y() + green_cube.h() - 10, str(green_cube.area()), color=(0, 255, 0))
        uart.write("GREEN Cube Detected at X:{} Y:{} Area:{}\n".format(green_cube.cx(), green_cube.cy(), green_cube.area()))

    # **Process Blue Line (Ignore if too small)**
    if blue_line:
        img.draw_rectangle(blue_line.rect(), color=(0, 0, 255))
        img.draw_string(blue_line.x(), blue_line.y() + blue_line.h() - 10, str(blue_line.area()), color=(0, 0, 255))
        uart.write("BLUE Line Detected at X:{} Width:{} Area:{}\n".format(blue_line.cx(), blue_line.w(), blue_line.area()))

    # **Process Orange Line (Ignore if invalid or too small)**
    if orange_line and not is_invalid_orange(orange_line, red_blobs):
        img.draw_rectangle(orange_line.rect(), color=(255, 165, 0))  # Orange color
        img.draw_string(orange_line.x(), orange_line.y() + orange_line.h() - 10, str(orange_line.area()), color=(255, 165, 0))
        uart.write("ORANGE Line Detected at X:{} Width:{} Area:{}\n".format(orange_line.cx(), orange_line.w(), orange_line.area()))

    # **Determine Direction**
    if direction == 0:
        if orange_line and not is_invalid_orange(orange_line, red_blobs):
            direction = 2  # Orange line first → turn right
        elif blue_line:
            direction = 1  # Blue line first → turn left

    # **Send Direction Command**
    uart.write(str(direction) + '\n')
