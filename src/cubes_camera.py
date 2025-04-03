import sensor, time
from pyb import UART, LED

# -------- Camera & Sensor Setup --------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)       # Use QVGA (as in your first code)
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time=500)

# Disable auto settings for stable color tracking
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=11000)
clock = time.clock()

# -------- UART & LED Setup --------
uart = UART(3, 19200)  # your UART settings

# Setup LEDs
red_led   = LED(1)
green_led = LED(2)
blue_led  = LED(3)

# Use a FINAL flag (True for final challenge mode, False for qualification)
FINAL = True
if FINAL:
    # Blink cyan (green+blue) to indicate camera ready (as in your second code)
    green_led.on()
    blue_led.on()
    time.sleep(0.5)
else:
    # Blink yellow (green+red) to indicate qualification mode
    green_led.on()
    red_led.on()
    time.sleep(0.5)
red_led.off()
green_led.off()
blue_led.off()
time.sleep(0.5)

# -------- Color Thresholds (LAB Space) --------
red_threshold    = [(19, 80, 6, 75, 80, 29)]
green_threshold  = [(20, 85, -60, -16, -2, 54)]
blue_threshold   = [(10, 80, -5, 25, -50, -14)]
orange_threshold = [(20, 80, 10, 45, 100, -20)]
pink_threshold   = [(25, 70, 14, 45, -15, 10)]
black_threshold  = [(0, 55, -10, 10, -10, 10)]

# -------- Define Regions of Interest (ROI) --------
image_height = sensor.height()
image_width  = sensor.width()
cubes_roi = (10, int(image_height * 0.6), image_width - 20, int(image_height * 0.6))
lines_roi = (5, int(image_height * 0.6), image_width - 10, int(image_height * 0.6))
# (Optionally, you could add a separate ROI for wall/parking if needed)

# -------- Blob Filtering Parameters --------
min_cube_size       = 30     # minimum pixel count for cubes
min_line_size       = 80     # minimum pixel count for lines
min_area            = 10     # ignore blobs with area < 10
min_valid_cube_area = 300    # only consider cubes with area >= 300
pink_wall_min_area  = 5000   # threshold for pink blob highlighting
black_wall_min_area = 5000   # threshold for black blob highlighting
min_black_height    = 10     # minimum height for a valid black blob

# -------- PD Parameters for Cube Following --------
kp_cube = 0.3     # proportional gain
kd_cube = 3.0     # derivative gain
last_err = 0
follow_threshold = 2100    # if cube area is below this, follow with PD control
STEERING_CENTER = 79       # servo center value
STEERING_MIN = 35          # minimum servo value (max right)
STEERING_MAX = 115         # maximum servo value (max left)

direction = 0  # turn direction: 0 = not set, 1 = left, 2 = right

# -------- Helper Functions --------
def get_largest_blob(blobs):
    """Return the largest blob in a list or None if empty."""
    return max(blobs, key=lambda b: b.area(), default=None)

def is_elongated(blob):
    """Return True if the blob is at least 5 times wider than it is tall."""
    return blob.w() >= 5 * blob.h()

def is_invalid_orange(orange_blob, red_blobs):
    """Check if an orange blob overlaps with a red blob in a way that should be ignored."""
    if orange_blob.h() > orange_blob.w():
        return "ignore_orange"  # ignore if taller than wide
    for red_blob in red_blobs:
        if (red_blob.x() < orange_blob.x() + orange_blob.w() and
            red_blob.x() + red_blob.w() > orange_blob.x() and
            red_blob.y() < orange_blob.y() + orange_blob.h() and
            red_blob.y() + red_blob.h() > orange_blob.y()):
            red_area = red_blob.area()
            orange_area = orange_blob.area()
            if red_area >= orange_area * 1.5:
                return "ignore_orange"
            elif orange_area >= red_area * 0.1:
                return "ignore_red"
            else:
                return None
    return None

def is_part_of_orange_line(red_blob, orange_blob):
    """
    Returns True if the red blob overlaps significantly with the orange blob,
    suggesting it is part of an orange line rather than a true red cube.
    """
    if orange_blob is None:
        return False
    # Calculate intersection of the bounding boxes
    x1 = max(red_blob.x(), orange_blob.x())
    y1 = max(red_blob.y(), orange_blob.y())
    x2 = min(red_blob.x() + red_blob.w(), orange_blob.x() + orange_blob.w())
    y2 = min(red_blob.y() + red_blob.h(), orange_blob.y() + orange_blob.h())
    if x2 > x1 and y2 > y1:
        intersection_area = (x2 - x1) * (y2 - y1)
        # If more than 50% of the red blob's area is overlapping, consider it part of the orange line.
        if intersection_area / red_blob.area() > 0.5:
            return True
    return False

# -------- Main Loop --------
while True:
    clock.tick()
    img = sensor.snapshot()
    target_x = image_width // 2  # target center for cube tracking

    # ---- Detect Blobs ----
    red_blobs    = img.find_blobs(red_threshold,    roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)
    green_blobs  = img.find_blobs(green_threshold,  roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)
    blue_blobs   = img.find_blobs(blue_threshold,   roi=lines_roi, pixels_threshold=min_line_size, area_threshold=min_line_size, merge=True)
    orange_blobs = img.find_blobs(orange_threshold, roi=lines_roi, pixels_threshold=min_line_size, area_threshold=min_line_size, merge=True)
    pink_blobs   = img.find_blobs(pink_threshold,   roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)
    black_blobs  = img.find_blobs(black_threshold,  roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)

    # ---- Get Largest Blobs ----
    red_cube    = get_largest_blob([b for b in red_blobs if b.area() >= min_valid_cube_area])
    green_cube  = get_largest_blob([b for b in green_blobs if b.area() >= min_valid_cube_area])
    blue_line   = get_largest_blob([b for b in blue_blobs if b.area() >= min_area])
    orange_line = get_largest_blob([b for b in orange_blobs if b.area() >= min_area])
    pink_blob   = get_largest_blob(pink_blobs)
    black_blob  = get_largest_blob(black_blobs)

    # ---- Highlight Large Pink Blob ----
    if pink_blob and pink_blob.area() >= pink_wall_min_area:
        img.draw_rectangle(pink_blob.rect(), color=(255, 20, 147))
        img.draw_string(pink_blob.x(), pink_blob.y() + pink_blob.h() - 10, str(pink_blob.area()), color=(255, 20, 147))
        uart.write("PINK")

    # ---- Highlight Large Black Blob (for turning/wall detection) ----
    if black_blob and black_blob.h() >= min_black_height:
        black_bottom = black_blob.y() + black_blob.h()
        black_center_x = black_blob.cx()
        lower_threshold = image_height * 0.6
        left_bound = image_width * 0.33
        right_bound = image_width * 0.66
        if black_bottom >= lower_threshold and left_bound < black_center_x < right_bound:
            img.draw_rectangle(black_blob.rect(), color=(10, 10, 10))
            img.draw_string(black_blob.x(), black_blob.y() + black_blob.h() - 10, "TURN", color=(255, 255, 255))
            uart.write("BLACK\n")

    # ---- Process Red Cube with PD Control ----
    # Skip processing if the red blob is likely part of the orange line.
    if red_cube and not is_elongated(red_cube) and not (orange_line and is_part_of_orange_line(red_cube, orange_line)):
        img.draw_rectangle(red_cube.rect(), color=(255, 0, 0))
        img.draw_cross(red_cube.cx(), red_cube.cy(), color=(255, 0, 0))
        img.draw_string(red_cube.x(), red_cube.y() + red_cube.h() - 10, str(red_cube.area()), color=(255, 0, 0))
        error = red_cube.cx() - target_x
        derivative = error - last_err
        correction = kp_cube * error + kd_cube * derivative
        last_err = error
        newSteering = STEERING_CENTER - correction
        newSteering = max(min(newSteering, STEERING_MAX), STEERING_MIN)
        if red_cube.area() < follow_threshold:
            uart.write("S%d\n" % int(newSteering))
            uart.write("F\n")
        else:
            uart.write("RED")

    # ---- Process Green Cube with PD Control ----
    if green_cube and not is_elongated(green_cube):
        img.draw_rectangle(green_cube.rect(), color=(0, 255, 0))
        img.draw_cross(green_cube.cx(), green_cube.cy(), color=(0, 255, 0))
        img.draw_string(green_cube.x(), green_cube.y() + green_cube.h() - 10, str(green_cube.area()), color=(0, 255, 0))
        error = green_cube.cx() - target_x
        derivative = error - last_err
        correction = kp_cube * error + kd_cube * derivative
        last_err = error
        newSteering = STEERING_CENTER - correction
        newSteering = max(min(newSteering, STEERING_MAX), STEERING_MIN)
        if green_cube.area() < follow_threshold:
            uart.write("S%d\n" % int(newSteering))
            uart.write("F\n")
        else:
            uart.write("GREEN")

    # ---- Process Blue Line ----
    if blue_line:
        img.draw_rectangle(blue_line.rect(), color=(0, 0, 255))
        img.draw_string(blue_line.x(), blue_line.y() + blue_line.h() - 10, str(blue_line.area()), color=(0, 0, 255))
        uart.write("BLUE")

    # ---- Process Orange Line ----
    if orange_line and not is_invalid_orange(orange_line, red_blobs):
        img.draw_rectangle(orange_line.rect(), color=(255, 165, 0))
        img.draw_string(orange_line.x(), orange_line.y() + orange_line.h() - 10, str(orange_line.area()), color=(255, 165, 0))
        uart.write("ORANGE")

    # ---- Determine and Send Turn Direction ----
    if direction == 0:
        if orange_line and not is_invalid_orange(orange_line, red_blobs):
            direction = 2  # turn right if orange line detected first
        elif blue_line:
            direction = 1  # turn left if blue line detected first
    uart.write(str(direction) + '\n')
