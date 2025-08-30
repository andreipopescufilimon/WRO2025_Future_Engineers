import sensor, time
from pyb import UART, LED

# -------- DEBUG FLAG --------
DEBUG = False

# -------- Camera & Sensor Setup --------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)       # QVGA: 320x240
sensor.set_vflip(True)
sensor.set_hmirror(True)

# Disable auto settings for stable color tracking
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=11000) # 9000 open,  10000 rest

sensor.skip_frames(time=2000)

clock = time.clock()

# -------- UART & LED Setup --------
uart = UART(3, 19200)
red_led   = LED(1)
green_led = LED(2)
blue_led  = LED(3)

# Blink yellow (green+red) to indicate qualification mode
green_led.on(); red_led.on()
time.sleep(0.5)
red_led.off(); green_led.off(); blue_led.off()
time.sleep(0.5)

# -------- Color Thresholds (LAB Space) --------
red_threshold    = [(32, 54, 40, 67, 17, 63)]
green_threshold  = [(36, 71, -64, -16, -12, 46), (28, 47, -38, -17, -7, 19)]
blue_threshold   = [(9, 76, -45, 27, -57, -8)]
orange_threshold = [(62, 91, -3, 43, 5, 69)]
pink_threshold   = [(30, 70, 10, 60, -15, 15)]
black_threshold  = [(0, 37, -26, 7, -17, 11)]

# -------- Define Regions of Interest (ROI) --------
img_h = sensor.height()
img_w = sensor.width()
cubes_roi = (0, int(img_h * 0.4), img_w, int(img_h * 0.6))
lines_roi = (5, int(img_h * 0.5), img_w - 10, int(img_h * 0.4))
wall_roi  = (50, int(img_h * 0.5 - 18), img_w - 100, int(img_h * 0.2 - 10))
final_wall_roi  = (30, int(80), img_w - 60, int(img_h - 60))

# -------- Blob Filtering Parameters --------
min_cube_size       = 500
min_line_size       = 800
min_area            = 10
min_valid_cube_area = 450
pink_wall_min_area  = 5000
black_wall_min_area = 7000
final_black_wall_min_area = 9000
min_black_height    = 39
min_black_width     = 0

# -------- PD Parameters for Cube Following --------
kp_cube = 0.3
kd_cube = 2.4
pid_error = 0.0
pid_last_error = 0.0
follow_threshold = 3900

direction = 0  # turn direction: 0 = not set, 1 = left, 2 = right

# -------- Helper Functions --------
def get_largest_blob(blobs):
    return max(blobs, key=lambda b: b.area(), default=None)

def is_elongated(blob):
    return blob.w() >= 5 * blob.h()

def is_invalid_orange(orange_blob, red_blobs):
    # ignore tall thin orange, or overlap logic with reds
    if orange_blob.h() > orange_blob.w():
        return "ignore_orange"
    for r in red_blobs:
        if (r.x() < orange_blob.x()+orange_blob.w() and
            r.x()+r.w() > orange_blob.x() and
            r.y() < orange_blob.y()+orange_blob.h() and
            r.y()+r.h() > orange_blob.y()):
            ra, oa = r.area(), orange_blob.area()
            if ra >= oa * 1.5:
                return "ignore_orange"
            elif oa >= ra * 0.1:
                return "ignore_red"
    return None

# -------- Main Loop --------
while True:
    clock.tick()
    img = sensor.snapshot()

    # ---- Draw ROIs on the image for debugging ----
    # cubes_roi: bottom 40% of the frame
 #   img.draw_rectangle(cubes_roi, color=(255, 155, 0))  # yellow
    # lines_roi: slightly inset bottom 40%
#    img.draw_rectangle(lines_roi, color=(0, 255, 255))  # cyan
    # wall_roi: middle 60% of the frame
    #img.draw_rectangle(final_wall_roi,  color=(255,   0, 255))  # magenta


    target_x = img_w // 2

    # ---- Detect Blobs ----
    red_blobs    = img.find_blobs(red_threshold,    roi=cubes_roi,
                                  pixels_threshold=min_cube_size,
                                  area_threshold=min_cube_size, merge=True)
    green_blobs  = img.find_blobs(green_threshold,  roi=cubes_roi,
                                  pixels_threshold=min_cube_size,
                                  area_threshold=min_cube_size, merge=True)
    blue_blobs   = img.find_blobs(blue_threshold,   roi=lines_roi,
                                  pixels_threshold=min_line_size,
                                  area_threshold=min_line_size, merge=True)
    orange_blobs = img.find_blobs(orange_threshold, roi=lines_roi,
                                  pixels_threshold=min_line_size,
                                  area_threshold=min_line_size, merge=True)
#    pink_blobs   = img.find_blobs(pink_threshold,   roi=cubes_roi,
#                                  pixels_threshold=min_cube_size,
#                                  area_threshold=min_cube_size, merge=True)
    black_blobs  = img.find_blobs(black_threshold,  roi=wall_roi,
                                  pixels_threshold=black_wall_min_area,
                                  area_threshold=black_wall_min_area, merge=True)

    black_blobs  = img.find_blobs(black_threshold,  roi=final_wall_roi,
                                pixels_threshold=final_black_wall_min_area,
                                area_threshold=final_black_wall_min_area, merge=True)

    # ---- Get Largest Blobs ----
    red_cube    = get_largest_blob([b for b in red_blobs   if b.area() >= min_valid_cube_area])
    green_cube  = get_largest_blob([b for b in green_blobs if b.area() >= min_valid_cube_area])
    blue_line   = get_largest_blob([b for b in blue_blobs  if b.area() >= min_area])
    orange_line = get_largest_blob([b for b in orange_blobs if b.area() >= min_area])
#    pink_blob   = get_largest_blob(pink_blobs)
    black_blob  = get_largest_blob(black_blobs)

    # ---- Highlight Large Pink Blob ----
#    if pink_blob and pink_blob.area() >= pink_wall_min_area:
#        img.draw_rectangle(pink_blob.rect(), color=(255,20,147))
#        img.draw_string(pink_blob.x(), pink_blob.y()+pink_blob.h()-10,
#                        str(pink_blob.area()), color=(255,20,147))
#        uart.write("PINK\n")

    # ---- Highlight/Detect Black Wall for Turns ----
    if black_blob:
        blob_h = black_blob.h()
        blob_area = black_blob.area()  # or black_blob.w() * black_blob.h()
        if blob_h >= min_black_height and blob_area >= black_wall_min_area:
            img.draw_rectangle(black_blob.rect(), color=(200,200,10))
            img.draw_string(black_blob.x(), black_blob.y() + blob_h - 10,
                            "TURN", color=(255,255,255))
            if DEBUG:
                print("Black wall: h={} area={}".format(blob_h, blob_area))
            uart.write("BLACK\n")


    # ---- Choose Closest Cube (largest red or green) ----
    candidates = []
    if red_cube:   candidates.append(('R', red_cube))
    if green_cube: candidates.append(('G', green_cube))

    if candidates:
        color_char, cube = max(candidates, key=lambda x: x[1].area())
        area = cube.area()
        col  = (255,0,0) if color_char=='R' else (0,255,0)

        # draw
        img.draw_rectangle(cube.rect(), color=col)
        img.draw_cross(cube.cx(), cube.cy(), color=col)
        img.draw_string(cube.x(), cube.y()+cube.h()-10, str(area), color=col)

        # PD on normalized error
        error = (cube.cx() - target_x) / float(target_x)
        pid_error = kp_cube * error + kd_cube * (error - pid_last_error)
        pid_last_error = error

        if area < follow_threshold:
            uart.write("S{:+.3f}\n".format(pid_error))
            if DEBUG:
                print("{} FOLLOW → err:{:+.3f}, pid:{:+.3f}, area:{}".format(
                      "RED" if color_char=='R' else "GREEN",
                      error, pid_error, area))
        else:
            # at close range, just report color
            uart.write(("RED\n" if color_char=='R' else "GREEN\n"))
            if DEBUG:
                print("{} CLOSE → area {} >= {}".format(
                      "RED" if color_char=='R' else "GREEN",
                      area, follow_threshold))

    # ---- Process Line Following (unchanged) ----
    valid_orange = None
    if orange_line and not is_invalid_orange(orange_line, red_blobs):
        valid_orange = orange_line

    chosen_line, chosen_color = None, None
    if blue_line and valid_orange:
        if blue_line.area() > valid_orange.area():
            chosen_line, chosen_color = blue_line, "BLUE"
        else:
            chosen_line, chosen_color = valid_orange, "ORANGE"
    elif blue_line:
        chosen_line, chosen_color = blue_line, "BLUE"
    elif valid_orange:
        chosen_line, chosen_color = valid_orange, "ORANGE"

    if chosen_line:
        c = (0,0,255) if chosen_color=="BLUE" else (255,165,0)
        img.draw_rectangle(chosen_line.rect(), color=c)
        img.draw_string(chosen_line.x(), chosen_line.y()+chosen_line.h()-10,
                        str(chosen_line.area()), color=c)
        uart.write(chosen_color + "\n")
        if direction == 0:
            direction = 1 if chosen_color=="BLUE" else 2

    # Always send the current turn direction
    uart.write(str(direction) + "\n")
