from machine import Pin, ADC, PWM
import time

# --- Joystick inputs (Pico ADC-capable pins only: GPIO26, 27, 28) ---
joy_x = ADC(26)  # azimuth axis
joy_y = ADC(27)  # elevation axis

# --- Azimuth motor ---
az_step = Pin(2, Pin.OUT)
az_dir  = Pin(3, Pin.OUT)

# --- Elevation motor ---
el_step = Pin(4, Pin.OUT)
el_dir  = Pin(5, Pin.OUT)

# --- Laser PWM: on immediately at boot, stays on ---
laser = PWM(Pin(15))
laser.freq(40000)        # 40kHz
laser.duty_u16(32768)    # adjust duty cycle to match your driver's spec

# --- Joystick tuning ---
CENTER = 32768        # Pico ADC is 16-bit (read_u16 returns 0-65535)
THRESHOLD_ON  = 24000   # offset must exceed this to START moving
THRESHOLD_OFF = 20000   # offset must drop below this to STOP moving (hysteresis gap)
RUN_FREQ = 150          # fixed step rate - slowed way down from before

NUM_SAMPLES = 8         # averaging window, smooths out noisy ADC reads

# --- Position tracking (in steps), zeroed at boot ---
# ASSUMES: azimuth manually placed at its MINIMUM (0 deg) position before power-on
# ASSUMES: elevation manually placed at horizon (0 deg) before power-on
az_pos = 0
el_pos = 0

# --- Soft limits (in steps), based on FULL STEP microstepping ---
STEPS_PER_DEG_AZ = 2.78
STEPS_PER_DEG_EL = 1.39

AZ_MIN = 0
AZ_MAX = int(180 * STEPS_PER_DEG_AZ)   # full 180° sweep, 0 to 180
EL_MIN = 0
EL_MAX = int(80 * STEPS_PER_DEG_EL)    # horizon to near-zenith

# --- Latched movement state per axis (for hysteresis) ---
az_moving = False
el_moving = False

def read_avg(adc, n=NUM_SAMPLES):
    total = 0
    for _ in range(n):
        total += adc.read_u16()
    return total // n

def joystick_to_freq_dir(raw, currently_moving):
    offset = raw - CENTER
    magnitude = abs(offset)
    direction = 1 if offset > 0 else 0

    if currently_moving:
        # already moving -- only stop once it drops below the lower threshold
        if magnitude < THRESHOLD_OFF:
            return 0, direction, False
        return RUN_FREQ, direction, True
    else:
        # not moving -- only start once it clears the higher threshold
        if magnitude >= THRESHOLD_ON:
            return RUN_FREQ, direction, True
        return 0, direction, False

def step_pulse(step_pin, high_us=3):
    step_pin.value(1)
    time.sleep_us(high_us)
    step_pin.value(0)

last_az_time = time.ticks_us()
last_el_time = time.ticks_us()

while True:
    now = time.ticks_us()

    x_raw = read_avg(joy_x)
    y_raw = read_avg(joy_y)

    az_freq, az_dir_val, az_moving = joystick_to_freq_dir(x_raw, az_moving)
    el_freq, el_dir_val, el_moving = joystick_to_freq_dir(y_raw, el_moving)

    # Enforce soft limits
    if az_dir_val == 1 and az_pos >= AZ_MAX:
        az_freq = 0
    if az_dir_val == 0 and az_pos <= AZ_MIN:
        az_freq = 0
    if el_dir_val == 1 and el_pos >= EL_MAX:
        el_freq = 0
    if el_dir_val == 0 and el_pos <= EL_MIN:
        el_freq = 0

    az_dir.value(az_dir_val)
    el_dir.value(el_dir_val)

    if az_freq > 0:
        interval_us = int(1_000_000 / az_freq)
        if time.ticks_diff(now, last_az_time) >= interval_us:
            step_pulse(az_step)
            az_pos += 1 if az_dir_val == 1 else -1
            last_az_time = now

    if el_freq > 0:
        interval_us = int(1_000_000 / el_freq)
        if time.ticks_diff(now, last_el_time) >= interval_us:
            step_pulse(el_step)
            el_pos += 1 if el_dir_val == 1 else -1
            last_el_time = now

    time.sleep_us(50)  # tiny yield, keeps loop from pegging CPU pointlessly