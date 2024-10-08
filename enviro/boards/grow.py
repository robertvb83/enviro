import time
import json
from breakout_bme280 import BreakoutBME280
from breakout_ltr559 import BreakoutLTR559
from machine import Pin, PWM
from enviro import i2c
from phew import logging
import enviro.helpers as helpers  # Import helpers functions for calculations
import os

# Get the directory of the current script
script_dir = os.path.dirname(os.path.abspath(__file__))
STATUS_FILE = os.path.join(script_dir, "status.txt")  # Path to the status file

CHANNEL_NAMES = ['A', 'B', 'C']
DRY_MOISTURE_THRESHOLD = 20  # Define your threshold for dry moisture
DRY_PHASE_DURATION = 86400  # 24 hours in seconds

bme280 = BreakoutBME280(i2c, 0x77)
ltr559 = BreakoutLTR559(i2c)

piezo_pwm = PWM(Pin(28))

moisture_sensor_pins = [
    Pin(15, Pin.IN, Pin.PULL_DOWN),
    Pin(14, Pin.IN, Pin.PULL_DOWN),
    Pin(13, Pin.IN, Pin.PULL_DOWN)
]

pump_pins = [
    Pin(12, Pin.OUT, value=0),
    Pin(11, Pin.OUT, value=0),
    Pin(10, Pin.OUT, value=0)
]

def load_state(filename):
    try:
        with open(filename, 'r') as file:
            return json.load(file)
    except (FileNotFoundError, json.JSONDecodeError):
        return {"dry_start_time": None}  # Initialize if file does not exist or is invalid

def save_state(state, filename):
    with open(filename, 'w') as file:
        json.dump(state, file)

def moisture_readings():
    results = []
    for i in range(0, 3):
        sensor = moisture_sensor_pins[i]
        last_value = sensor.value()
        start = time.ticks_ms()
        first = None
        last = None
        ticks = 0
        while ticks < 10 and time.ticks_diff(time.ticks_ms(), start) <= 1000:
            value = sensor.value()
            if last_value != value:
                if first is None:
                    first = time.ticks_ms()
                last = time.ticks_ms()
                ticks += 1
                last_value = value

        if not first or not last:
            results.append(0.0)
            continue

        average = time.ticks_diff(last, first) / ticks
        min_ms = 20
        max_ms = 80
        average = max(min_ms, min(max_ms, average))  # clamp range
        scaled = ((average - min_ms) / (max_ms - min_ms)) * 100
        results.append(round(scaled, 2))

    return results

def drip_noise():
    piezo_pwm.duty_u16(32768)
    for i in range(0, 10):
        f = i * 20
        piezo_pwm.freq((f * f) + 1000)      
        time.sleep(0.02)
    piezo_pwm.duty_u16(0)

def read_status():
    if os.path.exists(STATUS_FILE):
        with open(STATUS_FILE, "r") as file:
            status = file.read().strip()
            return status
    return None

def write_status(status):
    with open(STATUS_FILE, "w") as file:
        file.write(status)

def clear_status():
    if os.path.exists(STATUS_FILE):
        os.remove(STATUS_FILE)

def water(moisture_levels):
    from enviro import config

    min_targets = [
        config.moisture_min_target_a,
        config.moisture_min_target_b,
        config.moisture_min_target_c,
    ]
    max_targets = [
        config.moisture_max_target_a,
        config.moisture_max_target_b,
        config.moisture_max_target_c,
    ]
    max_watering_time = 120  # Maximum watering time in seconds (2 minutes)

    for i in range(3):
        status = read_status()
        continue_watering = status and status.startswith(f"unfinished_{i}")

        if continue_watering or moisture_levels[i] < min_targets[i]:
            logging.info(f"> sensor {CHANNEL_NAMES[i]} below minimum moisture target {min_targets[i]} (currently at {int(moisture_levels[i])}).")

            if config.auto_water:
                logging.info(f"  - starting pump {CHANNEL_NAMES[i]} until moisture reaches {max_targets[i]} or for a maximum of {max_watering_time} seconds")
                pump_pins[i].value(1)
                
                start_time = time.time()
                while read_moisture_levels()[i] < max_targets[i]:
                    if time.time() - start_time > max_watering_time:
                        logging.info(f"  - maximum watering time reached for pump {CHANNEL_NAMES[i]}")
                        write_status(f"unfinished_{i}")
                        break
                    time.sleep(10)  # Check every 10 seconds (adjust as needed)
                else:
                    # Only clear the status if the loop completes without breaking
                    clear_status()

                pump_pins[i].value(0)
                logging.info(f"  - stopped pump {CHANNEL_NAMES[i]}")
            else:
                logging.info(f"  - playing beep")
                for j in range(i + 1):
                    drip_noise()
                time.sleep(0.5)

def get_sensor_readings(seconds_since_last, is_usb_power):
    bme280.read()
    time.sleep(0.1)
    bme280_data = bme280.read()

    ltr_data = ltr559.get_reading()
    moisture_levels = moisture_readings()

    water(moisture_levels)  # Run pumps if needed

    temperature = bme280_data[0]
    humidity = bme280_data[2]
    pressure = bme280_data[1] / 100.0  # Convert pressure from Pa to hPa

    dew_point = helpers.calculate_dew_point(temperature, humidity)

    from ucollections import OrderedDict
    return OrderedDict({
        "temperature": round(temperature, 2),
        "humidity": round(humidity, 2),
        "pressure": round(pressure, 2),
        "luminance": round(ltr_data[BreakoutLTR559.LUX], 2),
        "moisture_a": round(moisture_levels[0], 2),
        "moisture_b": round(moisture_levels[1], 2),
        "moisture_c": round(moisture_levels[2], 2),
        "dew_point": round(dew_point, 2)
    })

def play_tone(frequency=None):
    if frequency:
        piezo_pwm.freq(frequency)
        piezo_pwm.duty_u16(32768)

def stop_tone():
    piezo_pwm.duty_u16(0)
