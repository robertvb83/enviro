import time
import json
from breakout_bme280 import BreakoutBME280
from breakout_ltr559 import BreakoutLTR559
from machine import Pin, PWM
from enviro import i2c
from phew import logging
import enviro.helpers as helpers  # Import helpers functions for calculations

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

def water():
    # Load initial state
    state = load_state('state.json')
    from enviro import config

    targets = [
        config.moisture_target_a,
        config.moisture_target_b,
        config.moisture_target_c
    ]

    # Get current moisture levels
    moisture_levels = moisture_readings()  

    for i in range(3):
        current_moisture = moisture_levels[i]

        # Check if moisture is below the dry threshold of 20%
        if current_moisture < DRY_MOISTURE_THRESHOLD:
            current_time = time.time()  # Get the current time
            if state["dry_start_time"] is None:
                state["dry_start_time"] = current_time
                save_state(state, 'state.json')
                logging.info(f"> Sensor {CHANNEL_NAMES[i]} below 20%, starting 24-hour dry phase.")
                continue  # Skip to the next sensor if still in dry phase
            else:
                if current_time - state["dry_start_time"] < DRY_PHASE_DURATION:
                    logging.info(f"> Sensor {CHANNEL_NAMES[i]} still in dry phase. No watering allowed.")
                    continue  # Skip to the next sensor if still in dry phase
                else:
                    state["dry_start_time"] = None  # Reset dry phase timer after 24 hours

        # Normal watering logic after dry phase
        if current_moisture < WET_MOISTURE_TARGET:
            logging.info(f"> Sensor {CHANNEL_NAMES[i]} below target (currently at {current_moisture}%). Starting watering...")

            # Start the pump
            pump_pins[i].value(1)

            # Set the start time for watering
            start_time = time.time()  # Get the current time

            # Watering loop for a minimum of 5 minutes or until >20% moisture is reached
            while (time.time() - start_time) < MAX_WATERING_TIME or current_moisture <= 20:
                # Get updated moisture level during watering
                moisture_levels = moisture_readings()  # Call moisture readings again
                current_moisture = moisture_levels[i]  # Update current moisture for the specific sensor
                logging.info(f"  - Current moisture level: {current_moisture}%")

                # Stop early if moisture level reaches or exceeds the wet target
                if current_moisture >= WET_MOISTURE_TARGET:
                    logging.info(f"  - Moisture target reached. Stopping pump {CHANNEL_NAMES[i]}.")
                    break

                # Sleep for a short time before the next check
                time.sleep(1)

            # Stop the pump after watering
            pump_pins[i].value(0)
            logging.info(f"  - Stopped watering after {int(time.time() - start_time)} seconds.")

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
