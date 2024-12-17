import time
import json
import math
from breakout_bme280 import BreakoutBME280
from breakout_ltr559 import BreakoutLTR559
from breakout_bme68x import BreakoutBME68X
from machine import Pin, PWM
from enviro import i2c
from phew import logging
import enviro.helpers as helpers  # Import helpers functions for calculations
from enviro import config
import os

# Get the directory of the current script
script_dir = os.path.dirname(os.path.abspath(__file__))
STATUS_FILE = os.path.join(script_dir, "status.txt")  # Path to the status file

CHANNEL_NAMES = ['A', 'B', 'C']
DRY_MOISTURE_THRESHOLD = 20  # Define your threshold for dry moisture
DRY_PHASE_DURATION = 86400  # 24 hours in seconds

# temperature and humidity correction array definitions

# For temperature offset
temperature_points = [-20, -10, 0, 20, 23.15, 23.3, 30]
temperature_offsets = [0.86, 0.86, 0.86, 0.86, 0.86, 0.9, 0.95]

temperature_points_usb = [-20, -10, 0, 20, 29.41, 29.43]
temperature_offsets_usb = [1, 1, 1.5, 2, 2.84, 2.87]

# For humidity factor
humidity_points = [-20, -10, 0, 20, 25, 30] # as Temperature
humidity_factors = [0.99, 0.99, 0.99, 0.99, 0.99, 0.99]

humidity_points_usb = [-20, -10, 0, 20, 29.41, 29.43] # as Temperature
humidity_factors_usb = [1, 1, 1, 0.975, 1.04, 1.02]

CHANNEL_NAMES = ["A", "B", "C"]

# Initialize onboard sensors
bme280 = BreakoutBME280(i2c, 0x77)
ltr559 = BreakoutLTR559(i2c)

piezo_pwm = PWM(Pin(28))

moisture_sensor_pins = [
    Pin(15, Pin.IN, Pin.PULL_DOWN),
    Pin(14, Pin.IN, Pin.PULL_DOWN),
    Pin(13, Pin.IN, Pin.PULL_DOWN),
]

pump_pins = [
    Pin(12, Pin.OUT, value=0),
    Pin(11, Pin.OUT, value=0),
    Pin(10, Pin.OUT, value=0),
]

# Initialize external BME688 sensor
bme688 = BreakoutBME68X(i2c, address=0x76)


def moisture_readings():
    results = []

    for i in range(0, 3):
        # count time for sensor to "tick" 25 times
        sensor = moisture_sensor_pins[i]

        last_value = sensor.value()
        start = time.ticks_ms()
        first = None
        last = None
        ticks = 0
        while ticks < 10 and time.ticks_diff(time.ticks_ms(), start) <= 1000:
            value = sensor.value()
            if last_value != value:
                if first == None:
                    first = time.ticks_ms()
                last = time.ticks_ms()
                ticks += 1
                last_value = value

        if not first or not last:
            results.append(0.0)
            continue

        # calculate the average tick between transitions in ms
        average = time.ticks_diff(last, first) / ticks
        # scale the result to a 0...100 range where 0 is very dry
        # and 100 is standing in water
        #
        # dry = 10ms per transition, wet = 80ms per transition
        min_ms = 20
        max_ms = 80
        average = max(min_ms, min(max_ms, average))  # clamp range
        scaled = ((average - min_ms) / (max_ms - min_ms)) * 100
        results.append(round(scaled, 2))

    return results

# make a semi convincing drip noise
def drip_noise():
    piezo_pwm.duty_u16(32768)
    for i in range(0, 10):
        f = i * 20
        piezo_pwm.freq((f * f) + 1000)      
        time.sleep(0.02)
    piezo_pwm.duty_u16(0)

# Read status of all pumps
def read_status():
    try:
        with open(STATUS_FILE, "r") as file:
            status_data = {}
            for line in file:
                line = line.strip()  # Remove leading/trailing whitespace
                if ":" in line:  # Ensure that the line contains a colon
                    try:
                        pump_id, status = line.split(":", 1)  # Split into pump_id and status
                        status_data[int(pump_id)] = status  # Store in the dictionary with pump_id as key
                    except ValueError:
                        continue  # Skip lines that can't be split properly
            return status_data
    except OSError:
        # If the file doesn't exist, create it with empty content
        with open(STATUS_FILE, "w") as file:
            file.write("")  # Initialize with empty content
        return {}  # Return an empty dictionary instead of None

# Write status for a specific pump
def write_status(pump_id, status):
    try:
        # Read current status data (assuming read_status returns a dictionary)
        status_data = read_status()

        # If the status_data is None (no file or empty), initialize an empty dictionary
        if status_data is None:
            status_data = {}

        # Update the status for the given pump_id
        status_data[pump_id] = status

        # Write updated statuses back to the file
        with open(STATUS_FILE, "w") as file:
            for pump_id, status in status_data.items():
                file.write(f"{pump_id}:{status}\n")

    except OSError:
        # If there's an OSError, initialize the status file with empty statuses 
        with open(STATUS_FILE, "w") as file:
            file.write("")  # Initialize with empty content

# Clear the status of a specific pump
def clear_status(pump_id):
    try:
        # Read current status data
        status_data = read_status()

        # If the pump exists, remove its status (set to empty)
        if pump_id in status_data:
            status_data[pump_id] = ""  # Set the pump's status to an empty string

        # Write updated statuses back to the file, ignoring empty values
        with open(STATUS_FILE, "w") as file:
            # Only write non-empty statuses
            file.write("\n".join(f"{key}:{value}" for key, value in status_data.items() if value) + "\n")
    except OSError:
        pass  # If the file doesn't exist, nothing needs to be done

def water(moisture_levels):
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
    max_watering_time = 15  # Maximum watering time in seconds (2 minutes)

    for i in range(3):
        status = read_status()
        continue_watering = status.get(i) == f"unfinished_{i}" or moisture_levels[i] < min_targets[i]

        if continue_watering:
            logging.info(f"> sensor {CHANNEL_NAMES[i]} below minimum moisture target {min_targets[i]} (currently at {int(moisture_levels[i])}).")

            if config.auto_water:
                logging.info(f"  - starting pump {CHANNEL_NAMES[i]} until moisture reaches {max_targets[i]} or for a maximum of {max_watering_time} seconds")
                pump_pins[i].value(1)
                
                start_time = time.time()
                while moisture_readings()[i] < max_targets[i]:
                    if time.time() - start_time > max_watering_time:
                        logging.info(f"  - maximum watering time reached for pump {CHANNEL_NAMES[i]}")
                        write_status(i, f"unfinished_{i}")  # Update status to indicate unfinished
                        break
                    time.sleep(0.5)  # Check every 1 seconds (adjust as needed)
                else:
                    # Only clear the status if the loop completes without breaking
                    clear_status(i)  # Clear the status of the current pump

                pump_pins[i].value(0)
                logging.info(f"  - stopped pump {CHANNEL_NAMES[i]}")
            else:
                logging.info(f"  - playing beep")
                for j in range(i + 1):
                    drip_noise()
                time.sleep(0.5)


def append_to_calibration_file(temperature, temp_offset, adjusted_humidity, humidity_factor, is_usb_power):
    # Select the appropriate filename based on the power source
    filename = "grow_calibration_data_usb.txt" if is_usb_power else "grow_calibration_data.txt"

    # Read existing data from the file
    try:
        with open(filename, "r") as f:
            lines = f.readlines()
            if lines:
                # Extracting the values safely
                temperature_points = eval(lines[0].strip().split('=')[1].strip())
                temperature_offsets = eval(lines[1].strip().split('=')[1].strip())
                humidity_points = eval(lines[2].strip().split('=')[1].strip())
                humidity_factors = eval(lines[3].strip().split('=')[1].strip())
            else:
                temperature_points = []
                temperature_offsets = []
                humidity_points = []
                humidity_factors = []
    except OSError:
        temperature_points = []
        temperature_offsets = []
        humidity_points = []
        humidity_factors = []
    except SyntaxError as e:
        print(f"Syntax error in the calibration file: {e}")
        # Reset lists in case of error
        temperature_points = []
        temperature_offsets = []
        humidity_points = []
        humidity_factors = []

    # Append the new values
    temperature_points.append(round(temperature, 2))
    if is_usb_power:
        temperature_offsets.append(round(temp_offset - config.usb_power_temperature_offset, 2))
    else:
        temperature_offsets.append(round(temp_offset, 2))
    humidity_points.append(round(temperature, 2)) # changed from adjusted_humidity
    humidity_factors.append(round(humidity_factor, 2))

    # Sort the arrays based on temperature and humidity
    temp_sorted = sorted(zip(temperature_points, temperature_offsets))
    humidity_sorted = sorted(zip(humidity_points, humidity_factors))

    # Unzip the sorted tuples back into lists
    temperature_points, temperature_offsets = zip(*temp_sorted) if temp_sorted else ([], [])
    humidity_points, humidity_factors = zip(*humidity_sorted) if humidity_sorted else ([], [])

    # Save the updated arrays back to the file
    with open(filename, "w") as f:
        f.write(f"temperature_points = {list(temperature_points)}\n")
        f.write(f"temperature_offsets = {list(temperature_offsets)}\n")
        f.write(f"humidity_points = {list(humidity_points)}\n")
        f.write(f"humidity_factors = {list(humidity_factors)}\n")


def get_sensor_readings(seconds_since_last, is_usb_power):
    # bme280 returns the register contents immediately and then starts a new reading
    # we want the current reading so do a dummy read to discard register contents first
    bme280.read()
    time.sleep(0.1)
    bme280_data = bme280.read()

    # Read from external BME688 sensor
    bme688_data = bme688.read()

    ltr_data = ltr559.get_reading()

    moisture_levels = moisture_readings()

    water(moisture_levels)  # run pumps if needed

    # Read temperature, humidity, and pressure
    temperature = bme280_data[0]
    humidity = bme280_data[2]
    pressure = bme280_data[1] / 100.0  # Convert pressure from Pa to hPa

    # Read from external BME688 sensor
    ext_temperature = bme688_data[0]
    ext_humidity = bme688_data[2]
    ext_pressure = bme688_data[1] / 100.0
    ext_gas_resistance = bme688_data[3]
    # an approximate air quality calculation that accounts for the effect of
    # humidity on the gas sensor
    # https://forums.pimoroni.com/t/bme680-observed-gas-ohms-readings/6608/25
    ext_aqi = round(math.log(ext_gas_resistance) + 0.04 * ext_humidity, 1)

    is_calibration = True
    if is_calibration:
        # calculate offset values for fitting
        calc_temp_offset = temperature - ext_temperature
        calc_adjusted_temperature = temperature - calc_temp_offset
        calc_absolute_humidity = helpers.relative_to_absolute_humidity(humidity, temperature, pressure)
        calc_adjusted_humidity = helpers.absolute_to_relative_humidity(calc_absolute_humidity, calc_adjusted_temperature, pressure)
        calc_humidity_factor = ext_humidity / calc_adjusted_humidity

        # Save the values to a text file
        append_to_calibration_file(temperature, calc_temp_offset, calc_adjusted_humidity, calc_humidity_factor, is_usb_power)
    
    if is_usb_power:
        usb_offset = helpers.interpolate(temperature, temperature_points_usb, temperature_offsets_usb) + config.usb_power_temperature_offset
        adjusted_temperature = temperature - usb_offset
    else:
        # Get sliding offset based on temperature
        non_usb_offset = helpers.interpolate(temperature, temperature_points, temperature_offsets)
        adjusted_temperature = temperature - non_usb_offset

    absolute_humidity = helpers.relative_to_absolute_humidity(humidity, temperature, pressure)
    adjusted_humidity = helpers.absolute_to_relative_humidity(absolute_humidity, adjusted_temperature, pressure)

    if is_usb_power:
        humidity_factor = helpers.interpolate(temperature, humidity_points_usb, humidity_factors_usb)
    else:
        humidity_factor = helpers.interpolate(temperature, humidity_points, humidity_factors)
    humidity = humidity_factor * adjusted_humidity  # Adjust humidity with correction factor

    temperature = adjusted_temperature
    
    # Calculate external absolute humidity using helpers
    ext_absolute_humidity = helpers.relative_to_absolute_humidity(ext_humidity, ext_temperature, ext_pressure)

    # Calculate predicted rel. humidity after venting using helpers
    calc_humidity = helpers.absolute_to_relative_humidity(ext_absolute_humidity, temperature, ext_pressure)

    # Calculate delta rel. humidity before/after venting using helpers
    delta_humidity = calc_humidity - humidity

    # Calculate dew point using helpers
    dew_point = helpers.calculate_dew_point(temperature, humidity)

    # Calculate ext_dew point using helpers
    ext_dew_point = helpers.calculate_dew_point(ext_temperature, ext_humidity)

    from ucollections import OrderedDict

    return OrderedDict(
        {
            "temperature": round(temperature, 2),
            "humidity": round(humidity, 2),
            "pressure": round(pressure, 2),
            "luminance": round(ltr_data[BreakoutLTR559.LUX], 2),
            "moisture_a": round(moisture_levels[0], 2),
            "moisture_b": round(moisture_levels[1], 2),
            "moisture_c": round(moisture_levels[2], 2),
            "dew_point": round(dew_point, 2),
            "ext_temperature": round(ext_temperature, 2),
            "ext_humidity": round(ext_humidity, 2),
            "ext_pressure": round(ext_pressure, 2),
            "ext_gas_resistance": round(ext_gas_resistance),
            "ext_aqi": ext_aqi,
            "ext_dew_point": round(ext_dew_point, 2),
            "calc_humidity": round(calc_humidity, 2),
            "delta_humidity": round(delta_humidity, 2)
        }
    )

def play_tone(frequency=None):
    if frequency:
        piezo_pwm.freq(frequency)
        piezo_pwm.duty_u16(32768)

def stop_tone():
    piezo_pwm.duty_u16(0)
