import time
import math
from breakout_bme280 import BreakoutBME280
from breakout_ltr559 import BreakoutLTR559
from breakout_bme68x import BreakoutBME68X
from machine import Pin, PWM
from enviro import i2c
from phew import logging
import enviro.helpers as helpers  # Import helpers functions for calculations
from enviro import config

# temperature and humidity correction array definitions

# For temperature offset
temperature_points = [-20, -10, 0, 20, 30]
temperature_offsets = [1, 1, 1, 1, 1]

# For humidity factor
humidity_points = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
humidity_factors = [1, 1, 1, 1, 0.975, 0.975, 0.975, 0.975, 1, 1, 1]

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


def water(moisture_levels):
    from enviro import config

    targets = [
        config.moisture_target_a,
        config.moisture_target_b,
        config.moisture_target_c,
    ]

    for i in range(0, 3):
        if moisture_levels[i] < targets[i]:
            # determine a duration to run the pump for
            duration = round((targets[i] - moisture_levels[i]) / 25, 1)

            logging.info(f"> sensor {CHANNEL_NAMES[i]} below moisture target {targets[i]} (currently at {int(moisture_levels[i])}).")

            if config.auto_water:
                logging.info(f"  - running pump {CHANNEL_NAMES[i]} for {duration} second(s)")
                pump_pins[i].value(1)
                time.sleep(duration)
                pump_pins[i].value(0)
            else:
                logging.info(f"  - playing beep")
                for j in range(0, i + 1):
                    drip_noise()
                time.sleep(0.5)


def append_to_calibration_file(temperature, temp_offset, adjusted_humidity, humidity_factor):
    # Read existing data from the file
    try:
        with open("grow_calibration_data.txt", "r") as f:
            lines = f.readlines()
            if lines:
                temperature_points = eval(lines[0].strip().split('=')[1])
                temperature_offsets = eval(lines[1].strip().split('=')[1])
                humidity_points = eval(lines[2].strip().split('=')[1])
                humidity_factors = eval(lines[3].strip().split('=')[1])
            else:
                temperature_points = []
                temperature_offsets = []
                humidity_points = []
                humidity_factors = []
    except FileNotFoundError:
        temperature_points = []
        temperature_offsets = []
        humidity_points = []
        humidity_factors = []

    # Append the new values
    temperature_points.append(temperature)
    temperature_offsets.append(temp_offset)
    humidity_points.append(adjusted_humidity)
    humidity_factors.append(humidity_factor)

    # Sort the arrays based on temperature and humidity
    temp_sorted = sorted(zip(temperature_points, temperature_offsets))
    humidity_sorted = sorted(zip(humidity_points, humidity_factors))

    temperature_points, temperature_offsets = zip(*temp_sorted)
    humidity_points, humidity_factors = zip(*humidity_sorted)

    # Save the updated arrays back to the file
    with open("grow_calibration_data.txt", "w") as f:
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
        append_to_calibration_file(temperature, calc_temp_offset, calc_adjusted_humidity, calc_humidity_factor)
    
    if is_usb_power:
        usb_offset = helpers.interpolate(temperature, temperature_points, temperature_offsets) + config.usb_power_temperature_offset
        adjusted_temperature = temperature - usb_offset
    else:
        # Get sliding offset based on temperature
        non_usb_offset = helpers.interpolate(temperature, temperature_points, temperature_offsets)
        adjusted_temperature = temperature - non_usb_offset

    absolute_humidity = helpers.relative_to_absolute_humidity(humidity, temperature, pressure)
    adjusted_humidity = helpers.absolute_to_relative_humidity(absolute_humidity, adjusted_temperature, pressure)
    temperature = adjusted_temperature

    humidity_factor = helpers.interpolate(adjusted_humidity, humidity_points, humidity_factors)
    humidity = humidity_factor * adjusted_humidity  # Adjust humidity with correction factor

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
