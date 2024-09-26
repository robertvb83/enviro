import time
import math
from breakout_bme280 import BreakoutBME280
from breakout_ltr559 import BreakoutLTR559
from breakout_bme68x import BreakoutBME68X
from machine import Pin, PWM
from enviro import i2c
from phew import logging
import enviro.helpers as helpers  # Import helpers functions for calculations

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


def get_temperature_offset(temp):
    # Define the temperature points and corresponding offsets
    temp_points = [-20, -10, 0, 20, 30]
    offset_points = [1, 1, 1, 1, 1]

    # If temperature is outside defined range, cap the offset
    if temp <= temp_points[0]:
        return offset_points[0]
    elif temp >= temp_points[-1]:
        return offset_points[-1]

    # Linear interpolation for temperatures within the defined range
    for i in range(1, len(temp_points)):
        if temp_points[i - 1] <= temp <= temp_points[i]:
            # Interpolate between temp_points[i-1] and temp_points[i]
            t1, t2 = temp_points[i - 1], temp_points[i]
            o1, o2 = offset_points[i - 1], offset_points[i]
            return o1 + (o2 - o1) * (temp - t1) / (t2 - t1)


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

    if is_usb_power:
        adjusted_temperature = temperature - config.usb_power_temperature_offset
    else:
        # Get sliding offset based on temperature
        non_usb_offset = get_temperature_offset(temperature)
        adjusted_temperature = temperature - non_usb_offset

    absolute_humidity = helpers.relative_to_absolute_humidity(humidity, temperature, pressure)
    humidity = 0.975 * helpers.absolute_to_relative_humidity(absolute_humidity, adjusted_temperature, pressure)
    temperature = adjusted_temperature

    # Read from external BME688 sensor
    ext_temperature = bme688_data[0]
    ext_humidity = bme688_data[2]
    ext_pressure = bme688_data[1] / 100.0
    ext_gas_resistance = bme688_data[3]
    # an approximate air quality calculation that accounts for the effect of
    # humidity on the gas sensor
    # https://forums.pimoroni.com/t/bme680-observed-gas-ohms-readings/6608/25
    ext_aqi = round(math.log(ext_gas_resistance) + 0.04 * ext_humidity, 1)

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
