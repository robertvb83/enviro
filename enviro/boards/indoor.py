import enviro.helpers as helpers
import math
from breakout_bme68x import BreakoutBME68X
from breakout_bh1745 import BreakoutBH1745

from enviro import config
from enviro import i2c


# temperature and humidity array definitions

# For temperature offset
temperature_points = [-20, -10, 20, 23.5, 30]
temperature_offsets = [1, 1, 1.1, 1.55, 2]

# For humidity factor
humidity_points = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
humidity_factors = [1, 1, 1, 1, 0.966, 0.966, 0.966, 0.966, 1, 1, 1]

# Onboard BME688 sensor
bme688 = BreakoutBME68X(i2c, address=0x77)

# External BME688 sensor
ext_bme688 = BreakoutBME68X(i2c, address=0x76)

bh1745 = BreakoutBH1745(i2c)
# need to write default values back into bh1745 chip otherwise it
# reports bad results (this is undocumented...)
i2c.writeto_mem(0x38, 0x44, b"\x02")


def lux_from_rgbc(r, g, b, c):
    if g < 1:
        tmp = 0
    elif c / g < 0.160:
        tmp = 0.202 * r + 0.766 * g
    else:
        tmp = 0.159 * r + 0.646 * g
    tmp = 0 if tmp < 0 else tmp
    integration_time = 160
    gain = 1
    return round(tmp / gain / integration_time * 160)


def colour_temperature_from_rgbc(r, g, b, c):
    if (g < 1) or (r + g + b < 1):
        return 0
    r_ratio = r / (r + g + b)
    b_ratio = b / (r + g + b)
    e = 2.71828
    ct = 0
    if c / g < 0.160:
        b_eff = min(b_ratio * 3.13, 1)
        ct = ((1 - b_eff) * 12746 * (e ** (-2.911 * r_ratio))) + (b_eff * 1637 * (e ** (4.865 * b_ratio)))
    else:
        b_eff = min(b_ratio * 10.67, 1)
        ct = ((1 - b_eff) * 16234 * (e ** (-2.781 * r_ratio))) + (b_eff * 1882 * (e ** (4.448 * b_ratio)))
    if ct > 10000:
        ct = 10000
    return round(ct)


def append_to_calibration_file(temperature, temp_offset, adjusted_humidity, humidity_factor):
    # Read existing data from the file
    try:
        with open("indoor_calibration_data.txt", "r") as f:
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
    with open("indoor_calibration_data.txt", "w") as f:
        f.write(f"temperature_points = {list(temperature_points)}\n")
        f.write(f"temperature_offsets = {list(temperature_offsets)}\n")
        f.write(f"humidity_points = {list(humidity_points)}\n")
        f.write(f"humidity_factors = {list(humidity_factors)}\n")


def get_sensor_readings(seconds_since_last, is_usb_power):
    # Onboard sensor data
    data = bme688.read()
    temperature = data[0]
    humidity = data[2]
    pressure = data[1] / 100.0

    # External sensor data
    ext_data = ext_bme688.read()
    ext_temperature = ext_data[0]
    ext_humidity = ext_data[2]

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

    # (only onboard sensor) Compensate for additional heating when on usb power - this also changes the
    # relative humidity value.
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

    gas_resistance = data[3]
    # an approximate air quality calculation that accounts for the effect of
    # humidity on the gas sensor
    # https://forums.pimoroni.com/t/bme680-observed-gas-ohms-readings/6608/25
    aqi = round(math.log(gas_resistance) + 0.04 * humidity, 1)

    # External sensor pressure and gas resistance
    ext_pressure = ext_data[1] / 100.0
    ext_gas_resistance = ext_data[3]

    # External air quality index
    ext_aqi = round(math.log(ext_gas_resistance) + 0.04 * ext_humidity, 1)

    bh1745.measurement_time_ms(160)
    r, g, b, c = bh1745.rgbc_raw()

    # Calculate external absolute humidity using helpers
    ext_absolute_humidity = helpers.relative_to_absolute_humidity(ext_humidity, ext_temperature, ext_pressure)

    # Calculate predicted rel. humidity after venting using helpers
    calc_humidity = helpers.absolute_to_relative_humidity(ext_absolute_humidity, temperature, ext_pressure)

    # Calculate delta rel. humidity before/after venting using helpers
    delta_humidity = calc_humidity - humidity

    # Calculate dew point using helpers
    dew_point = helpers.calculate_dew_point(temperature, humidity)
    ext_dew_point = helpers.calculate_dew_point(ext_temperature, ext_humidity)

    from ucollections import OrderedDict

    return OrderedDict(
        {
            # Onboard sensor readings
            "temperature": round(temperature, 2),
            "humidity": round(humidity, 2),
            "pressure": round(pressure, 2),
            "gas_resistance": round(gas_resistance),
            "aqi": aqi,
            "luminance": lux_from_rgbc(r, g, b, c),
            "color_temperature": colour_temperature_from_rgbc(r, g, b, c),
            "dew_point": round(dew_point, 2),
            # External sensor readings
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
