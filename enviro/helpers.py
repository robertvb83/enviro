from enviro.constants import *
import machine, math, os, time

# miscellany
# ===========================================================================
def datetime_string():
    dt = machine.RTC().datetime()
    return "{0:04d}-{1:02d}-{2:02d}T{4:02d}:{5:02d}:{6:02d}Z".format(*dt)


def datetime_file_string():
    dt = machine.RTC().datetime()
    return "{0:04d}-{1:02d}-{2:02d}T{4:02d}_{5:02d}_{6:02d}Z".format(*dt)


def date_string():
    dt = machine.RTC().datetime()
    return "{0:04d}-{1:02d}-{2:02d}".format(*dt)


def timestamp(dt):
    year = int(dt[0:4])
    month = int(dt[5:7])
    day = int(dt[8:10])
    hour = int(dt[11:13])
    minute = int(dt[14:16])
    second = int(dt[17:19])
    return time.mktime((year, month, day, hour, minute, second, 0, 0))


def uid():
    return "{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}".format(*machine.unique_id())


# file management helpers
# ===========================================================================
def file_size(filename):
    try:
        return os.stat(filename)[6]
    except OSError:
        return None


def file_exists(filename):
    try:
        return (os.stat(filename)[0] & 0x4000) == 0
    except OSError:
        return False


def mkdir_safe(path):
    try:
        os.mkdir(path)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise
        pass  # directory already exists, this is fine


def copy_file(source, target):
    with open(source, "rb") as infile:
        with open(target, "wb") as outfile:
            while True:
                chunk = infile.read(1024)
                if not chunk:
                    break
                outfile.write(chunk)


# temperature and humidity helpers
# ===========================================================================

# https://www.calctool.org/atmospheric-thermodynamics/absolute-humidity#what-is-and-how-to-calculate-absolute-humidity
def relative_to_absolute_humidity(relative_humidity, temperature_in_c, pressure_in_hpa):
    temperature_in_k = celcius_to_kelvin(temperature_in_c)
    actual_vapor_pressure = get_actual_vapor_pressure(relative_humidity, temperature_in_k, pressure_in_hpa)

    return actual_vapor_pressure / (WATER_VAPOR_SPECIFIC_GAS_CONSTANT * temperature_in_k)


def absolute_to_relative_humidity(absolute_humidity, temperature_in_c, pressure_in_hpa):
    temperature_in_k = celcius_to_kelvin(temperature_in_c)
    saturation_vapor_pressure = get_saturation_vapor_pressure(temperature_in_k, pressure_in_hpa)

    return ((WATER_VAPOR_SPECIFIC_GAS_CONSTANT * temperature_in_k * absolute_humidity) / saturation_vapor_pressure * 100)


def celcius_to_kelvin(temperature_in_c):
    return temperature_in_c + 273.15


# https://www.calctool.org/atmospheric-thermodynamics/absolute-humidity#actual-vapor-pressure
# http://cires1.colorado.edu/~voemel/vp.html
def get_actual_vapor_pressure(relative_humidity, temperature_in_k, pressure_in_hpa):
    return get_saturation_vapor_pressure(temperature_in_k, pressure_in_hpa) * (relative_humidity / 100)


def get_saturation_vapor_pressure(temperature_in_k, pressure_in_hpa):
    v = 1 - (temperature_in_k / CRITICAL_WATER_TEMPERATURE)
    f = 1.00071 * math.exp(0.0000045 * pressure_in_hpa)  # Enhancement Factor

    # empirical constants
    a1 = -7.85951783
    a2 = 1.84408259
    a3 = -11.7866497
    a4 = 22.6807411
    a5 = -15.9618719
    a6 = 1.80122502

    return (f * CRITICAL_WATER_PRESSURE * math.exp(CRITICAL_WATER_TEMPERATURE / temperature_in_k * (a1 * v + a2 * v**1.5 + a3 * v**3 + a4 * v**3.5 + a5 * v**4 + a6 * v**7.5)))


# Dew point calculation using the Magnus formula
def calculate_dew_point(temperature_in_c, relative_humidity):
    if temperature_in_c >= 0:
        K_0 = 6.1094
        K_1 = 17.625
        K_2 = 243.04
    else:
        K_0 = 6.1121
        K_1 = 22.587
        K_2 = 273.86

    # Magnus formula
    alpha = math.log(relative_humidity / 100.0) + (K_1 * temperature_in_c) / (K_2 + temperature_in_c)
    dew_point = (K_2 * alpha) / (K_1 - alpha)

    return dew_point


def interpolate(value, points, corrections):
    """
    Generic function to interpolate a correction factor based on a value.
    
    :param value: The input value for which we want to find the correction (e.g., temperature or humidity).
    :param points: Array of reference points (e.g., temperature points or humidity points).
    :param corrections: Array of correction factors corresponding to the points.
    
    :return: The interpolated correction factor.
    """
    # If value is outside the range, cap it
    if value <= points[0]:
        return corrections[0]
    elif value >= points[-1]:
        return corrections[-1]

    # Linear interpolation within the defined range
    for i in range(1, len(points)):
        if points[i - 1] <= value <= points[i]:
            t1, t2 = points[i - 1], points[i]
            c1, c2 = corrections[i - 1], corrections[i]
            return c1 + (c2 - c1) * (value - t1) / (t2 - t1)
