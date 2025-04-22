# boss.py — Contains all custom logic

import time, math, os
from enviro import i2c
from breakout_bme68x import BreakoutBME68X
from phew import logging
from enviro.helpers import *  # for constants only (e.g., CRITICAL_WATER_TEMPERATURE)

# === Boss Settings ===
MOISTURE_MIN = [0, 0, 0]
MOISTURE_MAX = [70, 70, 70]
STATUS_FILE = "wtr_status.txt"
CHANNEL_NAMES = ["A", "B", "C"]
CAL_FILE = "grow_calibration_data.txt"
CAL_FILE_USB = "grow_calibration_data_usb.txt"

# === Correction Curves ===
TEMP_CURVE = [-20, -10, 0, 20, 23.15, 23.3, 30]
TEMP_OFFSETS = [0.86, 0.86, 0.86, 0.86, 0.86, 0.9, 0.95]
TEMP_CURVE_USB = [-20, -10, 0, 20, 29.41, 29.43]
TEMP_OFFSETS_USB = [1, 1, 1.5, 2, 2.84, 2.87]

HUM_CURVE = [-20, -10, 0, 20, 25, 30]
HUM_FACTORS = [0.99] * 6
HUM_CURVE_USB = TEMP_CURVE_USB
HUM_FACTORS_USB = [1, 1, 1, 0.975, 1.04, 1.02]

class Boss:
    def __init__(self, calibrate=False):
        self.sensor = BossSensor(calibrate=calibrate)
        self.status = BossWateringStatus()

    def run_watering(self, moisture_levels, pump_pins, drip_noise=None, read_moisture=None):
        from enviro import cache_upload, helpers

        min_targets = MOISTURE_MIN
        max_targets = MOISTURE_MAX
        max_watering_time = 10
        did_water = [False, False, False]  # Flag to track watering activity

        for i in range(3):
            status = self.status.get(i)
            continue_watering = status == f"unfinished_{i}" or moisture_levels[i] < min_targets[i]

            if continue_watering:
                logging.info(f"> sensor {CHANNEL_NAMES[i]} below minimum moisture target {min_targets[i]} (currently at {int(moisture_levels[i])}).")

                try:
                    from enviro import config
                    auto_water = config.auto_water
                except:
                    auto_water = True

                if auto_water:
                    logging.info(f"  - starting pump {CHANNEL_NAMES[i]} until moisture reaches {max_targets[i]} or for a maximum of {max_watering_time} seconds")
                    pump_pins[i].value(1)
                    start_time = time.time()

                    # Log pump ON event
                    did_water[i] = True  # Mark that watering happened
                    pump_state = pump_pins[i].value()
                    self.log_moisture_and_pump(i, moisture_levels[i], pump_state)
                    
                    while True:
                        current_level = read_moisture()[i]
                        if current_level >= max_targets[i]:
                            self.status.clear(i)
                            break
                        if time.time() - start_time > max_watering_time:
                            logging.info(f"  - maximum watering time reached for pump {CHANNEL_NAMES[i]}")
                            self.status.set_unfinished(i)
                            break
                        time.sleep(0.5)

                    pump_pins[i].value(0)
                    logging.info(f"  - stopped pump {CHANNEL_NAMES[i]}")

                    # Log pump OFF event
                    time.sleep(1)
                    pump_state = pump_pins[i].value()
                    self.log_moisture_and_pump(i, read_moisture()[i], pump_state)
                
                else:
                    logging.info(f"  - auto watering disabled")
                    if drip_noise:
                        for j in range(i + 1):
                            drip_noise()
                        time.sleep(0.5)
                    else:
                        logging.info(f"  - no drip_noise defined; skipping beep")
        return did_water
                        
    def log_moisture_and_pump(self, i, moisture, pump_status):
        from enviro import cache_upload
        from ucollections import OrderedDict
    
        reading = OrderedDict({
            f"moisture_{CHANNEL_NAMES[i].lower()}": round(moisture, 2),
            f"pump_{CHANNEL_NAMES[i].lower()}": pump_status
        })

        logging.debug(f"  - caching moisture and pump status for channel {CHANNEL_NAMES[i]}")
        cache_upload(reading)
    
    def get_external_data(self, bme280_data, is_usb):
        t, p, h = bme280_data[0], bme280_data[1] / 100, bme280_data[2]
        ext = BossSensor().read()

        temp_curve = TEMP_CURVE_USB if is_usb else TEMP_CURVE
        temp_offsets = TEMP_OFFSETS_USB if is_usb else TEMP_OFFSETS
        hum_curve = HUM_CURVE_USB if is_usb else HUM_CURVE
        hum_factors = HUM_FACTORS_USB if is_usb else HUM_FACTORS

        temp_offset = BossHelpers.interpolate(t, temp_curve, temp_offsets)
        adj_temp = t - temp_offset

        abs_h = BossHelpers.relative_to_absolute_humidity_p(h, t, p)
        rel_h = BossHelpers.absolute_to_relative_humidity_p(abs_h, adj_temp, p)
        hum_factor = BossHelpers.interpolate(t, hum_curve, hum_factors)
        corrected_h = rel_h * hum_factor

        ext_abs_h = BossHelpers.relative_to_absolute_humidity_p(ext["humidity"], ext["temperature"], ext["pressure"])
        calc_h = BossHelpers.absolute_to_relative_humidity_p(ext_abs_h, adj_temp, ext["pressure"])
        delta_h = calc_h - corrected_h

        if self.sensor.calibrate:
            calc_temp_offset = t - ext["temperature"]
            calc_adj_temp = t - calc_temp_offset
            calc_abs_h = BossHelpers.relative_to_absolute_humidity_p(h, t, p)
            calc_adj_h = BossHelpers.absolute_to_relative_humidity_p(calc_abs_h, calc_adj_temp, p)
            calc_hum_factor = ext["humidity"] / calc_adj_h if calc_adj_h else 1.0

            BossHelpers.append_calibration(
                t,
                calc_temp_offset,
                calc_adj_h,
                calc_hum_factor,
                is_usb
            )

        return {
            "temperature": round(adj_temp, 2),
            "humidity": round(corrected_h, 2),
            "pressure": round(p, 2),
            "dew_point": round(BossHelpers.calculate_dew_point(adj_temp, corrected_h), 2),
            "ext_temperature": round(ext["temperature"], 2),
            "ext_humidity": round(ext["humidity"], 2),
            "ext_pressure": round(ext["pressure"], 2),
            "ext_gas_resistance": round(ext["gas_resistance"]),
            "ext_aqi": round(math.log(ext["gas_resistance"]) + 0.04 * ext["humidity"], 1),
            "ext_dew_point": round(BossHelpers.calculate_dew_point(ext["temperature"], ext["humidity"]), 2),
            "calc_humidity": round(calc_h, 2),
            "delta_humidity": round(delta_h, 2),
        }

class BossSensor:
    def __init__(self, calibrate=False):
        self.bme688 = BreakoutBME68X(i2c, address=0x76)
        self.calibrate = calibrate

    def read(self):
        data = self.bme688.read()
        return {
            "temperature": data[0],
            "pressure": data[1] / 100.0,
            "humidity": data[2],
            "gas_resistance": data[3]
        }

class BossWateringStatus:
    def _load(self):
        try:
            with open(STATUS_FILE, "r") as f:
                return {int(k): float(v) for k, v in (line.strip().split(":") for line in f if ":" in line)}
        except:
            return {}

    def _save(self, data):
        with open(STATUS_FILE, "w") as f:
            for k, v in data.items():
                f.write(f"{k}:{v}\n")

    def get(self, pump_id):
        return self._load().get(pump_id)

    def set_unfinished(self, pump_id):
        data = self._load()
        data[pump_id] = time.time()
        self._save(data)

    def clear(self, pump_id):
        data = self._load()
        data.pop(pump_id, None)
        self._save(data)

class BossHelpers:
    @staticmethod
    def append_calibration(temp, offset, adjusted_humidity, factor, is_usb):
        fname = CAL_FILE_USB if is_usb else CAL_FILE
        try:
            with open(fname, "r") as f:
                lines = f.readlines()
                tp = eval(lines[0].split("=")[1])
                to = eval(lines[1].split("=")[1])
                hp = eval(lines[2].split("=")[1])
                hf = eval(lines[3].split("=")[1])
        except (SyntaxError, OSError):
            tp, to, hp, hf = [], [], [], []

        tp.append(round(temp, 2))
        to.append(round(offset, 2))
        hp.append(round(temp, 2))
        hf.append(round(factor, 2))

        tp, to = zip(*sorted(zip(tp, to))) if tp else ([], [])
        hp, hf = zip(*sorted(zip(hp, hf))) if hp else ([], [])

        with open(fname, "w") as f:
            f.write(f"temperature_points = {list(tp)}\n")
            f.write(f"temperature_offsets = {list(to)}\n")
            f.write(f"humidity_points = {list(hp)}\n")
            f.write(f"humidity_factors = {list(hf)}\n")

    @staticmethod
    def interpolate(value, points, corrections):
        if value <= points[0]:
            return corrections[0]
        if value >= points[-1]:
            return corrections[-1]
        for i in range(1, len(points)):
            if points[i - 1] <= value <= points[i]:
                t1, t2 = points[i - 1], points[i]
                c1, c2 = corrections[i - 1], corrections[i]
                return c1 + (c2 - c1) * (value - t1) / (t2 - t1)

    @staticmethod
    def celcius_to_kelvin(temp_c):
        return temp_c + 273.15

    @staticmethod
    def get_actual_vapor_pressure_p(rh, temp_k, pressure_hpa):
        return BossHelpers.get_saturation_vapor_pressure_p(temp_k, pressure_hpa) * (rh / 100)

    @staticmethod
    def get_saturation_vapor_pressure_p(temp_k, pressure_hpa):
        v = 1 - (temp_k / CRITICAL_WATER_TEMPERATURE)
        f = 1.00071 * math.exp(0.0000045 * pressure_hpa)
        a1, a2, a3 = -7.85951783, 1.84408259, -11.7866497
        a4, a5, a6 = 22.6807411, -15.9618719, 1.80122502
        return f * CRITICAL_WATER_PRESSURE * math.exp(
            CRITICAL_WATER_TEMPERATURE / temp_k *
            (a1*v + a2*v**1.5 + a3*v**3 + a4*v**3.5 + a5*v**4 + a6*v**7.5)
        )

    @staticmethod
    def relative_to_absolute_humidity_p(rh, temp_c, pressure_hpa):
        temp_k = BossHelpers.celcius_to_kelvin(temp_c)
        return BossHelpers.get_actual_vapor_pressure_p(rh, temp_k, pressure_hpa) / (WATER_VAPOR_SPECIFIC_GAS_CONSTANT * temp_k)

    @staticmethod
    def absolute_to_relative_humidity_p(ah, temp_c, pressure_hpa):
        temp_k = BossHelpers.celcius_to_kelvin(temp_c)
        svp = BossHelpers.get_saturation_vapor_pressure_p(temp_k, pressure_hpa)
        return (WATER_VAPOR_SPECIFIC_GAS_CONSTANT * temp_k * ah) / svp * 100

    @staticmethod
    def calculate_dew_point(temp_c, rh):
        if temp_c >= 0:
            K0, K1, K2 = 6.1094, 17.625, 243.04
        else:
            K0, K1, K2 = 6.1121, 22.587, 273.86
        alpha = math.log(rh / 100.0) + (K1 * temp_c) / (K2 + temp_c)
        return (K2 * alpha) / (K1 - alpha)

