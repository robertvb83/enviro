# boss.py — Contains all custom logic

import time, math, os
from enviro import i2c
from breakout_bme68x import BreakoutBME68X
from enviro.helpers import *
from phew import logging

# === Boss Settings ===
MOISTURE_MIN = [10, 10, 10]
MOISTURE_MAX = [70, 70, 70]
DRY_PHASE_DURATION = 86400  # 24 hours
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

    def run_watering(self, moisture_levels, pump_pins):
        for i in range(3):
            now = time.time()
            last = self.status.get(i)
            needs_dryout = last is None or now - last >= DRY_PHASE_DURATION
            if not needs_dryout:
                logging.info(f"Skipping channel {CHANNEL_NAMES[i]} (still drying out)")
                continue

            if moisture_levels[i] < MOISTURE_MIN[i]:
                logging.info(f"Boss watering: {CHANNEL_NAMES[i]} low ({moisture_levels[i]} < {MOISTURE_MIN[i]})")
                pump_pins[i].value(1)
                start = time.time()
                while moisture_levels[i] < MOISTURE_MAX[i]:
                    if time.time() - start > 15:
                        self.status.set_unfinished(i)
                        break
                    time.sleep(0.5)
                else:
                    self.status.clear(i)
                pump_pins[i].value(0)

    def get_external_data(self, bme280_data, is_usb):
        t, p, h = bme280_data[0], bme280_data[1] / 100, bme280_data[2]
        ext = BossSensor().read()

        temp_curve = TEMP_CURVE_USB if is_usb else TEMP_CURVE
        temp_offsets = TEMP_OFFSETS_USB if is_usb else TEMP_OFFSETS
        hum_curve = HUM_CURVE_USB if is_usb else HUM_CURVE
        hum_factors = HUM_FACTORS_USB if is_usb else HUM_FACTORS

        usb_offset = 4.5 if is_usb else 0
        temp_offset = interpolate(t, temp_curve, temp_offsets) + usb_offset
        adj_temp = t - temp_offset

        abs_h = relative_to_absolute_humidity(h, t, p)
        rel_h = absolute_to_relative_humidity(abs_h, adj_temp, p)
        hum_factor = interpolate(t, hum_curve, hum_factors)
        corrected_h = rel_h * hum_factor

        ext_abs_h = relative_to_absolute_humidity(ext["humidity"], ext["temperature"], ext["pressure"])
        calc_h = absolute_to_relative_humidity(ext_abs_h, adj_temp, ext["pressure"])
        delta_h = calc_h - corrected_h

        if self.sensor.calibrate:
            BossHelpers.append_calibration(t, temp_offset, rel_h, hum_factor, is_usb)

        return {
            "temperature": round(adj_temp, 2),
            "humidity": round(corrected_h, 2),
            "pressure": round(p, 2),
            "dew_point": round(calculate_dew_point(adj_temp, corrected_h), 2),
        
            "ext_temperature": round(ext["temperature"], 2),
            "ext_humidity": round(ext["humidity"], 2),
            "ext_pressure": round(ext["pressure"], 2),
            "ext_gas_resistance": round(ext["gas_resistance"]),
            "ext_aqi": round(math.log(ext["gas_resistance"]) + 0.04 * ext["humidity"], 1),
            "ext_dew_point": round(calculate_dew_point(ext["temperature"], ext["humidity"]), 2),
            "calc_humidity": round(calc_h, 2),
            "delta_humidity": round(delta_h, 2),
        }

class BossSensor:
    def __init__(self, calibrate=False):
        self.bme688 = BreakoutBME68X(i2c, address=0x76)
        self.calibrate = calibrate

    def read(self):
        t, p, h, g = self.bme688.read()
        return {
            "temperature": t,
            "pressure": p / 100.0,
            "humidity": h,
            "gas_resistance": g
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
    def append_calibration(temp, offset, rel_h, factor, is_usb):
        fname = CAL_FILE_USB if is_usb else CAL_FILE
        try:
            with open(fname, "r") as f:
                lines = f.readlines()
                tp = eval(lines[0].split("=")[1])
                to = eval(lines[1].split("=")[1])
                hp = eval(lines[2].split("=")[1])
                hf = eval(lines[3].split("=")[1])
        except:
            tp, to, hp, hf = [], [], [], []

        tp.append(round(temp, 2))
        to.append(round(offset, 2))
        hp.append(round(temp, 2))
        hf.append(round(factor, 2))

        tp, to = zip(*sorted(zip(tp, to)))
        hp, hf = zip(*sorted(zip(hp, hf)))

        with open(fname, "w") as f:
            f.write(f"temperature_points = {list(tp)}\n")
            f.write(f"temperature_offsets = {list(to)}\n")
            f.write(f"humidity_points = {list(hp)}\n")
            f.write(f"humidity_factors = {list(hf)}\n")
