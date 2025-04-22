# grow.py — Main logic with boss feature toggles

USE_BOSS_SENSOR_LOGIC = True
USE_BOSS_WATERING_LOGIC = True
USE_BOSS_CALIBRATION = False

import sys
if "/" not in sys.path:
    sys.path.append("/")

from enviro.boss import Boss, BossHelpers

import time
import math
from machine import Pin, PWM
from enviro import i2c, config
from phew import logging
from breakout_bme280 import BreakoutBME280
from breakout_ltr559 import BreakoutLTR559
from ucollections import OrderedDict

# === Boss init ===
boss = Boss(calibrate=USE_BOSS_CALIBRATION)

# === Hardware setup ===
bme280 = BreakoutBME280(i2c, 0x77)
ltr559 = BreakoutLTR559(i2c)
piezo_pwm = PWM(Pin(28))

pump_pins = [
    Pin(12, Pin.OUT, value=0),
    Pin(11, Pin.OUT, value=0),
    Pin(10, Pin.OUT, value=0),
]

moisture_sensor_pins = [
    Pin(15, Pin.IN, Pin.PULL_DOWN),
    Pin(14, Pin.IN, Pin.PULL_DOWN),
    Pin(13, Pin.IN, Pin.PULL_DOWN),
]

CHANNEL_NAMES = ["A", "B", "C"]

# === Moisture sensor ===
def moisture_readings():
    results = []
    for i in range(3):
        sensor = moisture_sensor_pins[i]
        last_value = sensor.value()
        start = time.ticks_ms()
        first = last = None
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
        average = max(20, min(80, average))
        scaled = ((average - 20) / 60) * 100
        results.append(round(scaled, 2))
    return results

# === Beep ===
def drip_noise():
    piezo_pwm.duty_u16(32768)
    for i in range(10):
        piezo_pwm.freq((i * 20) ** 2 + 1000)
        time.sleep(0.02)
    piezo_pwm.duty_u16(0)

# === Original watering logic (used only if toggle is off) ===
def water(moisture_levels):
    targets = [config.moisture_target_a, config.moisture_target_b, config.moisture_target_c]
    for i in range(3):
        if moisture_levels[i] < targets[i]:
            logging.info(f"> Channel {CHANNEL_NAMES[i]} below target {targets[i]} (currently {moisture_levels[i]})")
            if config.auto_water:
                pump_pins[i].value(1)
                time.sleep(5)
                pump_pins[i].value(0)
            else:
                for _ in range(i + 1):
                    drip_noise()
                time.sleep(0.5)

# === Main reading function ===
def get_sensor_readings(seconds_since_last, is_usb_power):
    bme280.read()
    time.sleep(0.1)
    bme_data = bme280.read()
    ltr_data = ltr559.get_reading()
    moisture = moisture_readings()

    # Watering logic (choose original or boss)
    if USE_BOSS_WATERING_LOGIC:
        did_water = boss.run_watering(moisture, pump_pins, drip_noise=drip_noise, read_moisture=moisture_readings)
        # Only re-read moisture for watered channels
        for i in range(3):
            if did_water[i]:
                moisture[i] = moisture_readings()[i]
    else:
        did_water = [False, False, False]
        water(moisture)

    press = bme_data[1] / 100.0
    ltr_lux = ltr_data[BreakoutLTR559.LUX]

    # Sensor logic (choose original or boss-calibrated)
    if USE_BOSS_SENSOR_LOGIC:
        boss_data = boss.get_external_data(bme_data, is_usb_power)
        readings = OrderedDict({
            "temperature": boss_data["temperature"],
            "humidity": boss_data["humidity"],
            "pressure": boss_data["pressure"],
            "luminance": round(ltr_lux, 2),
            "moisture_a": round(moisture[0], 2) if not did_water[0] else None,
            "moisture_b": round(moisture[1], 2) if not did_water[1] else None,
            "moisture_c": round(moisture[2], 2) if not did_water[2] else None,
            "dew_point": boss_data["dew_point"],
        })
        # Append the remaining boss readings
        for key in ("temperature", "humidity", "pressure", "dew_point"):
            boss_data.pop(key, None)
        readings.update(boss_data)
        
    else:
        temp = bme_data[0]
        humid = bme_data[2]
        dew = BossHelpers.calculate_dew_point(temp, humid)

        readings = OrderedDict({
            "temperature": round(temp, 2),
            "humidity": round(humid, 2),
            "pressure": round(press, 2),
            "luminance": round(ltr_lux, 2),
            "moisture_a": round(moisture[0], 2) if not did_water[0] else None,
            "moisture_b": round(moisture[1], 2) if not did_water[1] else None,
            "moisture_c": round(moisture[2], 2) if not did_water[2] else None,
            "dew_point": round(dew, 2),
        })

    # Clean up None entries
    for key in list(readings):
        if readings[key] is None:
            del readings[key]
    
    return readings
