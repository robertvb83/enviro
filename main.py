# Enviro - wireless environmental monitoring and logging
#
# On first run Enviro will go into provisioning mode where it appears
# as a wireless access point called "Enviro <board type> Setup". Connect
# to the access point with your phone, tablet or laptop and follow the
# on screen instructions.
#
# The provisioning process will generate a `config.py` file which 
# contains settings like your wifi username/password, how often you
# want to log data, and where to upload your data once it is collected.
#
# You can use enviro out of the box with the options that we supply
# or alternatively you can create your own firmware that behaves how
# you want it to - please share your setups with us! :-)
#
# Need help? check out https://pimoroni.com/enviro-guide
#
# Happy data hoarding folks,
#
#   - the Pimoroni pirate crew

# uncomment the below two lines to change the amount of logging enviro will do
# from phew import logging
# logging.disable_logging_types(logging.LOG_DEBUG)

# Issue #117 where neeed to sleep on startup otherwis emight not boot
from time import sleep, sleep_ms
#sleep(0.5)
sleep(2)
from machine import Pin

# import enviro firmware, this will trigger provisioning if needed
import enviro
import os

# heartbeat pin setup at global level
# heartbeat_pin = Pin(17, Pin.OUT, value=0)  # Start LOW
def send_heartbeat():
    print("\n---Start Heartbeat---")
    print(f"Initial state: {heartbeat_pin.value()}")  # Should be 0
    
    heartbeat_pin.value(1)
    print(f"After setting HIGH: {heartbeat_pin.value()}")  # Should be 1
    sleep_ms(50)
    
    heartbeat_pin.value(0)
    print(f"After setting LOW: {heartbeat_pin.value()}")  # Should be 0
    print("---End Heartbeat---\n")

import mqttsimple

mqtt_client = None

def init_mqtt():
    global mqtt_client
    try:
        mqtt_client = mqttsimple.MQTTClient(
            client_id="enviro-grow",
            server="192.168.178.108",  # IP of your HA MQTT broker
            port=1883,
            user="youruser",
            password="yourpassword"
        )
        mqtt_client.connect()
    except Exception as e:
        print(f"MQTT connect failed: {e}")
        mqtt_client = None

def send_soft_heartbeat():
    global mqtt_client
    if mqtt_client is None:
        init_mqtt()
    if mqtt_client:
        try:
            mqtt_client.publish("enviro/heartbeat", str(int(time.time())))
            print("Heartbeat sent via MQTT")
        except Exception as e:
            print(f"MQTT publish failed: {e}")
            mqtt_client = None

try:
  # initialise enviro
  enviro.startup()

  # if the clock isn't set...
  if not enviro.is_clock_set():
    enviro.logging.info("> clock not set, synchronise from ntp server")
    if not enviro.sync_clock_from_ntp():
      # failed to talk to ntp server go back to sleep for another cycle
      enviro.halt("! failed to synchronise clock")  

  # check disk space...
  if enviro.low_disk_space():
    # less than 10% of diskspace left, this probably means cached results
    # are not getting uploaded so warn the user and halt with an error
    
    # Issue #126 to try and upload if disk space is low
    # is an upload destination set?
    if enviro.config.destination:
      enviro.logging.error("! low disk space. Attempting to upload file(s)")

      # if we have enough cached uploads...
      enviro.logging.info(f"> {enviro.cached_upload_count()} cache file(s) need uploading")
      if not enviro.upload_readings():
        enviro.halt("! reading upload failed")
    else:
      # no destination so go to sleep
      enviro.halt("! low disk space")

  # TODO this seems to be useful to keep around?
  filesystem_stats = os.statvfs(".")
  enviro.logging.debug(f"> {filesystem_stats[3]} blocks free out of {filesystem_stats[2]}")

  # TODO should the board auto take a reading when the timer has been set, or wait for the time?
  # take a reading from the onboard sensors
  enviro.logging.debug(f"> taking new reading")
  reading = enviro.get_sensor_readings()

  # here you can customise the sensor readings by adding extra information
  # or removing readings that you don't want, for example:
  # 
  #   del readings["temperature"]        # remove the temperature reading
  #
  #   readings["custom"] = my_reading()  # add my custom reading value

  # is an upload destination set?
  if enviro.config.destination:
    # if so cache this reading for upload later
    enviro.logging.debug(f"> caching reading for upload")
    enviro.cache_upload(reading)

    # if we have enough cached uploads...
    if enviro.is_upload_needed():
      enviro.logging.info(f"> {enviro.cached_upload_count()} cache file(s) need uploading")
      if not enviro.upload_readings():
        enviro.halt("! reading upload failed")
    else:
      enviro.logging.info(f"> {enviro.cached_upload_count()} cache file(s) not being uploaded. Waiting until there are {enviro.config.upload_frequency} file(s)")
  else:
    # otherwise save reading to local csv file (look in "/readings")
    enviro.logging.debug(f"> saving reading locally")
    enviro.save_reading(reading)

  # Heartbeat from GPIO17 to ESP32 Pin 21 
  # send_heartbeat()
  send_soft_heartbeat()
  
  # go to sleep until our next scheduled reading
  enviro.sleep()

# handle any unexpected exception that has occurred
except Exception as exc:
  enviro.exception(exc)
