# enviro config file

# you may edit this file by hand but if you enter provisioning mode
# then the file will be automatically overwritten with new details

provisioned = True

# enter a nickname for this board
nickname = 'enviro-growbox'

# network access details
wifi_ssid = 'SSID'
wifi_password = 'WIFI-PW'
# wifi_ssid = 'Robotron'
# wifi_password = 'robertvb'
wifi_country = 'DE'

# how often to wake up and take a reading (in minutes)
reading_frequency = 5

# how often to trigger a resync of the onboard RTC (in hours)
resync_frequency = 168

# where to upload to ("http", "mqtt", "adafruit_io", "influxdb")
destination = 'influxdb'

# how often to upload data (number of cached readings)
upload_frequency = 1

# web hook settings
custom_http_url = ''
custom_http_username = ''
custom_http_password = ''

# mqtt broker settings
mqtt_broker_address = ''
mqtt_broker_username = ''
mqtt_broker_password = ''
# mqtt broker if using local SSL
mqtt_broker_ca_file = None

# adafruit ui settings
adafruit_io_username = ''
adafruit_io_key = ''

# influxdb settings
influxdb_org = ''
influxdb_url = 'http://192.168.178.108:8086'
influxdb_token = ''
influxdb_bucket = 'home'

# grow specific settings
auto_water = True
moisture_target_a = 0
moisture_target_b = 0
moisture_target_c = 0

# compensate for usb power
usb_power_temperature_offset = 4.5

