# GW-ESP8266-BME680-BSEC
MySensors gateway on ESP8266 with BME680 Air quality sensor.

## Based on the following examples:
MySensors/GatewayESP8266
BSEC Software Library/basic_config_state

## Hardware
It is currently running on WeMos D1 Mini Pro.

# Prerequisites
## Arduino IDE 1.8.5
Arduino IDE 1.8.5 can be downloaded from this [link](https://www.arduino.cc/download_handler.php)

## MySensors
The MySensors library 2.0.x is available in the IDEs Library Manager.

## BSEC Software Library
The BSEC Software Library can be found [here](https://github.com/BoschSensortec/BSEC-Arduino-library)
used version: 0255bd177335e9ac3557c30193b98ceac4cd564b
Follow the instructions in the README.md

# BSEC Configuration Settings
The BSEC library is configured to 3.3V supply voltage, 3sec interval, 4days calibration history. (generic_33v_3s_28d)
The config files are in the BSEC binary zip "config/" folders. Copy the "bsec_serialized_configurations_iaq.h" and "bsec_serialized_configurations_iaq.c" files from the desired subfolder to change the configuration.

# WiFi configuration
Change the MY_ESP8266_SSID and MY_ESP8266_PASSWORD defines in the source. Add your network SSID and password!
