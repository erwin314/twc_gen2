# twc_gen2
Tesla Wall Connector (generation 2) ESPHome controller.

Features:
- control multiple Tesla Wall Connector's (TWC's)
- load balancing between TWC's
- configure maximum current for each individual TWC
- configure maximum current for all TWC's combined
- maximum current for all TWC's combined can be controller from Home Assistant
- individual TWC's can be turned on/off from Home Assistant
- sensors for current, voltage, VIN number, etc.


This has been tested using an Olimex ESP32-POE-ISO-IND and an Olimex MOD-RS485.

To find the serial number of your TWC, check the sticker located on the side of the device (the same side as the Reset button). The serial number is labeled as the “TSN” and consists of 11 alphanumeric characters, for example: A18B0001234.

Example config file:
```yaml
esphome:
  name: ...
  friendly_name: ...

esp32:
  board: esp32-poe-iso
  framework:
    type: arduino

# Enable logging
logger:
  level: DEBUG
  logs:
    component: ERROR
    twc_gen2: DEBUG

# Enable Home Assistant API
api:
  encryption:
    key: ...

ota:
  - platform: esphome
    password: ...

# Enable ethernet (for network and PoE)
ethernet:
  type: LAN8720
  mdc_pin: GPIO23
  mdio_pin: GPIO18
  clk_mode: GPIO17_OUT
  phy_addr: 0
  power_pin: GPIO12

uart:
  id: twc_uart
  tx_pin: GPIO4
  rx_pin: GPIO36
  baud_rate: 9600
  data_bits: 8
  parity: NONE
  stop_bits: 1

external_components:
  - source: github://erwin314/twc_gen2
    components: [twc_gen2]
  # Or you can use a local copy:
  #- source:
  #    type: local
  #    path: /workspaces/esphome/esphome/components
  #  components: [twc_gen2]

twc_gen2:
  uart_id: twc_uart
  flow_control_pin: 14
  read_only: false
  # Add the serial numbers of you Tesla Wall Connector's here.
  # Set read_only to True if you don't know your serial number(s),
  # you can than look in the logs to find the serial number(s).
  c1_serial: A1.........
  c2_serial: A1.........
  global_limit: 16
  c1_limit: 16
  c2_limit: 16

number:
  - platform: twc_gen2
    global_limit:
      name: TWC limiet
      min_value: 12
      max_value: 32

switch:
  - platform: twc_gen2
    c1_allow_charging:
      name: 1 Allow Charging
    c2_allow_charging:
      name: 2 Allow Charging

text_sensor:
  - platform: twc_gen2
    c1_serial:
      name: 1 Serial
    c2_serial:
      name: 2 Serial
    c1_vin:
      name: 1 VIN
    c2_vin:
      name: 2 VIN

sensor:
  platform: twc_gen2
  c1_voltage1:
    name: 1 Voltage L1
  c1_voltage2:
    name: 1 Voltage L2
  c1_voltage3:
    name: 1 Voltage L3
  c2_voltage1:
    name: 2 Voltage L1
  c2_voltage2:
    name: 2 Voltage L2
  c2_voltage3:
    name: 2 Voltage L3

  c1_current1:
    name: 1 Current L1
  c1_current2:
    name: 1 Current L2
  c1_current3:
    name: 1 Current L3
  c2_current1:
    name: 2 Current L1
  c2_current2:
    name: 2 Current L2
  c2_current3:
    name: 2 Current L3

  c1_energy:
    name: 1 Energy
  c2_energy:
    name: 2 Energy

  c1_state:
    name: 1 State
  c2_state:
    name: 2 State

  c1_actual_current:
    name: 1 Actual Current
  c2_actual_current:
    name: 2 Actual Current
```
