#
# Credit goes to WinterDragoness from the Tesla Motor Club forums for figuring out the protocol (with many others in the forum also giving help)
# and Craig 128 for the original C code.
#
# And with the following repositories/people this code has been made possible:
# https://github.com/dracoventions/TWCManager/blob/master/TWCManager.py
# https://github.com/ngardiner/TWCManager/tree/main
# https://github.com/craigpeacock/TWC
# https://github.com/jnicolson/twc-controller/tree/main
# https://github.com/jnicolson/esphome-twc-controller
# https://github.com/craigpeacock/Tesla-ChargePort-Decoder/blob/master/ChargePort.xlsx
#
from esphome import pins
import esphome.codegen as cg
from esphome import cpp_generator as cppg
import esphome.config_validation as cv
from esphome.core import ID
from esphome.cpp_helpers import gpio_pin_expression
from esphome.components import uart
from esphome.const import (
    CONF_FLOW_CONTROL_PIN,
    CONF_ID,
)

AUTO_LOAD = ["number", "switch", "sensor", "text_sensor"]
DEPENDENCIES = ["uart"]

CONF_READ_ONLY = "read_only"
CONF_C1_SERIAL = "c1_serial"
CONF_C2_SERIAL = "c2_serial"
CONF_C3_SERIAL = "c3_serial"
CONF_GLOBAL_LIMIT = "global_limit"
CONF_C1_LIMIT = "c1_limit"
CONF_C2_LIMIT = "c2_limit"
CONF_C3_LIMIT = "c3_limit"

# Define our namespace
twc_gen2_ns = cg.esphome_ns.namespace('twc_gen2')

# Define our classes
TwcGen2Controller = twc_gen2_ns.class_('TwcGen2Controller', cg.Component)


# This is the main schema definition.
# It is used to validate the config yaml provided by the end-user.
# It also sets the default values and defines class types.
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(TwcGen2Controller),
    cv.Required(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
    cv.Optional(CONF_READ_ONLY, False): bool,
    cv.Optional(CONF_C1_SERIAL): cv.All(str, cv.Length(min=11, max=11)),
    cv.Optional(CONF_C2_SERIAL): cv.All(str, cv.Length(min=11, max=11)),
    cv.Optional(CONF_C3_SERIAL): cv.All(str, cv.Length(min=11, max=11)),
    cv.Required(CONF_GLOBAL_LIMIT): cv.int_range(6,3*32),
    cv.Optional(CONF_C1_LIMIT): cv.int_range(6,32),
    cv.Optional(CONF_C2_LIMIT): cv.int_range(6,32),
    cv.Optional(CONF_C3_LIMIT): cv.int_range(6,32),
    }).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)


# Generate C++ code using the config values (from the yaml provided by the end-user)
async def to_code(config):
    # Create a C++ new instance of the TwcGen2Controler class
    controller = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(controller, config)

    # Connect it to the uart device
    await uart.register_uart_device(controller, config)
    # and set the flow control pin
    pin = await gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
    cg.add(controller.set_flow_control_pin(pin))

    read_only = config[CONF_READ_ONLY]
    cg.add(controller.set_read_only(read_only))

    if limit := config.get(CONF_GLOBAL_LIMIT):
         cg.add(cppg.AssignmentExpression(None, None, controller.global_limit, limit))
    if limit := config.get(CONF_C1_LIMIT):
         cg.add(cppg.AssignmentExpression(None, None, controller.twc_limit[0], limit))
    if limit := config.get(CONF_C2_LIMIT):
         cg.add(cppg.AssignmentExpression(None, None, controller.twc_limit[1], limit))
    if limit := config.get(CONF_C3_LIMIT):
         cg.add(cppg.AssignmentExpression(None, None, controller.twc_limit[2], limit))

    if serial := config.get(CONF_C1_SERIAL):
         cg.add(controller.set_connector_serial(0, serial))
    if serial := config.get(CONF_C2_SERIAL):
         cg.add(controller.set_connector_serial(1, serial))
    if serial := config.get(CONF_C3_SERIAL):
         cg.add(controller.set_connector_serial(2, serial))



