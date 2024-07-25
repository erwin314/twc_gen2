import esphome.codegen as cg
from esphome import cpp_generator as cppg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import (
    ENTITY_CATEGORY_DIAGNOSTIC
)

from . import TwcGen2Controller

CONF_TWC_GEN2_ID = "twc_gen2_id"
CONF_Cnr_SERIAL = "c{}_serial"
CONF_Cnr_VIN = "c{}_vin"

AUTO_LOAD = ["text_sensor"]

CONFIG_SCHEMA = {
    cv.GenerateID(CONF_TWC_GEN2_ID): cv.use_id(TwcGen2Controller),
    **{
        cv.Optional(CONF_Cnr_SERIAL.format(i+1)): text_sensor.text_sensor_schema(icon="mdi:identifier" , entity_category=ENTITY_CATEGORY_DIAGNOSTIC)
        for i in range(0, 3)
      },
    **{
        cv.Optional(CONF_Cnr_VIN.format(i+1)): text_sensor.text_sensor_schema(icon="mdi:car-side")
        for i in range(0, 3)
      }
}

async def to_code(config):
    parent = await cg.get_variable(config[CONF_TWC_GEN2_ID])

    for i in range(0,3):
        if c := config.get(CONF_Cnr_SERIAL.format(i+1)):
            sens = await text_sensor.new_text_sensor(c)
            cg.add(cppg.AssignmentExpression(None, None, parent.serial_text_sensor[i], sens))

        if c := config.get(CONF_Cnr_VIN.format(i+1)):
            sens = await text_sensor.new_text_sensor(c)
            cg.add(cppg.AssignmentExpression(None, None, parent.vin_text_sensor[i], sens))
