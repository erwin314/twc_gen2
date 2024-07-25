import esphome.codegen as cg
from esphome import cpp_generator as cppg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_KILOWATT_HOURS,
    UNIT_VOLT
)

from . import TwcGen2Controller


CONF_TWC_GEN2_ID = "twc_gen2_id"
CONF_Cnr_VOLTAGEnr = "c{}_voltage{}"
CONF_Cnr_CURRENTnr = "c{}_current{}"
CONF_Cnr_ENERGY = "c{}_energy"
CONF_Cnr_ACTUAL_CURRENT = "c{}_actual_current"
CONF_Cnr_STATE = "c{}_state"

AUTO_LOAD = ["sensor"]

CONFIG_SCHEMA = {
    cv.GenerateID(CONF_TWC_GEN2_ID): cv.use_id(TwcGen2Controller),
    **{
        cv.Optional(CONF_Cnr_VOLTAGEnr.format(i+1,j+1)): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
        )
        for i in range(0, 3)
        for j in range(0, 3)
      },
    **{
        cv.Optional(CONF_Cnr_CURRENTnr.format(i+1,j+1)): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
        )
        for i in range(0, 3)
        for j in range(0, 3)
      },
    **{
        cv.Optional(CONF_Cnr_ENERGY.format(i+1)): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT_HOURS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
        )
        for i in range(0, 3)
      },
    **{
        cv.Optional(CONF_Cnr_ACTUAL_CURRENT.format(i+1)): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
        )
        for i in range(0, 3)
      },
    **{
        cv.Optional(CONF_Cnr_STATE.format(i+1)): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
        )
        for i in range(0, 3)
      },

}

async def to_code(config):
    parent = await cg.get_variable(config[CONF_TWC_GEN2_ID])

    for i in range(0,3):

      if c:= config.get(CONF_Cnr_ENERGY.format(i+1)):
        sens = await sensor.new_sensor(c)
        cg.add(cppg.AssignmentExpression(None, None, parent.energy_sensor[i], sens))

      if c:= config.get(CONF_Cnr_ACTUAL_CURRENT.format(i+1)):
        sens = await sensor.new_sensor(c)
        cg.add(cppg.AssignmentExpression(None, None, parent.actual_current_sensor[i], sens))

      if c:= config.get(CONF_Cnr_STATE.format(i+1)):
        sens = await sensor.new_sensor(c)
        cg.add(cppg.AssignmentExpression(None, None, parent.state_sensor[i], sens))

      for j in range(0,3):

        if c := config.get(CONF_Cnr_VOLTAGEnr.format(i+1,j+1)):
            sens = await sensor.new_sensor(c)
            cg.add(cppg.AssignmentExpression(None, None, parent.voltage_sensor[i][j], sens))

        if c := config.get(CONF_Cnr_CURRENTnr.format(i+1,j+1)):
            sens = await sensor.new_sensor(c)
            cg.add(cppg.AssignmentExpression(None, None, parent.current_sensor[i][j], sens))

