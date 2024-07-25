import esphome.codegen as cg
from esphome import cpp_generator as cppg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import (
    CONF_INITIAL_VALUE,
    CONF_NAME,
    DEVICE_CLASS_SWITCH,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_POWER
)

from . import (
    twc_gen2_ns,
    TwcGen2Controller
)



CONF_TWC_GEN2_ID = "twc_gen2_id"
CONF_Cnr_ALLOW_CHARGING = "c{}_allow_charging"


AUTO_LOAD = ["switch"]

AllowChargingSwitch = twc_gen2_ns.class_('AllowCharging', cg.Component)

CONFIG_SCHEMA = {
    cv.GenerateID(CONF_TWC_GEN2_ID): cv.use_id(TwcGen2Controller),
    ** {
        cv.Optional(CONF_Cnr_ALLOW_CHARGING.format(i+1)): switch.switch_schema(class_=AllowChargingSwitch, icon=ICON_POWER, device_class=DEVICE_CLASS_SWITCH).extend({
            cv.GenerateID(): cv.declare_id(AllowChargingSwitch),
            cv.Optional(CONF_INITIAL_VALUE, True): bool,
        })
        for i in range(0,3)
    }
}

async def to_code(config):
    parent = await cg.get_variable(config[CONF_TWC_GEN2_ID])

    for i in range(0,3):
        if c := config.get(CONF_Cnr_ALLOW_CHARGING.format(i+1)):
            # Create a new Allow-Charging-switch
            sw = await switch.new_switch(c)
            cg.add(sw.set_controller(parent))
            cg.add(sw.set_idx(i))
            # Do we need to set an initial value?
            if c[CONF_INITIAL_VALUE]:
                cg.add(sw.set_initial_value(c[CONF_INITIAL_VALUE]))
            await cg.register_component(sw, c)
            # Make sure the main controller knows about this switch
            cg.add(parent.add_allow_charging_switch(i, sw))
