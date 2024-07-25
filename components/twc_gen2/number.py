import esphome.codegen as cg
from esphome import cpp_generator as cppg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_INITIAL_VALUE,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_NAME,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_SWITCH,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_POWER,
    UNIT_AMPERE
)

from . import (
    twc_gen2_ns,
    TwcGen2Controller
)

CONF_TWC_GEN2_ID = "twc_gen2_id"
CONF_GLOBAL_LIMIT = "global_limit"


AUTO_LOAD = ["number"]

CurrentLimitNumber = twc_gen2_ns.class_('CurrentLimit', cg.Component)


def validate_min_max_value(config):
    if config[CONF_MIN_VALUE] > config[CONF_MAX_VALUE]:
        raise cv.Invalid("min_value must not be greater than max_value")
    return config


CONFIG_SCHEMA = {
    cv.GenerateID(CONF_TWC_GEN2_ID): cv.use_id(TwcGen2Controller),

    cv.Optional(CONF_GLOBAL_LIMIT): cv.All(
        number.number_schema(class_=CurrentLimitNumber,device_class=DEVICE_CLASS_CURRENT,unit_of_measurement=UNIT_AMPERE
        ).extend({
            cv.GenerateID(): cv.declare_id(CurrentLimitNumber),
            #cv.Optional(CONF_INITIAL_VALUE, 6): cv.int_range(6, 32),
            cv.Optional(CONF_MIN_VALUE, 6): cv.int_range(6, 32),
            cv.Required(CONF_MAX_VALUE): cv.int_range(6, 32),
        }),
        validate_min_max_value
    )

}

async def to_code(config):
    parent = await cg.get_variable(config[CONF_TWC_GEN2_ID])

    if current_limit_config := config.get(CONF_GLOBAL_LIMIT):
        nr = await number.new_number(
            current_limit_config,
            step=1,
            min_value=current_limit_config[CONF_MIN_VALUE],
            max_value=current_limit_config[CONF_MAX_VALUE],
            )
        cg.add(nr.set_controller(parent))
        #cg.add(nr.set_initial_value(parent.global_limit))
        await cg.register_component(nr, current_limit_config)
        cg.add(parent.set_global_limit_number(nr))

