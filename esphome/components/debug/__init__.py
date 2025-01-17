import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.components import sensor, text_sensor
from esphome.const import (
    CONF_ID,
    CONF_DEVICE,
    CONF_FREE,
    CONF_FRAGMENTATION,
    CONF_BLOCK,
    CONF_LOOP_TIME,
    UNIT_MILLISECOND,
    UNIT_PERCENT,
    UNIT_BYTES,
    ICON_COUNTER,
    ICON_TIMER,
)

CODEOWNERS = ["@OttoWinter"]
DEPENDENCIES = ["logger"]

debug_ns = cg.esphome_ns.namespace("debug")
DebugComponent = debug_ns.class_("DebugComponent", cg.PollingComponent)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DebugComponent),
        cv.Optional(CONF_DEVICE): text_sensor.TEXT_SENSOR_SCHEMA.extend(
            {cv.GenerateID(): cv.declare_id(text_sensor.TextSensor)}
        ),
        cv.Optional(CONF_FREE): sensor.sensor_schema(UNIT_BYTES, ICON_COUNTER, 0),
        cv.Optional(CONF_BLOCK): sensor.sensor_schema(UNIT_BYTES, ICON_COUNTER, 0),
        cv.Optional(CONF_FRAGMENTATION): cv.All(
            cv.only_on_esp8266,
            cv.require_framework_version(esp8266_arduino=cv.Version(2, 5, 2)),
            sensor.sensor_schema(UNIT_PERCENT, ICON_COUNTER, 1),
        ),
        cv.Optional(CONF_LOOP_TIME): sensor.sensor_schema(
            UNIT_MILLISECOND, ICON_TIMER, 1
        ),
    }
).extend(cv.polling_component_schema("60s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_DEVICE in config:
        sens = cg.new_Pvariable(config[CONF_DEVICE][CONF_ID])
        await text_sensor.register_text_sensor(sens, config[CONF_DEVICE])
        cg.add(var.set_device_info_sensor(sens))

    if CONF_FREE in config:
        sens = await sensor.new_sensor(config[CONF_FREE])
        cg.add(var.set_free_sensor(sens))

    if CONF_BLOCK in config:
        sens = await sensor.new_sensor(config[CONF_BLOCK])
        cg.add(var.set_block_sensor(sens))

    if CONF_FRAGMENTATION in config:
        sens = await sensor.new_sensor(config[CONF_FRAGMENTATION])
        cg.add(var.set_fragmentation_sensor(sens))

    if CONF_LOOP_TIME in config:
        sens = await sensor.new_sensor(config[CONF_LOOP_TIME])
        cg.add(var.set_loop_time_sensor(sens))
