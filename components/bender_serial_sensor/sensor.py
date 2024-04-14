import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID, CONF_UPDATE_INTERVAL

DEPENDENCIES = ['uart']

bender_serial_sensor_ns = cg.esphome_ns.namespace('bender_serial_sensor')
BenderSerialSensor = bender_serial_sensor_ns.class_('BenderSerialSensor', cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(BenderSerialSensor),
    cv.Optional(CONF_UPDATE_INTERVAL): cv.update_interval,
    # Assume we have a custom method to set sensitivity
    cv.Optional("sensitivity", default=10): cv.positive_int,
}).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield uart.register_uart_device(var, config)

    # Custom method setup, assume set_sensitivity exists in your C++ class
    cg.add(var.set_sensitivity(config["sensitivity"]))

    # Handle the update interval if necessary
    if CONF_UPDATE_INTERVAL in config:
        interval = config[CONF_UPDATE_INTERVAL]
        cg.add(var.set_update_interval(interval))
