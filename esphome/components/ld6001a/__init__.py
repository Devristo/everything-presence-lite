import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_THROTTLE

DEPENDENCIES = ["uart"]
MULTI_CONF = True

ld6001a_ns = cg.esphome_ns.namespace("ld6001a")
LD6001AComponent = ld6001a_ns.class_("LD6001AComponent", cg.PollingComponent, uart.UARTDevice)

CONF_LD6001A_ID = "ld6001a_id"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LD6001AComponent),
            cv.Optional(CONF_THROTTLE, default="1000ms"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(milliseconds=1)),
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.polling_component_schema("500ms"))
)

LD6001ABaseSchema = cv.Schema(
    {
        cv.GenerateID(CONF_LD6001A_ID): cv.use_id(LD6001AComponent),
    },
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ld6001a",
    require_tx=True,
    require_rx=True,
    parity="EVEN",
    stop_bits=1,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_throttle(config[CONF_THROTTLE]))
