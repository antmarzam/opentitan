// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Device table API auto-generated by `dtgen`
<%
    from topgen.lib import Name
    from dtgen.helper import Extension

    device_name = helper.ip.name

    include_guard = "OPENTITAN_DT_{}_H_".format(device_name.upper())
%>\

#ifndef ${include_guard}
#define ${include_guard}

#include "dt_api.h"
#include <stdint.h>

/**
 * List of instances.
 */
${helper.inst_enum.render()}

/**
 * List of register blocks.
 *
 * Register blocks are guaranteed to start at 0 and to be consecutively numbered.
 */
${helper.reg_block_enum.render()}

/** Primary register block (associated with the "primary" set of registers that control the IP). */
<%
  default_reg_block_name = (helper.reg_block_enum.name + Name(["primary"])).as_c_enum()
  default_reg_block_value = (helper.reg_block_enum.name + Name.from_snake_case(helper.default_node)).as_c_enum()
%>\
static const ${helper.reg_block_enum.name.as_c_type()} ${default_reg_block_name} = ${default_reg_block_value};

% if helper.has_irqs():
/**
 * List of IRQs.
 *
 * IRQs are guaranteed to be numbered consecutively from 0.
 */
${helper.irq_enum.render()}

% endif
% if helper.has_alerts() and helper.has_alert_handler():
/**
 * List of Alerts.
 *
 * Alerts are guaranteed to be numbered consecutively from 0.
 */
${helper.alert_enum.render()}

% endif
% if helper.has_clocks():
/**
 * List of clock ports.
 *
 * Clock ports are guaranteed to be numbered consecutively from 0.
 */
${helper.clock_enum.render()}

% endif
% if helper.has_periph_io():
/**
 * List of peripheral I/O.
 *
 * Peripheral I/O are guaranteed to be numbered consecutively from 0.
 */
${helper.periph_io_enum.render()}

% endif
% if helper.has_wakeups():
/**
 * List of wakeups.
 *
 * Wakeups are guaranteed to be numbered consecutively from 0.
 */
${helper.wakeup_enum.render()}

% endif

/**
 * Get the ${device_name} instance from an instance ID
 *
 * For example, `dt_uart_from_instance_id(kDtInstanceIdUart3) == kDtUart3`.
 *
 * @param dt Instance ID.
 * @return A ${device_name} instance.
 *
 * NOTE This function only makes sense if the instance ID has device type ${device_name},
 * otherwise the returned value is unspecified.
 */
dt_${device_name}_t dt_${device_name}_from_instance_id(dt_instance_id_t inst_id);

/**
 * Get the instance ID of an instance.
 *
 * @param dt Instance of ${device_name}.
 * @return The instance ID of that instance.
 */
dt_instance_id_t dt_${device_name}_instance_id(dt_${device_name}_t dt);

/**
 * Get the register base address of an instance.
 *
 * @param dt Instance of ${device_name}.
 * @param reg_block The register block requested.
 * @return The register base address of the requested block.
 */
uint32_t dt_${device_name}_reg_block(
    dt_${device_name}_t dt,
    dt_${device_name}_reg_block_t reg_block);

/**
 * Get the primary register base address of an instance.
 *
 * This is just a convenience function, equivalent to
 * `dt_${device_name}_reg_block(dt, ${default_reg_block_value})`
 *
 * @param dt Instance of ${device_name}.
 * @return The register base address of the primary register block.
 */
static inline uint32_t dt_${device_name}_primary_reg_block(
    dt_${device_name}_t dt) {
  return dt_${device_name}_reg_block(dt, ${default_reg_block_value});
}

% if helper.has_irqs():
/**
 * Get the PLIC ID of a ${device_name} IRQ for a given instance.
 *
 * If the instance is not connected to the PLIC, this function
 * will return `kDtPlicIrqIdNone`.
 *
 * @param dt Instance of ${device_name}.
 * @param irq_type A ${device_name} IRQ.
 * @return The PLIC ID of the IRQ of this instance.
 */
dt_plic_irq_id_t dt_${device_name}_irq_to_plic_id(
    dt_${device_name}_t dt,
    dt_${device_name}_irq_t irq);

/**
 * Convert a global IRQ ID to a local ${device_name} IRQ type.
 *
 * @param dt Instance of ${device_name}.
 * @param irq A PLIC ID that belongs to this instance.
 * @return The ${device_name} IRQ, or `${helper.irq_enum.name.as_c_enum()}Count`.
 *
 * NOTE This function assumes that the PLIC ID belongs to the instance
 * of ${device_name} passed in parameter. In other words, it must be the case that
 * `dt_${device_name}_instance_id(dt) == dt_plic_id_to_instance_id(irq)`. Otherwise, this function
 * will return `${helper.irq_enum.name.as_c_enum()}Count`.
 */
dt_${device_name}_irq_t dt_${device_name}_irq_from_plic_id(
    dt_${device_name}_t dt,
    dt_plic_irq_id_t irq);

%endif

% if helper.has_alerts() and helper.has_alert_handler():
/**
 * Get the alert ID of a ${device_name} alert for a given instance.
 *
 * NOTE This function only makes sense if the instance is connected to the Alert Handler. For any
 * instances where the instance is not connected, the return value is unspecified.
 *
 * @param dt Instance of ${device_name}.
 * @param alert_type A ${device_name} alert.
 * @return The Alert Handler alert ID of the alert of this instance.
 */
dt_alert_id_t dt_${device_name}_alert_to_alert_id(
    dt_${device_name}_t dt,
    dt_${device_name}_alert_t alert);

/**
 * Convert a global alert ID to a local ${device_name} alert type.
 *
 * @param dt Instance of ${device_name}.
 * @param alert A global alert ID that belongs to this instance.
 * @return The ${device_name} alert, or `${helper.alert_enum.name.as_c_enum()}Count`.
 *
 * NOTE This function assumes that the global alert ID belongs to the
 * instance of ${device_name} passed in parameter. In other words, it must be the case
 * that `dt_${device_name}_instance_id(dt) == dt_alert_id_to_instance_id(alert)`. Otherwise,
 * this function will return `${helper.alert_enum.name.as_c_enum()}Count`.
 */
dt_${device_name}_alert_t dt_${device_name}_alert_from_alert_id(
    dt_${device_name}_t dt,
    dt_alert_id_t alert);

%endif

% if helper.has_periph_io():
/**
 * Get the peripheral I/O description of an instance.
 *
 * @param dt Instance of ${device_name}.
 * @param sig Requested peripheral I/O.
 * @return Description of the requested peripheral I/O for this instance.
 */
dt_periph_io_t dt_${device_name}_periph_io(
    dt_${device_name}_t dt,
    dt_${device_name}_periph_io_t sig);
% endif

% if helper.has_clocks():
/**
 * Get the clock signal connected to a clock port of an instance.
 *
 * @param dt Instance of ${device_name}.
 * @param sig Clock port.
 * @return Clock signal.
 */
dt_clock_t dt_${device_name}_clock(
    dt_${device_name}_t dt,
    dt_${device_name}_clock_t clk);
% endif

## Extension
${helper.render_extension(Extension.DtIpPos.HeaderEnd)}

#endif  // ${include_guard}
