/*
 * Copyright (c) 2021, Adam Boardman
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "../api/mcp23017.h"
#include "../api/mcp23017_private.h"
#include <cstdio>

#ifdef MOCK_PICO_PI
#include "../test/pico_pi_mocks.h"
#else
#include "hardware/gpio.h"
#endif

mutex_t Mcp23017::i2c_mutex;

Mcp23017::Mcp23017(i2c_inst_t *_i2c, uint8_t _address) : i2c(_i2c), address(_address) {
	if (!mutex_is_initialized(&i2c_mutex)) 
        mutex_init(&i2c_mutex);
}

int Mcp23017::write_register(uint8_t reg, uint8_t value) const {
	uint8_t command[] = { reg, value };
	int result = i2c_write_blocking(i2c, address, command, 2, false);
	if (result == PICO_ERROR_GENERIC) {
		return result;
	}
	return PICO_ERROR_NONE;
}

int Mcp23017::read_register(uint8_t reg) const {
	uint8_t buffer = 0;
	int result;
	result = i2c_write_blocking(i2c, address,  &reg, 1, true);
	mcp_debug("i2c_write_blocking: %d\n",result);
	if (result == PICO_ERROR_GENERIC) {
		return result;
	}

	result = i2c_read_blocking(i2c, address, &buffer, 1, false);
	mcp_debug("i2c_read_blocking: %d, read: %d\n",result, buffer);
	if (result == PICO_ERROR_GENERIC)
		return result;

	return buffer;
}

int Mcp23017::write_dual_registers(uint8_t reg, int value) const {
	uint8_t command[] = {
			reg,
			static_cast<uint8_t>(value & 0xff),
			static_cast<uint8_t>((value>>8) & 0xff)
	};
	int result = i2c_write_blocking(i2c, address, command, 3, false);
	if (result == PICO_ERROR_GENERIC) {
		return result;
	}
	return PICO_ERROR_NONE;
}

int Mcp23017::read_dual_registers(uint8_t reg) const {
	uint8_t buffer[2]{};
	int result;
	result = i2c_write_blocking(i2c, address,  &reg, 1, true);
	mcp_debug("i2c_write_blocking: %d\n",result);
	if (result == PICO_ERROR_GENERIC) {
		return result;
	}

	result = i2c_read_blocking(i2c, address, buffer, 2, false);
	mcp_debug("i2c_read_blocking: %d, read: %d,%d\n",result, buffer[0], buffer[1]);
	if (result == PICO_ERROR_GENERIC)
		return result;

	return (buffer[1]<<8) + buffer[0];
}

int Mcp23017::setup(bool mirroring, bool polarity) const {
	mutex_enter_blocking(&i2c_mutex);
	int result;
	result = setup_bank_configuration(MCP23017_IOCONA, mirroring, polarity);
	if (result != 0) {
		mutex_exit(&i2c_mutex);
		return PICO_ERROR_GENERIC;
	}

	result = setup_bank_configuration(MCP23017_IOCONB, mirroring, polarity);
	mutex_exit(&i2c_mutex);
	return result;
}

int Mcp23017::setup_bank_configuration(int reg, bool mirroring, bool polarity) const {
	int ioConValue = 0;
	set_bit(ioConValue, MCP23017_IOCON_BANK_BIT, false);
	set_bit(ioConValue, MCP23017_IOCON_MIRROR_BIT, mirroring);
	set_bit(ioConValue, MCP23017_IOCON_SEQOP_BIT, false);
	set_bit(ioConValue, MCP23017_IOCON_DISSLW_BIT, false);
	set_bit(ioConValue, MCP23017_IOCON_HAEN_BIT, false);
	set_bit(ioConValue, MCP23017_IOCON_ODR_BIT, false);
	set_bit(ioConValue, MCP23017_IOCON_INTPOL_BIT, polarity);
	return write_register(reg, ioConValue);
}

int Mcp23017::get_last_interrupt_pin() const {
	int intFlag;

	mutex_enter_blocking(&i2c_mutex);
	intFlag = read_dual_registers(MCP23017_INTFA); //also MCP23017_INTFB
	mutex_exit(&i2c_mutex);
	mcp_debug("INTF %d",intFlag);
	if (intFlag != PICO_ERROR_GENERIC) {
		for (int i = 0; i < 16; i++) {
			if (is_bit_set(intFlag, i)) {
				return i;
			}
		}
	}

	return PICO_ERROR_GENERIC;
}

int Mcp23017::get_interrupt_values() const {
	mutex_enter_blocking(&i2c_mutex);
	int result = read_dual_registers(MCP23017_INTCAPA); //will include MCP23017_INTCAPB
	mutex_exit(&i2c_mutex);
	return result;
}

int Mcp23017::update_and_get_input_values() {
	mutex_enter_blocking(&i2c_mutex);
	int result = read_dual_registers(MCP23017_GPIOA); //will include MCP23017_GPIOB
	if (result != PICO_ERROR_GENERIC) {
		last_input = result;
		result = PICO_ERROR_NONE;
	}
	mutex_exit(&i2c_mutex);
	return result;
}

bool Mcp23017::get_last_input_pin_value(int pin) const {
	mutex_enter_blocking(&i2c_mutex);
	bool result = is_bit_set(last_input, pin);
	mutex_exit(&i2c_mutex);
	return result;
}

uint16_t Mcp23017::get_last_input_pin_values() const {
	mutex_enter_blocking(&i2c_mutex);
	uint16_t result = last_input;
	mutex_exit(&i2c_mutex);
	return result;
}

int Mcp23017::get_address() const {
	return address;
}

int Mcp23017::set_io_direction(int direction) const {
	mutex_enter_blocking(&i2c_mutex);
	int result = write_dual_registers(MCP23017_IODIRA, direction); //inc MCP23017_IODIRB
	mutex_exit(&i2c_mutex);
	return result;
}

int Mcp23017::set_pullup(int direction) const {
	mutex_enter_blocking(&i2c_mutex);
	int result = write_dual_registers(MCP23017_GPPUA, direction); //inc MCP23017_GPPUB, direction >> 8);
	mutex_exit(&i2c_mutex);
	return result;
}

int Mcp23017::set_interrupt_type(int compare_to_reg) const {
	mutex_enter_blocking(&i2c_mutex);
	int result = write_dual_registers(MCP23017_INTCONA, compare_to_reg); //inc MCP23017_INTCONB
	mutex_exit(&i2c_mutex);
	return result;
}

int Mcp23017::enable_interrupt(int enabled) const {
	mutex_enter_blocking(&i2c_mutex);
	int result = write_dual_registers(MCP23017_GPINTENA, enabled); //inc MCP23017_GPINTENB
	mutex_exit(&i2c_mutex);
	return result;
}

int Mcp23017::set_all_output_bits(int all_bits) {
	mutex_enter_blocking(&i2c_mutex);
	output = all_bits;
	int result = write_dual_registers(MCP23017_GPIOA, all_bits); //inc MCP23017_GPIOB
	mutex_exit(&i2c_mutex);
	return result;
}

void Mcp23017::set_output_bit_for_pin(int pin, bool set) {
	mutex_enter_blocking(&i2c_mutex);
	set_bit(output, pin, set);
	mutex_exit(&i2c_mutex);
}

bool Mcp23017::get_output_bit_for_pin(int pin) const {
	mutex_enter_blocking(&i2c_mutex);
	bool result = is_bit_set(output, pin);
	mutex_exit(&i2c_mutex);
	return result;
}

int Mcp23017::flush_output() const {
	mutex_enter_blocking(&i2c_mutex);
	int result = write_dual_registers(MCP23017_GPIOA, output); //inc MCP23017_GPIOB
	mutex_exit(&i2c_mutex);
	return result;
}
