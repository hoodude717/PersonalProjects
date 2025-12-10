#include <stdio.h>
#include "soc/reg_base.h" // DR_REG_GPIO_BASE, DR_REG_IO_MUX_BASE
#include "driver/rtc_io.h" // rtc_gpio_*
#include "pin.h"

// TODO: GPIO Matrix Registers - GPIO_OUT_REG, GPIO_OUT_W1TS_REG, ...
// NOTE: Remember to enclose the macro values in parenthesis, as below
#define GPIO_OUT_REG			(DR_REG_GPIO_BASE+0x04)
#define GPIO_OUT_W1TS_REG 		(DR_REG_GPIO_BASE+0x08)
#define GPIO_OUT_W1TC_REG 		(DR_REG_GPIO_BASE+0x0c)
#define GPIO_OUT1_REG			(DR_REG_GPIO_BASE+0x10)
#define GPIO_OUT1_W1TS_REG 		(DR_REG_GPIO_BASE+0x14)
#define GPIO_OUT1_W1TC_REG 		(DR_REG_GPIO_BASE+0x18)
#define GPIO_ENABLE_REG			(DR_REG_GPIO_BASE+0x20)
#define GPIO_ENABLE_W1TS_REG	(DR_REG_GPIO_BASE+0x24)
#define GPIO_ENABLE_W1TC_REG	(DR_REG_GPIO_BASE+0x28)
#define GPIO_ENABLE1_REG		(DR_REG_GPIO_BASE+0x2c)
#define GPIO_ENABLE1_W1TS_REG	(DR_REG_GPIO_BASE+0x30)
#define GPIO_ENABLE1_W1TC_REG	(DR_REG_GPIO_BASE+0x34)
#define GPIO_IN_REG 			(DR_REG_GPIO_BASE+0x3c)
#define GPIO_IN1_REG 			(DR_REG_GPIO_BASE+0x40)


// TODO: IO MUX Registers
// HINT: Add DR_REG_IO_MUX_BASE with PIN_MUX_REG_OFFSET[n]
#define IO_MUX_REG(n) 		(DR_REG_IO_MUX_BASE + PIN_MUX_REG_OFFSET[n])

// TODO: IO MUX Register Fields - FUN_WPD, FUN_WPU,GPIO_PIN_BASE_REG + (n*
#define FUN_WPD 	7
#define FUN_WPU 	8
#define FUN_IE 		9
#define FUN_DRV0 	10
#define FUN_DRV1 	11
#define MCU_SEL0 	12
#define MCU_SEL1 	13 
#define MCU_SEL2 	14 

// REset the IO_MUX_n_REG to default pull up settings
#define IO_MUX_RST_MCU_SEL(n) (REG_SET_BIT(IO_MUX_REG(n), MCU_SEL1))
#define IO_MUX_RST_FUN_WPU(n) (REG_SET_BIT(IO_MUX_REG(n), FUN_WPU))
#define IO_MUX_RST_FUN_DRV(n) (REG_SET_BIT(IO_MUX_REG(n), FUN_DRV1))
#define IO_MUX_RST 0x2900


#define GPIO_FUNC_BASE_OUT_SEL_CFG_REG 0x3FF44530
#define GPIO_FUNC_OUT_SEL_CFG_REG(n) 	REG((GPIO_FUNC_BASE_OUT_SEL_CFG_REG + (n*0x04)))
#define GPIO_FUNC_OUT_SEL_RST(n) GPIO_FUNC_OUT_SEL_CFG_REG(n) = 0x100
//GPIO PIN REGISTERS
#define GPIO_PIN_PAD_DRIVER 2
#define GPIO_PIN_BASE_REG 0x3FF44088
#define GPIO_PIN_REG(n) 	REG((GPIO_PIN_BASE_REG + (n*0x04)))
#define GPIO_PIN_CLR(n) 	(GPIO_PIN_REG(n)=0)

#define REG(r) (*(volatile uint32_t *)(r))
#define REG_BITS 32
// TODO: Finish these macros. HINT: Use the REG() macro.
#define REG_SET_BIT(r,b) (REG(r) |= (1 << b))
#define REG_CLR_BIT(r,b) (REG(r) &= ~(1 << b))
#define REG_GET_BIT(r,b) ((REG(r) & (1 << b)) != 0)

// Gives byte offset of IO_MUX Configuration Register
// from base address DR_REG_IO_MUX_BASE
static const uint8_t PIN_MUX_REG_OFFSET[] = {
    0x44, 0x88, 0x40, 0x84, 0x48, 0x6c, 0x60, 0x64, // pin  0- 7
    0x68, 0x54, 0x58, 0x5c, 0x34, 0x38, 0x30, 0x3c, // pin  8-15
    0x4c, 0x50, 0x70, 0x74, 0x78, 0x7c, 0x80, 0x8c, // pin 16-23
    0x90, 0x24, 0x28, 0x2c, 0xFF, 0xFF, 0xFF, 0xFF, // pin 24-31
    0x1c, 0x20, 0x14, 0x18, 0x04, 0x08, 0x0c, 0x10, // pin 32-39
};


// Reset the configuration of a pin to not be an input or an output.
// Pull-up is enabled so the pin does not float.
int32_t pin_reset(pin_num_t pin)
{
	if (rtc_gpio_is_valid_gpio(pin)) { // hand-off work to RTC subsystem
		rtc_gpio_deinit(pin);
		rtc_gpio_pullup_en(pin);
		rtc_gpio_pulldown_dis(pin);
	}
	// Reset GPIO_PINn_REG: All fields zero
	GPIO_PIN_CLR(pin);

	//Reset GPIO_FUNCn_OUT_SEL_CFG_REG: GPIO_FUNCn_OUT_SEL=0x100
	GPIO_FUNC_OUT_SEL_RST(pin);
	//Reset IO_MUX_x_REG: MCU_SEL=2, FUN_DRV=2, FUN_WPU=1
	REG(IO_MUX_REG(pin)) = IO_MUX_RST;
	// NOTE: By default, pin should not float, save power with FUN_WPU=1

	// Now that the pin is reset, set the output level to zero
	return pin_set_level(pin, 0);
}

// Enable or disable a pull-up on the pin.
int32_t pin_pullup(pin_num_t pin, bool enable)
{
	if (rtc_gpio_is_valid_gpio(pin)) { // hand-off work to RTC subsystem
		if (enable) return rtc_gpio_pullup_en(pin);
		else return rtc_gpio_pullup_dis(pin);
	}
	//Set or clear the FUN_WPU bit in an IO_MUX register
	if (enable) {
		REG_SET_BIT(IO_MUX_REG(pin), FUN_WPU);
	}
	else {
		REG_CLR_BIT(IO_MUX_REG(pin), FUN_WPU);
	}

	return 0;
}

// Enable or disable a pull-down on the pin.
int32_t pin_pulldown(pin_num_t pin, bool enable)
{
	if (rtc_gpio_is_valid_gpio(pin)) { // hand-off work to RTC subsystem
		if (enable) return rtc_gpio_pulldown_en(pin);
		else return rtc_gpio_pulldown_dis(pin);
	}
	// Set or clear the FUN_WPD bit in an IO_MUX register
	if (enable) {
		REG_SET_BIT(IO_MUX_REG(pin), FUN_WPD);
	}
	else {
		REG_CLR_BIT(IO_MUX_REG(pin), FUN_WPD);
	}

	return 0;
}

// Enable or disable the pin as an input signal.
int32_t pin_input(pin_num_t pin, bool enable)
{
	//Set or clear the FUN_IE bit in an IO_MUX register
	if(enable) {
		REG_SET_BIT(IO_MUX_REG(pin), FUN_IE);
	}
	else {
		REG_CLR_BIT(IO_MUX_REG(pin), FUN_IE);
	}
	return 0;
}
// Enable or disable the pin as an output signal.
int32_t pin_output(pin_num_t pin, bool enable)
{
	//Set or clear the I/O pin bit in the ENABLE or ENABLE1 register
	if (enable){
		(pin < REG_BITS) ? REG_SET_BIT(GPIO_ENABLE_W1TS_REG, pin) : REG_SET_BIT(GPIO_ENABLE1_W1TS_REG, (pin - REG_BITS));
	} 
	else {
		(pin < REG_BITS) ? REG_SET_BIT(GPIO_ENABLE_W1TC_REG, pin) : REG_SET_BIT(GPIO_ENABLE1_W1TC_REG, (pin - REG_BITS));
	}
	return 0;
}

// Enable or disable the pin as an open-drain signal.
int32_t pin_odrain(pin_num_t pin, bool enable)
{
	//Set or clear the PAD_DRIVER bit in a PIN register
	if(enable) {
		REG_SET_BIT(GPIO_PIN_REG(pin), GPIO_PIN_PAD_DRIVER);
	}
	else {
		REG_CLR_BIT(GPIO_PIN_REG(pin), GPIO_PIN_PAD_DRIVER);
	}
	return 0;
}

// Sets the output signal level if the pin is configured as an output.
int32_t pin_set_level(pin_num_t pin, int32_t level)
{
	//Set or clear the I/O pin bit in the OUT or OUT1 register
	if (level){
		(pin < REG_BITS) ? REG_SET_BIT(GPIO_OUT_W1TS_REG, pin) : REG_SET_BIT(GPIO_OUT1_W1TS_REG, (pin - REG_BITS));
	} 
	else {
		(pin < REG_BITS) ? REG_SET_BIT(GPIO_OUT_W1TC_REG, pin) : REG_SET_BIT(GPIO_OUT1_W1TC_REG, (pin - REG_BITS));
	}
	return level;
}

// Gets the input signal level if the pin is configured as an input.
int32_t pin_get_level(pin_num_t pin)
{
	//Get the I/O pin bit from the IN or IN1 register
	return (pin < REG_BITS) ? REG_GET_BIT(GPIO_IN_REG, pin) : REG_GET_BIT(GPIO_IN1_REG, (pin-REG_BITS));

}

// Get the value of the input registers, one pin per bit.
// The two 32-bit input registers are concatenated into a uint64_t.
uint64_t pin_get_in_reg(void)
{
	//Read the IN and IN1 registers, return the concatenated values
	uint64_t inReg = 0;
	inReg = REG(GPIO_IN1_REG); 
	inReg = inReg << REG_BITS;
	inReg += REG(GPIO_IN_REG);

	return inReg; 
}

// Get the value of the output registers, one pin per bit.
// The two 32-bit output registers are concatenated into a uint64_t.
uint64_t pin_get_out_reg(void)
{
	//Read the OUT and OUT1 registers, return the concatenated values
	uint64_t outReg = 0;
	outReg = REG(GPIO_OUT1_REG); 
	outReg = outReg << REG_BITS;
	outReg += REG(GPIO_OUT_REG);
	return outReg;
}
