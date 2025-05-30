/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "adxl382.h"
#include <errno.h>

/* Example code to talk to a ADXL382 acceleromater sensor via SPI.

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefore SPI) cannot be used at 5v.

   You will need to use a level shifter on the SPI lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board and a generic adxl382 board, other
   boards may vary.

  
   GPIO2 (pin 4) I2C1_sda-> sda on adxl382 board
   GPIO3 (pin 5) I2C1_sCL-> SCL on adxl382 board
   3.3v (pin 36) -> VS & VDDIO pin on adxl382 board
   GND (pin 38)  -> GND on adxl382 board


   Note: SPI devices can have a number of different naming schemes for pins. See
   the Wikipedia page at https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
   for variations.

*/

// device has default bus address of 0x76

int ret;
uint8_t register_value;
uint8_t status0;
uint8_t fifo_status[2];
uint8_t fifo_data[36];
uint8_t set_fifo_entries = 0x0C;
uint16_t fifo_entries;
bool chID_enable = true; //FIFO channel id
uint8_t fifo_read_bytes;
struct adxl38x_fractional_val data_frac[15];
static char getaxis(uint8_t chID);
uint8_t count = 0;

void scan_i2c_bus(i2c_inst_t *i2c, const char *label) {
    printf("Scanning %s...\n", label);
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        uint8_t dummy;
        // Try to read a single byte from each address
        int result = i2c_read_blocking(i2c, addr, &dummy, 1, false);
        if (result >= 0) {
            printf("  Found device at 0x%02X\n", addr);
        }
    }
    printf("Done scanning %s.\n\n", label);
}
int main() {
    stdio_init_all();
    sleep_ms(1000);
    printf("USB port is successfully initialised\n");
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #warning i2c / adxl382_i2c example requires a board with I2C pins
        puts("Default I2C pins were not defined");
    return 0;
#else
    //set up led blink
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // useful information for picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
    bi_decl(bi_program_description("ADXL382 I2C example for the Raspberry Pi Pico"));

    i2c_init(i2c_default, 400000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    // gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    // gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

	// Scan the I2C bus for devices
	scan_i2c_bus(i2c_default, "i2c0");

    //Initialize ADXL382 by reading device ID
    ret = adxl38x_init();
	if (ret) {
		gpio_put(LED_PIN, 1);
		printf("Error: couldnt initialize ADXL382\n");
		goto error;
	} else {
		printf("ADXL is successfully initialised\n");
	}

        
    ret = adxl38x_soft_reset();
	if (ret == -EAGAIN) {
		printf("Error: Reset was not successful\n");
		goto error;
	}
	else if (ret)
		goto error;
	ret = adxl38x_set_range(ADXL38X_OP_MODE,ADXL38X_MASK_RANGE, ADXL382_RANGE_15G);
	if (ret)
		goto error;
	else
		printf("ADXL382 range is set to 15g\n");
	// FIFO sequence
	// Put the part in standby mode
	ret = adxl38x_set_to_standby();
	if (ret)
		goto error;
	

	// Set DIG_EN register to 0x78 (Enable XYZ axes and FIFO enable)
	printf("Enable XYZ axes and FIFO\n");
	register_value = 0x78;
	ret = write_register(ADXL38X_DIG_EN, 
					&register_value, 1);
	ret = read_register(ADXL38X_DIG_EN, 1, &register_value);
	printf("DIG_EN register value: 0x%02X\n", register_value);
	if (ret)
		goto error;

	// Set FIFO_CFG0 to 0x60 (Channel ID enable and FIFO stream mode)
	printf("Set FIFO_CFG0 to 0x60 (Channel ID enable and FIFO stream mode)\n");
	ret = adxl38x_accel_set_FIFO(set_fifo_entries,
				     false, ADXL38X_FIFO_STREAM, chID_enable, false);

	// Set INT0_MAP0 to 0x08 (FIFO_WATERMARK_INT0)
	printf("Set INT0_MAP0 to 0x08 (FIFO_WATERMARK_INT0)\n");
	register_value = 0x08;
	ret = write_register( ADXL38X_INT0_MAP0, &register_value, 1);
	if (ret)
        printf("Error: Error in setting INT0_MAP0\n");
		goto error;

	// Put the part in HP mode and read data when FIFO watermark pin is set
	printf("Set HP mode\n");
	ret = adxl38x_set_op_mode(ADXL38X_OP_MODE, ADXL38X_MASK_OP_MODE, ADXL38X_MODE_HP);
	if (ret) {
        printf("Error: Error in setting HP_MODE\n");
		goto error;
	}
	else
		printf("ADXL382 is in HP mode\n");

	printf("Starting watermark check\n");
	while (true) {
		
		// Read status to assert if FIFO_WATERMARK bit set
		ret = read_register(ADXL38X_STATUS0, 1, &status0);
		if (ret)
			goto error;
		printf("Status 0: %d\n", status0);
		ret = read_register(ADXL38X_FIFO_STATUS0, 2, fifo_status);
		if (ret)
			goto error;
		fifo_entries = (fifo_status[0] | ((uint16_t)fifo_status[1] << 8));
		fifo_entries = fifo_entries & 0x01ff;


		// Read FIFO status and data if FIFO_WATERMARK is set
		if (status0 & (1<<3)) {
			count++;
			gpio_put(LED_PIN, 0);
			printf(" FIFO_WATERMARK is set. Total fifo entries =  %d\n", fifo_entries);
			if (fifo_entries < set_fifo_entries)
				goto unmatch_error;

			// Read data from FIFO (can read at least upto 12 samples * 3 bytes (chID, data))
            if (chID_enable)
                fifo_read_bytes = 3;
			ret = read_register(ADXL38X_FIFO_DATA, fifo_read_bytes*fifo_entries, fifo_data);
			if (ret)
				goto error;

			// Parse Data for fist 4 (x,y,z) sets
			printf("Sample Count = %d\n", count*12);
			printf("First four entries (absolute values printed for magnitude between -1g & 1g):\n");
           

			for (int b = 0; b < 36; b += 3) {
				ret = adxl38x_data_raw_to_gees((fifo_data + b + 1), data_frac, ADXL382_RANGE_15G);
				if (ret)
					goto error;
				printf("%c : %lld.%07dg\n", getaxis(fifo_data[b]), data_frac->integer,
					labs(data_frac->fractional));
			}
		}
		
	}

error:
	if (ret)
		printf("Error: Error occurred!\n");
	else
		printf("The program has ended after successful execution\n");
	return 0;
unmatch_error:
	printf("Error: Number of entries in FIFO not matching the number set in FIFO config\n");
	return 0;


#endif
}

/***************************************************************************//**
 * @brief Assigns axis based on channel index
 *
 * @param chID         - Channel index
 *
 * @return ret         - Corresponding channel ID for channel index provided
*******************************************************************************/
static char getaxis(uint8_t chID)
{
	if (chID)
		return chID > 1 ? 'z' : 'y';
	return 'x';
}
