/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "TFMINIS.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <parameters/param.h>

void
TFMINIS::print_usage()
{
	PRINT_MODULE_USAGE_NAME("tfminis", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x10);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int tfminis_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = TFMINIS;
	BusCLIArguments cli{true, false};
	cli.rotation = (Rotation)distance_sensor_s::ROTATION_FORWARD_FACING;
	cli.default_i2c_frequency = 400000;  // Fast speed (400Khz)
	cli.i2c_address = TFMINIS_BASEADDR;

	param_t tfminis_0_add = PARAM_INVALID;
	param_t tfminis_1_add = PARAM_INVALID;
	param_t tfminis_2_add = PARAM_INVALID;
	tfminis_0_add = param_find("TFMINIS_0_ADDR");
	tfminis_1_add = param_find("TFMINIS_1_ADDR");
	tfminis_2_add = param_find("TFMINIS_2_ADDR");

	int32_t tfmini_addr0 = 0;
	int32_t tfmini_addr1 = 0;
	int32_t tfmini_addr2 = 0;

	param_get(tfminis_0_add, &tfmini_addr0);
	param_get(tfminis_1_add, &tfmini_addr1);
	param_get(tfminis_2_add, &tfmini_addr2);

	while ((ch = cli.getOpt(argc, argv, "A:")) != EOF) {		//seperate arg for instance number, address param can be ignored since
									// it will be overwritten by function below
		switch (ch) {
		case 'A':
			switch(atoi(cli.optArg())){
				case 0:								//instance 0 always faces backwards
					cli.i2c_address = tfmini_addr0;
					cli.rotation = (Rotation)distance_sensor_s::ROTATION_BACKWARD_FACING;
					break;

				case 1:								//instance 1 always faces left
					cli.i2c_address = tfmini_addr1;
					cli.rotation = (Rotation)distance_sensor_s::ROTATION_LEFT_FACING;
					break;

				case 2:								//instance 2 always faces right
					cli.i2c_address = tfmini_addr2;
					cli.rotation = (Rotation)distance_sensor_s::ROTATION_RIGHT_FACING;
					break;

				// case 3:								//instance 3 always faces upwards
				// 	cli.i2c_address = tfmini_addr0;
				// 	cli.rotation = (Rotation)distance_sensor_s::ROTATION_UPWARD_FACING;
				// 	break;

				// case 4:								//instance 4 always faces front
				// 	cli.i2c_address = tfmini_addr1;
				// 	cli.rotation = (Rotation)distance_sensor_s::ROTATION_FORWARD_FACING;
				// 	break;

				// case 5:								//instance 2 always faces down
				// 	cli.i2c_address = tfmini_addr2;
				// 	cli.rotation = (Rotation)distance_sensor_s::ROTATION_DOWNWARD_FACING;
				// 	break;
			}
		}
	}

	// while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
	// 	switch (ch) {
	// 	case 'R':
	// 		cli.rotation = (Rotation)atoi(cli.optArg());
	// 		PX4_INFO("rotation is %d", ch);
	// 		break;
	// 	}
	// }

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_TFMINIS);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
