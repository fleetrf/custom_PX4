/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "collision_avoid_basic.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/distance_sensor.h>

//check header requirements for flight mode change
#include <uORB/topics/vehicle_command.h>
#include <uORB/Publication.hpp>
#include <lib/modes/standard_modes.hpp>


int TemplateModule::print_status()
{
	PX4_INFO("Running");
	return 0;
}

int TemplateModule::custom_command(int argc, char *argv[])
{

if (!is_running()) {
	print_usage("not running");
	return 1;
}


return print_usage("unknown command");
}


int TemplateModule::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("collision_avoid_basic",
				SCHED_DEFAULT,
				SCHED_PRIORITY_DEFAULT,
				1024,
				(px4_main_t)&run_trampoline,
				(char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

TemplateModule *TemplateModule::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	TemplateModule *instance = new TemplateModule(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

TemplateModule::TemplateModule(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void TemplateModule::run()
{
	int sensor_sub_fd_1 = orb_subscribe_multi(ORB_ID(distance_sensor), 0);
	int sensor_sub_fd_2 = orb_subscribe_multi(ORB_ID(distance_sensor), 1);
	int sensor_sub_fd_3 = orb_subscribe_multi(ORB_ID(distance_sensor), 2);

	px4_pollfd_struct_t fds[1];
	fds[0].fd = sensor_sub_fd_1;
	fds[0].events = POLLIN;

	// initialize parameters
	parameters_update(true);

	while (!should_exit()) {

		// wait for up to 50ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 50);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			struct distance_sensor_s raw_1;
			struct distance_sensor_s raw_2;
			struct distance_sensor_s raw_3;
			orb_copy(ORB_ID(distance_sensor), sensor_sub_fd_1, &raw_1);
			orb_copy(ORB_ID(distance_sensor), sensor_sub_fd_2, &raw_2);
			orb_copy(ORB_ID(distance_sensor), sensor_sub_fd_3, &raw_3);

			PX4_INFO("current distance of 1: %f, 2: %f, 3: %f", (double)raw_1.current_distance, (double)raw_2.current_distance, (double)raw_3.current_distance);

			if(raw_1.current_distance <= 1 || raw_2.current_distance <= 1 || raw_3.current_distance <= 1){	//temporary function... tends to crash QGC
				bool flightModeChangeStatus = changeFlightMode(vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM);

				switch (flightModeChangeStatus){
					case 0:
						PX4_INFO("flight mode change unsuccessful");
						break;

					case 1:
						PX4_INFO("flight mode change successful");
						break;
				}

			}

		}

		parameters_update();
	}

	orb_unsubscribe(sensor_sub_fd_1);
	orb_unsubscribe(sensor_sub_fd_2);
	orb_unsubscribe(sensor_sub_fd_3);
}

bool TemplateModule::changeFlightMode(const uint32_t cmd)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = 0.0f;
	vcmd.param2 = 0.0f;
	vcmd.param3 = 0.0f;
	vcmd.param4 = 0.0f;
	vcmd.param5 = 0.0f;
	vcmd.param6 = 0.0f;
	vcmd.param7 = NAN;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}

void TemplateModule::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int TemplateModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start

)DESCR_STR");

PRINT_MODULE_USAGE_NAME("module", "collision_avoid_basic");
PRINT_MODULE_USAGE_COMMAND("start");
// PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
// PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int collision_avoid_basic_main(int argc, char *argv[])
{
	return TemplateModule::main(argc, argv);
}
