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

#include <uORB/topics/distance_sensor.h>

#include <uORB/topics/vehicle_command.h>
#include <uORB/Publication.hpp>
#include <lib/modes/standard_modes.hpp>
#include <commander/px4_custom_mode.h>

#include <uORB/topics/parameter_update.h>
#include <parameters/param.h>

bool number_of_occurences = 0;	//used to set loiter cmd only once
bool disable_hold = false;

int CollisionAvoidBasic::print_status()
{
	PX4_INFO("Running");
	return 0;
}

int CollisionAvoidBasic::custom_command(int argc, char *argv[])
{

if (!is_running()) {
	print_usage("not running");
	return 1;
}


return print_usage("unknown command");
}


int CollisionAvoidBasic::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("collision_avoid_basic",
				SCHED_DEFAULT,
				SCHED_PRIORITY_DEFAULT,
				1500,
				(px4_main_t)&run_trampoline,
				(char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

CollisionAvoidBasic *CollisionAvoidBasic::instantiate(int argc, char *argv[])
{
	bool disable_prot = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:de", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			disable_prot = true;
			PX4_WARN("disabling hold protection");
			disable_hold = true;
			break;

		case 'e':
			disable_prot = false;
			PX4_WARN("enabling hold protection");
			disable_hold = false;
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

	CollisionAvoidBasic *instance = new CollisionAvoidBasic(disable_prot);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

CollisionAvoidBasic::CollisionAvoidBasic(bool disable_prot)
	: ModuleParams(nullptr)
{
}

void CollisionAvoidBasic::run()
{
	PX4_INFO("STARTING");

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
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

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


			if(raw_1.current_distance <= getClosestDistance()){
				startProtect(1);
			}else if(raw_2.current_distance <= getClosestDistance()){
				startProtect(2);
			}else if(raw_3.current_distance <= getClosestDistance()){
				startProtect(3);
			}
			else{
				number_of_occurences = 0;
			}

		}

	}

	orb_unsubscribe(sensor_sub_fd_1);
	orb_unsubscribe(sensor_sub_fd_2);
	orb_unsubscribe(sensor_sub_fd_3);
}

float CollisionAvoidBasic::getClosestDistance(void){
	return closest_distance;
}

void CollisionAvoidBasic::parameters_update(bool force)	//basically sets closest distance
{
	param_t collis_dist_check = PARAM_INVALID;
	collis_dist_check = param_find("COLLIS_DIST_WARN");

	float myParam = 0;
	param_get(collis_dist_check, &myParam);

	closest_distance = myParam;

}

// void CollisionAvoidBasic::px4_send_log(char* msg_to_send)
// {
// 	// we need to advertise with a valid message
// 	log_message_s log_message{};
// 	log_message.severity = 6; // info
// 	strcpy((char *)log_message.text, msg_to_send);
// 	log_message.timestamp = hrt_absolute_time();
// 	orb_log_message_pub = orb_advertise(ORB_ID(log_message), &log_message);
// }

void CollisionAvoidBasic::startProtect(int instance_num){
	if(!disable_hold){
		if(number_of_occurences == 0){
			bool flightModeStatus = changeFlightModeToLoiter();
			switch(flightModeStatus){
				case 1:
					// px4_send_log("flight mode changed successfully, instance %d too close", instance_num);
					break;
				case 0:
					// px4_send_log("flight mode not changed");
					break;
			}
			number_of_occurences = 1;
		}else{
			PX4_INFO("hold mode set for instance %d", instance_num);
		}
	}
}

bool CollisionAvoidBasic::changeFlightModeToLoiter(void)		//called if too close or data is stale
{
	vehicle_command_s vcmd{};
	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	vcmd.param1 = 1.0f;
	vcmd.param2 = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
	vcmd.param3 = (float)PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
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


int CollisionAvoidBasic::print_usage(const char *reason)
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
PRINT_MODULE_USAGE_PARAM_FLAG('d', "disable hold protection", true);
PRINT_MODULE_USAGE_PARAM_FLAG('e', "enable hold protection", true);
// PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int collision_avoid_basic_main(int argc, char *argv[])
{
	return CollisionAvoidBasic::main(argc, argv);
}
