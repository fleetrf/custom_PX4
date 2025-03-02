/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/log.h>
#include <uORB/topics/distance_sensor.h>

extern "C" __EXPORT int collision_avoid_basic_main(int argc, char *argv[]);

int collision_avoid_basic_main(int argc, char *argv[])
{
	PX4_INFO("starting avoidance");
	int sensor_sub_fd_1 = orb_subscribe_multi(ORB_ID(distance_sensor), 0);
	int sensor_sub_fd_2 = orb_subscribe_multi(ORB_ID(distance_sensor), 1);
	int sensor_sub_fd_3 = orb_subscribe_multi(ORB_ID(distance_sensor), 2);
	while (true){
		struct distance_sensor_s raw_1;
		struct distance_sensor_s raw_2;
		struct distance_sensor_s raw_3;
		orb_copy(ORB_ID(distance_sensor), sensor_sub_fd_1, &raw_1);
		orb_copy(ORB_ID(distance_sensor), sensor_sub_fd_2, &raw_2);
		orb_copy(ORB_ID(distance_sensor), sensor_sub_fd_3, &raw_3);
		PX4_INFO("current distance of 1: %f, 2: %f, 3: %f", (double)raw_1.current_distance, (double)raw_2.current_distance, (double)raw_3.current_distance);
	}
	return OK;
}

