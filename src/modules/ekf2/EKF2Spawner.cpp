/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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

#include "EKF2Spawner.hpp"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <lib/parameters/param.h>

using namespace time_literals;

int EKF2Spawner::spawn(int argc, char *argv[],
		      px4::atomic<EKF2 *> (&objects)[EKF2_MAX_INSTANCES]
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
		      , px4::atomic<EKF2Selector *> &ekf2_selector
#endif
		      )
{
	bool success = false;
	bool replay_mode = false;

	// Check for replay mode flag
	if (argc > 1 && !strcmp(argv[1], "-r")) {
		PX4_INFO("replay mode enabled");
		replay_mode = true;
	}

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	// Check if multi-instance mode is enabled
	int32_t sens_imu_mode = 1;
	param_get(param_find("SENS_IMU_MODE"), &sens_imu_mode);

	int32_t ekf2_instances = 0;
	param_get(param_find("EKF2_INST_NO"), &ekf2_instances);

	if (!replay_mode && sens_imu_mode == 0 && ekf2_instances > 0) {
		// Multi-instance mode with manual configuration
		success = spawnMultiInstances(objects, ekf2_selector);
	} else
#endif
	{
		// Single instance mode
		success = spawnSingleInstance(objects, replay_mode);
	}

	return success ? PX4_OK : PX4_ERROR;
}

bool EKF2Spawner::spawnSingleInstance(px4::atomic<EKF2 *> (&objects)[EKF2_MAX_INSTANCES], bool replay_mode)
{
	EKF2 *ekf2_inst = new EKF2(false, px4::wq_configurations::INS0, replay_mode);

	if (ekf2_inst) {
		objects[0].store(ekf2_inst);
		ekf2_inst->ScheduleNow();
		return true;
	}

	return false;
}

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
bool EKF2Spawner::spawnMultiInstances(px4::atomic<EKF2 *> (&objects)[EKF2_MAX_INSTANCES],
				      px4::atomic<EKF2Selector *> &ekf2_selector)
{
	// Get number of instances to create (1-6)
	int32_t instances_to_create = 0;
	param_get(param_find("EKF2_INST_NO"), &instances_to_create);
	instances_to_create = math::constrain(instances_to_create, (int32_t)1, (int32_t)6);

	PX4_INFO("Creating %" PRId32 " EKF2 instances with manual configuration", instances_to_create);

	// Start EKF2Selector
	if (ekf2_selector.load() == nullptr) {
		EKF2Selector *inst = new EKF2Selector();
		if (inst) {
			ekf2_selector.store(inst);
		} else {
			PX4_ERR("Failed to create EKF2 selector");
			return false;
		}
	}

	int instances_created = 0;
	const hrt_abstime time_started = hrt_absolute_time();
	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};

	// Wait for sensors and create instances
	while ((instances_created < instances_to_create)
	       && (vehicle_status_sub.get().arming_state != vehicle_status_s::ARMING_STATE_ARMED)
	       && ((hrt_elapsed_time(&time_started) < 30_s)
		   || (vehicle_status_sub.get().hil_state == vehicle_status_s::HIL_STATE_ON))) {

		vehicle_status_sub.update();

		for (int instance = 0; instance < instances_to_create; instance++) {
			// Skip if already created
			if (objects[instance].load() != nullptr) {
				continue;
			}

			// Get manual configuration for this instance from EKF2_X_IMU and EKF2_X_MAG params
			char imu_param[16], mag_param[16];
			snprintf(imu_param, sizeof(imu_param), "EKF2_%d_IMU", instance);
			snprintf(mag_param, sizeof(mag_param), "EKF2_%d_MAG", instance);

			int32_t imu_index = instance;  // Default to instance number
			int32_t mag_index = 0;         // Default to MAG 0

			param_get(param_find(imu_param), &imu_index);
			param_get(param_find(mag_param), &mag_index);

			uint8_t imu = static_cast<uint8_t>(imu_index);
			uint8_t mag = static_cast<uint8_t>(mag_index);

			// Check if sensors are ready
			uORB::SubscriptionData<vehicle_imu_s> vehicle_imu_sub{ORB_ID(vehicle_imu), imu};

#if defined(CONFIG_EKF2_MAGNETOMETER)
			uORB::SubscriptionData<vehicle_magnetometer_s> vehicle_mag_sub{ORB_ID(vehicle_magnetometer), mag};
			vehicle_mag_sub.update();

			if (!vehicle_imu_sub.advertised() || (!vehicle_mag_sub.advertised() && mag != 0)) {
				px4_usleep(10000);
				continue;
			}
#else
			if (!vehicle_imu_sub.advertised()) {
				px4_usleep(10000);
				continue;
			}
#endif

			// Create the instance
			EKF2 *ekf2_inst = new EKF2(true, px4::ins_instance_to_wq(imu), false);

			if (ekf2_inst && ekf2_inst->multi_init(imu, mag)) {
				int actual_instance = ekf2_inst->instance();

				if ((actual_instance >= 0) && (actual_instance < EKF2_MAX_INSTANCES)
				    && (objects[actual_instance].load() == nullptr)) {
					objects[actual_instance].store(ekf2_inst);
					instances_created++;

					PX4_INFO("EKF2 instance %d: IMU%u + MAG%u", actual_instance, imu, mag);

					ekf2_selector.load()->ScheduleNow();
				} else {
					PX4_ERR("Instance numbering problem: %d", actual_instance);
					delete ekf2_inst;
				}
			} else {
				PX4_ERR("Failed to init instance %d: IMU%u MAG%u", instance, imu, mag);
				delete ekf2_inst;
				px4_usleep(100000);
			}
		}

		if (instances_created < instances_to_create) {
			px4_usleep(10000);
		}
	}

	return instances_created > 0;
}
#endif // CONFIG_EKF2_MULTI_INSTANCE
