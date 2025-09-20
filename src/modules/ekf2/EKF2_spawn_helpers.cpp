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

/**
 * @file EKF2_spawn_helpers.cpp
 * Helper functions for EKF2 task spawning with manual sensor assignment.
 */

#include "EKF2.hpp"

using namespace time_literals;

// ==== Configuration Functions ====

EKF2::SpawnConfig EKF2::parseSpawnArguments(int argc, char *argv[])
{
	SpawnConfig config;

	if (argc > 1 && !strcmp(argv[1], "-r")) {
		PX4_INFO("replay mode enabled");
		config.replay_mode = true;
	}

	return config;
}

bool EKF2::configureMultiInstance(SpawnConfig &config)
{
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	int32_t sens_imu_mode = 1;
	param_get(param_find("SENS_IMU_MODE"), &sens_imu_mode);

	if (sens_imu_mode != 0) {
		return false; // Single mode - ekf selector requires SENS_IMU_MODE = 0
	}

	config.multi_mode = true;

	// Get number of instances to create
	param_t param_inst_no = param_find("EKF2_INST_NO");
	if (param_inst_no != PARAM_INVALID) {
		param_get(param_inst_no, &config.instance_count);
	} else {
		config.instance_count = 1; // Default value
		PX4_WARN("EKF2_INST_NO parameter not found, using default value 1");
	}

	// Limit to maximum of 6 instances as specified
	if (config.instance_count < 1 || config.instance_count > 6) {
		const int32_t limited_instances = math::constrain(config.instance_count,
		                                                 static_cast<int32_t>(1),
		                                                 static_cast<int32_t>(6));
		PX4_WARN("EKF2_INST_NO limited %" PRId32 " -> %" PRId32,
		         config.instance_count, limited_instances);
		param_set_no_notification(param_find("EKF2_INST_NO"), &limited_instances);
		config.instance_count = limited_instances;
	}

	// Configure IMU instances
	if (!configureImuInstances(config)) {
		return false;
	}

	// Configure magnetometer instances
	configureMagInstances(config);

	return true;
#else
	return false; // Multi-instance not compiled in
#endif // CONFIG_EKF2_MULTI_INSTANCE
}

bool EKF2::configureImuInstances(SpawnConfig &config)
{
	// IMUs (1 - MAX_NUM_IMUS supported)
	param_get(param_find("EKF2_MULTI_IMU"), &config.imu_instances);

	if (config.imu_instances < 1 || config.imu_instances > MAX_NUM_IMUS) {
		const int32_t imu_instances_limited = math::constrain(config.imu_instances,
		                                                     static_cast<int32_t>(1),
		                                                     static_cast<int32_t>(MAX_NUM_IMUS));
		PX4_WARN("EKF2_MULTI_IMU limited %" PRId32 " -> %" PRId32,
		         config.imu_instances, imu_instances_limited);
		param_set_no_notification(param_find("EKF2_MULTI_IMU"), &imu_instances_limited);
		config.imu_instances = imu_instances_limited;
	}

	return true;
}

void EKF2::configureMagInstances(SpawnConfig &config)
{
#if defined(CONFIG_EKF2_MAGNETOMETER)
	int32_t sens_mag_mode = 1;
	const param_t param_sens_mag_mode = param_find("SENS_MAG_MODE");
	param_get(param_sens_mag_mode, &sens_mag_mode);

	if (sens_mag_mode == 0) {
		const param_t param_ekf2_mult_mag = param_find("EKF2_MULTI_MAG");
		param_get(param_ekf2_mult_mag, &config.mag_instances);

		// Mags (1 - MAX_NUM_MAGS supported)
		if (config.mag_instances > MAX_NUM_MAGS) {
			const int32_t mag_instances_limited = math::constrain(config.mag_instances,
			                                                     static_cast<int32_t>(1),
			                                                     static_cast<int32_t>(MAX_NUM_MAGS));
			PX4_WARN("EKF2_MULTI_MAG limited %" PRId32 " -> %" PRId32,
			         config.mag_instances, mag_instances_limited);
			param_set_no_notification(param_ekf2_mult_mag, &mag_instances_limited);
			config.mag_instances = mag_instances_limited;

		} else if (config.mag_instances <= 1) {
			// properly disable multi-magnetometer at sensors hub level
			PX4_WARN("EKF2_MULTI_MAG disabled, resetting SENS_MAG_MODE");

			// re-enable at sensors level
			sens_mag_mode = 1;
			param_set(param_sens_mag_mode, &sens_mag_mode);

			config.mag_instances = 1;
		}

	} else {
		config.mag_instances = 1;
	}
#else
	config.mag_instances = 1;
#endif // CONFIG_EKF2_MAGNETOMETER
}

// ==== Instance Creation Functions ====

bool EKF2::initializeEKF2Selector()
{
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	// Access the external static variable declared in EKF2.cpp
	extern px4::atomic<EKF2Selector *> _ekf2_selector;

	// Start EKF2Selector if it's not already running
	if (_ekf2_selector.load() == nullptr) {
		EKF2Selector *inst = new EKF2Selector();

		if (inst) {
			_ekf2_selector.store(inst);
			return true;
		} else {
			PX4_ERR("Failed to create EKF2 selector");
			return false;
		}
	}
	return true;
#else
	return false;
#endif // CONFIG_EKF2_MULTI_INSTANCE
}

int EKF2::createSingleInstance(bool replay_mode)
{
	// Access the external static variable declared in EKF2.cpp
	extern px4::atomic<EKF2 *> _objects[EKF2_MAX_INSTANCES];

	// Launch regular single instance
	EKF2 *ekf2_inst = new EKF2(false, px4::wq_configurations::INS0, replay_mode);

	if (ekf2_inst) {
		_objects[0].store(ekf2_inst);
		ekf2_inst->ScheduleNow();
		return PX4_OK;
	}

	return PX4_ERROR;
}

#if defined(CONFIG_EKF2_MULTI_INSTANCE)

// ==== Multi-Instance Functions with Manual Sensor Assignment ====

int EKF2::createMultipleInstances(const SpawnConfig &config)
{
	// Access external variables
	extern px4::atomic<EKF2 *> _objects[EKF2_MAX_INSTANCES];
	extern px4::atomic<EKF2Selector *> _ekf2_selector;

	const hrt_abstime time_started = hrt_absolute_time();
	int instances_created = 0;

	// Get vehicle status for arming check
	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};

	PX4_INFO("Creating %" PRId32 " EKF2 instances using manual sensor assignments",
	         config.instance_count);

	// Create the requested number of instances (up to 6)
	for (int instance = 0; instance < config.instance_count && instance < 6; instance++) {

		if (isVehicleArmed(vehicle_status_sub) &&
		    !((hrt_elapsed_time(&time_started) < 30_s) || isHilModeActive(vehicle_status_sub))) {
			break;
		}

		vehicle_status_sub.update();

		// Get manual sensor assignments for this instance
		char imu_param_name[16], mag_param_name[16];
		snprintf(imu_param_name, sizeof(imu_param_name), "EKF2_%d_IMU", instance);
		snprintf(mag_param_name, sizeof(mag_param_name), "EKF2_%d_MAG", instance);

		// Read the specific IMU and MAG parameters for this instance
		int32_t imu_idx = instance;  // Default fallback
		int32_t mag_idx = 0;         // Default fallback

		param_t param_imu = param_find(imu_param_name);
		param_t param_mag = param_find(mag_param_name);

		if (param_imu != PARAM_INVALID) {
			param_get(param_imu, &imu_idx);
		} else {
			PX4_WARN("Parameter %s not found, using default IMU %d", imu_param_name, instance);
		}

		if (param_mag != PARAM_INVALID) {
			param_get(param_mag, &mag_idx);
		} else {
			PX4_WARN("Parameter %s not found, using default MAG 0", mag_param_name);
		}

		// Validate sensor indices
		if (imu_idx < 0 || imu_idx >= config.imu_instances) {
			PX4_ERR("Instance %d: Invalid IMU index %" PRId32 " (available: 0-%" PRId32 ")",
			        instance, imu_idx, config.imu_instances - 1);
			continue;
		}

		if (mag_idx < 0 || mag_idx >= config.mag_instances) {
			PX4_ERR("Instance %d: Invalid MAG index %" PRId32 " (available: 0-%" PRId32 ")",
			        instance, mag_idx, config.mag_instances - 1);
			continue;
		}

		// Check if sensors are ready
		if (!isSensorDataValid(static_cast<uint8_t>(imu_idx), static_cast<uint8_t>(mag_idx), config.mag_instances)) {
			PX4_WARN("Instance %d: Sensors IMU%" PRId32 "+MAG%" PRId32 " not ready, retrying...",
			         instance, imu_idx, mag_idx);
			px4_usleep(10000);
			// Retry sensor check for this instance
			if (!isSensorDataValid(static_cast<uint8_t>(imu_idx), static_cast<uint8_t>(mag_idx), config.mag_instances)) {
				PX4_ERR("Instance %d: Sensors IMU%" PRId32 "+MAG%" PRId32 " failed to become ready",
				        instance, imu_idx, mag_idx);
				continue;
			}
		}

		// Create the EKF2 instance
		EKF2 *ekf2_inst = new EKF2(true, px4::ins_instance_to_wq(static_cast<uint8_t>(imu_idx)), false);

		if (ekf2_inst && ekf2_inst->multi_init(static_cast<uint8_t>(imu_idx), static_cast<uint8_t>(mag_idx))) {
			int actual_instance = ekf2_inst->instance();

			if ((actual_instance >= 0) && (_objects[actual_instance].load() == nullptr)) {
				_objects[actual_instance].store(ekf2_inst);
				instances_created++;

				PX4_INFO("EKF2[%d]: Created with IMU%" PRId32 " + MAG%" PRId32 " (manual assignment)",
				         actual_instance, imu_idx, mag_idx);

				logInstanceCreation(actual_instance, static_cast<uint8_t>(imu_idx), static_cast<uint8_t>(mag_idx));

				if (_ekf2_selector.load()) {
					_ekf2_selector.load()->ScheduleNow();
				}

			} else {
				PX4_ERR("Instance %d: numbering problem (actual instance: %d)", instance, actual_instance);
				delete ekf2_inst;
			}

		} else {
			PX4_ERR("Instance %d: Failed to create with IMU%" PRId32 " MAG%" PRId32,
			        instance, imu_idx, mag_idx);
			delete ekf2_inst;
		}
	}

	PX4_INFO("Successfully created %d EKF2 instances using manual sensor assignments", instances_created);
	return (instances_created > 0) ? PX4_OK : PX4_ERROR;
}

// ==== Utility Functions ====

bool EKF2::isSensorDataValid(uint8_t imu, uint8_t mag, int32_t mag_instances)
{
	uORB::SubscriptionData<vehicle_imu_s> vehicle_imu_sub{ORB_ID(vehicle_imu), imu};

#if defined(CONFIG_EKF2_MAGNETOMETER)
	uORB::SubscriptionData<vehicle_magnetometer_s> vehicle_mag_sub{ORB_ID(vehicle_magnetometer), mag};
	vehicle_mag_sub.update();

	// Mag & IMU data must be valid, first mag can be ignored initially
	return (vehicle_mag_sub.advertised() || mag == 0) && (vehicle_imu_sub.advertised());
#else
	return vehicle_imu_sub.advertised();
#endif // CONFIG_EKF2_MAGNETOMETER
}

bool EKF2::isVehicleArmed(uORB::SubscriptionData<vehicle_status_s> &vehicle_status_sub)
{
	vehicle_status_sub.update();
	return vehicle_status_sub.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED;
}

bool EKF2::isHilModeActive(uORB::SubscriptionData<vehicle_status_s> &vehicle_status_sub)
{
	return vehicle_status_sub.get().hil_state == vehicle_status_s::HIL_STATE_ON;
}

void EKF2::logInstanceCreation(int instance, uint8_t imu, uint8_t mag)
{
	uORB::SubscriptionData<vehicle_imu_s> vehicle_imu_sub{ORB_ID(vehicle_imu), imu};

#if defined(CONFIG_EKF2_MAGNETOMETER)
	uORB::SubscriptionData<vehicle_magnetometer_s> vehicle_mag_sub{ORB_ID(vehicle_magnetometer), mag};

	PX4_DEBUG("starting instance %d, IMU:%" PRIu8 " (%" PRIu32 "), MAG:%" PRIu8 " (%" PRIu32 ")",
	          instance,
	          imu, vehicle_imu_sub.get().accel_device_id,
	          mag, vehicle_mag_sub.get().device_id);
#else
	PX4_DEBUG("starting instance %d, IMU:%" PRIu8 " (%" PRIu32 ")",
	          instance,
	          imu, vehicle_imu_sub.get().accel_device_id);
#endif // CONFIG_EKF2_MAGNETOMETER
}

#endif // CONFIG_EKF2_MULTI_INSTANCE
