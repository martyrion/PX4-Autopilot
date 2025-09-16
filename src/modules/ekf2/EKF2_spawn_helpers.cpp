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
 * Helper functions for EKF2 task spawning and instance management.
 */

#include "EKF2.hpp"

using namespace time_literals;

// Simple struct for sensor selection
struct SensorSelection {
	uint8_t imu;
	uint8_t mag;
};

static SensorSelection getInstanceConfiguration(int instance)
{
	char imu_param[16], mag_param[16];
	snprintf(imu_param, sizeof(imu_param), "EKF2_%d_IMU", instance);
	snprintf(mag_param, sizeof(mag_param), "EKF2_%d_MAG", instance);

	int32_t imu_index = instance;  // Default to instance number
	int32_t mag_index = 0;         // Default to MAG 0

	param_get(param_find(imu_param), &imu_index);
	param_get(param_find(mag_param), &mag_index);

	return {
		static_cast<uint8_t>(imu_index),
		static_cast<uint8_t>(mag_index)
	};
}

static bool hasManualConfiguration()
{
	// Check if any EKF2_X_IMU parameter exists
	for (int instance = 0; instance < 4; instance++) {
		char imu_param[16];
		snprintf(imu_param, sizeof(imu_param), "EKF2_%d_IMU", instance);

		if (param_find(imu_param) != PARAM_INVALID) {
			return true;
		}
	}
	return false;
}

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

// ==== Multi-Instance Functions ====

int EKF2::allocateMultiInstances(const SpawnConfig &config)
{
	InstanceAllocationState state;
	state.time_started = hrt_absolute_time();

	if (hasManualConfiguration()) {
		// Use manual configuration
		return attemptManualInstanceCreation(config, state);
	} else {
		// Use automatic configuration
		return attemptAutomaticInstanceCreation(config, state);
	}
}

int EKF2::attemptManualInstanceCreation(const SpawnConfig &config, InstanceAllocationState &state)
{
	bool created_any_instance = false;
	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};

	// Always create 4 instances when in multi-instance mode
	const int instances_to_create = math::min(4, static_cast<int>(EKF2_MAX_INSTANCES));

	while (shouldContinueAllocation(state, instances_to_create, vehicle_status_sub)) {

		for (int instance = 0; instance < instances_to_create; instance++) {
			SensorSelection selection = getInstanceConfiguration(instance);

			// Skip if already created for this sensor combination
			if (state.ekf2_instance_created[selection.imu][selection.mag]) {
				continue;
			}

			// Validate sensor availability
			if (selection.imu >= static_cast<uint8_t>(config.imu_instances) ||
			    selection.mag >= static_cast<uint8_t>(config.mag_instances)) {
				PX4_WARN("EKF2[%d]: IMU%u MAG%u not available (have %" PRId32 " IMUs, %" PRId32 " MAGs)",
				         instance, selection.imu, selection.mag,
				         config.imu_instances, config.mag_instances);
				continue;
			}

			// Check sensor data availability
			if (!isSensorDataValid(selection.imu, selection.mag, config.mag_instances)) {
				px4_usleep(1000);
				continue;
			}

			// Create instance
			if (createEKF2Instance(selection.imu, selection.mag, state)) {
				PX4_INFO("EKF2[%d]: IMU%u+MAG%u (manual)",
				         instance, selection.imu, selection.mag);
				state.ekf2_instance_created[selection.imu][selection.mag] = true;
				created_any_instance = true;

				// Stop if we've reached the limit
				if (state.multi_instances_allocated >= instances_to_create) {
					return PX4_OK;
				}
			} else {
				px4_usleep(100000);
				continue;
			}
		}

		px4_usleep(10000);
	}

	return created_any_instance ? PX4_OK : PX4_ERROR;
}

int EKF2::attemptAutomaticInstanceCreation(const SpawnConfig &config, InstanceAllocationState &state)
{
	// Always create 4 instances when in multi-instance mode
	const int instances_to_create = math::min(4, static_cast<int>(EKF2_MAX_INSTANCES));

	// allocate EKF2 instances until all found or arming
	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};

	while (shouldContinueAllocation(state, instances_to_create, vehicle_status_sub)) {
		if (attemptInstanceCreation(config, state)) {
			// Successfully created at least one instance this iteration
		} else {
			px4_usleep(10000); // Wait before next attempt if no instances created
		}
	}

	return (state.multi_instances_allocated > 0) ? PX4_OK : PX4_ERROR;
}

bool EKF2::shouldContinueAllocation(const InstanceAllocationState &state,
                                   int multi_instances,
                                   uORB::SubscriptionData<vehicle_status_s> &vehicle_status_sub)
{
	return (state.multi_instances_allocated < multi_instances)
	       && !isVehicleArmed(vehicle_status_sub)
	       && ((hrt_elapsed_time(&state.time_started) < 30_s) || isHilModeActive(vehicle_status_sub));
}

bool EKF2::attemptInstanceCreation(const SpawnConfig &config, InstanceAllocationState &state)
{
	bool created_instance = false;

	// iterate through all imu/mag combinations configured
	for (uint8_t mag = 0; mag < static_cast<uint8_t>(config.mag_instances); mag++) {
		for (uint8_t imu = 0; imu < static_cast<uint8_t>(config.imu_instances); imu++) {

			// Respect max global limit
			if (state.multi_instances_allocated >= static_cast<int>(EKF2_MAX_INSTANCES)) {
				return created_instance;
			}

			// Skip if already created for this sensor pair
			if (state.ekf2_instance_created[imu][mag]) {
				continue;
			}

			// Check sensor data availability
			if (!isSensorDataValid(imu, mag, config.mag_instances)) {
				// sensor not ready yet -> try others or retry later
				continue;
			}

			// Create instance
			if (createEKF2Instance(imu, mag, state)) {
				PX4_INFO("EKF2[auto]: IMU%u+MAG%u", imu, mag);
				state.ekf2_instance_created[imu][mag] = true;
				created_instance = true;

				// If we've reached the requested number of instances, we can return quickly
				const int requested_instances = math::min(config.imu_instances * config.mag_instances,
				                                         static_cast<int32_t>(EKF2_MAX_INSTANCES));
				if (state.multi_instances_allocated >= requested_instances) {
					return created_instance;
				}
			} else {
				// allocation failed: avoid spinning too fast
				px4_usleep(100000);
			}
		}
	}

	return created_instance;
}

bool EKF2::createEKF2Instance(uint8_t imu, uint8_t mag, InstanceAllocationState &state)
{
	// Access the external static variable declared in EKF2.cpp
	extern px4::atomic<EKF2 *> _objects[EKF2_MAX_INSTANCES];
	extern px4::atomic<EKF2Selector *> _ekf2_selector;

	EKF2 *ekf2_inst = new EKF2(true, px4::ins_instance_to_wq(imu), false);

	if (ekf2_inst && ekf2_inst->multi_init(imu, mag)) {
		int actual_instance = ekf2_inst->instance(); // match uORB instance numbering

		if ((actual_instance >= 0) && (_objects[actual_instance].load() == nullptr)) {
			_objects[actual_instance].store(ekf2_inst);
			state.multi_instances_allocated++;

			logInstanceCreation(actual_instance, imu, mag);

			if (_ekf2_selector.load()) {
				_ekf2_selector.load()->ScheduleNow();
			}

			return true;
		} else {
			PX4_ERR("instance numbering problem instance: %d", actual_instance);
			delete ekf2_inst;
			return false;
		}
	} else {
		PX4_ERR("alloc and init failed imu: %" PRIu8 " mag:%" PRIu8, imu, mag);
		delete ekf2_inst;
		return false;
	}
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
