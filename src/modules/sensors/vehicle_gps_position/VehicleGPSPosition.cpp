/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#include "VehicleGPSPosition.hpp"

#include <px4_platform_common/log.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>

namespace sensors
{
VehicleGPSPosition::VehicleGPSPosition() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	// Advertise single instance publisher (for backward compatibility)
	_vehicle_gps_position_pub.advertise();
}

VehicleGPSPosition::~VehicleGPSPosition()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleGPSPosition::Start()
{
	// force initial updates
	ParametersUpdate(true);

	ScheduleNow();

	return true;
}

void VehicleGPSPosition::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_gps_sub) {
		sub.unregisterCallback();
	}
}

void VehicleGPSPosition::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		// GPS callback registration: always register both if multi-instance mode enabled
		if (_param_veh_gps_pos_multi.get()) {
			// Multi-instance mode: need both GPS receivers
			for (auto &sub : _sensor_gps_sub) {
				sub.registerCallback();
			}
		} else {
			// Standard mode: use existing logic
			if (_param_sens_gps_mask.get() == 0) {
				_sensor_gps_sub[0].registerCallback();
			} else {
				for (auto &sub : _sensor_gps_sub) {
					sub.registerCallback();
				}
			}
		}

		// Configure blending parameters (used for instance 0 in both modes)
		_gps_blending.setBlendingUseSpeedAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_SPD_ACC);
		_gps_blending.setBlendingUseHPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_HPOS_ACC);
		_gps_blending.setBlendingUseVPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_VPOS_ACC);
		_gps_blending.setBlendingTimeConstant(_param_sens_gps_tau.get());
		_gps_blending.setPrimaryInstance(_param_sens_gps_prime.get());

		// Log multi-instance mode if enabled
		if (_param_veh_gps_pos_multi.get() && !_multi_instance_warning_logged) {
			PX4_INFO("GPS Multi-Instance Mode ENABLED");
			PX4_INFO("Instance 0: Blended/selected GPS (for flight control)");
			PX4_INFO("Instance 1: Raw GPS 0 data");
			PX4_INFO("Instance 2: Raw GPS 1 data");
			_multi_instance_warning_logged = true;
		} else if (!_param_veh_gps_pos_multi.get() && _multi_instance_warning_logged) {
			PX4_INFO("GPS Multi-Instance Mode DISABLED");
			_multi_instance_warning_logged = false;
		}
	}
}

void VehicleGPSPosition::Run()
{
	perf_begin(_cycle_perf);
	ParametersUpdate();

	// Check all GPS instances for updates
	bool any_gps_updated = false;
	sensor_gps_s gps_data[GPS_MAX_RECEIVERS];
	bool gps_updated[GPS_MAX_RECEIVERS] = {false};

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if (_sensor_gps_sub[i].updated()) {
			_sensor_gps_sub[i].copy(&gps_data[i]);
			gps_updated[i] = true;
			any_gps_updated = true;

			// Update blending with new data
			_gps_blending.setGpsData(gps_data[i], i);

			if (!_sensor_gps_sub[i].registered()) {
				_sensor_gps_sub[i].registerCallback();
			}
		}
	}

	if (any_gps_updated) {
		// Update blending/selection logic
		_gps_blending.update(hrt_absolute_time());

		// Always publish blended/selected output to instance 0
		if (_gps_blending.isNewOutputDataAvailable()) {
			sensor_gps_s gps_output{_gps_blending.getOutputGpsData()};

			// Clear device_id if blending
			if (_gps_blending.getSelectedGps() == GpsBlending::GPS_MAX_RECEIVERS_BLEND) {
				gps_output.device_id = 0;
			}

			_vehicle_gps_position_pub.publish(gps_output);
		}

		// Multi-instance mode: publish raw GPS data to instances 1 and 2
		// NO FALLBACK - each instance only publishes if its GPS is available
		if (_param_veh_gps_pos_multi.get()) {
			// Publish GPS 0 to instance 1 ONLY if GPS 0 is updated
			if (gps_updated[0]) {
				sensor_gps_s gps0_copy = gps_data[0];
				gps0_copy.selected_rtcm_instance = 0; // Mark source GPS
				_vehicle_gps_position_pub_multi_1.publish(gps0_copy);
			}

			// Publish GPS 1 to instance 2 ONLY if GPS 1 is updated
			if (gps_updated[1]) {
				sensor_gps_s gps1_copy = gps_data[1];
				gps1_copy.selected_rtcm_instance = 1; // Mark source GPS
				_vehicle_gps_position_pub_multi_2.publish(gps1_copy);
			}

			// NO FALLBACK - removed the single GPS fallback scenario code
		}
	}

	ScheduleDelayed(300_ms); // backup schedule

	perf_end(_cycle_perf);
}

void VehicleGPSPosition::PrintStatus()
{
	PX4_INFO_RAW("[vehicle_gps_position] Selected GPS: %d\n", _gps_blending.getSelectedGps());

	if (_param_veh_gps_pos_multi.get()) {
		PX4_INFO_RAW("[vehicle_gps_position] Multi-Instance Mode Active:\n");
		PX4_INFO_RAW("  Instance 0: Blended/selected GPS (flight control)\n");
		PX4_INFO_RAW("  Instance 1: Raw GPS 0 data only (no fallback)\n");
		PX4_INFO_RAW("  Instance 2: Raw GPS 1 data only (no fallback)\n");
		PX4_INFO_RAW("  Note: Instances 1&2 will stop updating if their GPS fails\n");
	} else {
		PX4_INFO_RAW("[vehicle_gps_position] Standard mode (single instance)\n");
	}

	// Print blending configuration - cast to int to fix format warning
	PX4_INFO_RAW("[vehicle_gps_position] Blending mask: %d\n", (int)_param_sens_gps_mask.get());
	PX4_INFO_RAW("[vehicle_gps_position] Primary GPS: %d\n", (int)_param_sens_gps_prime.get());
}

}; // namespace sensors
