/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file EKF2CustomParams.cpp
 * Helper functions for EKF2::Run() method
 * @author Dimitris Fkiaras <
 */

#include "EKF2.hpp"

void EKF2::applyCustomParameters()
{
	PX4_INFO("Instance %d: Applying custom parameters", _instance);

	// Arrays for all custom parameters based on instance ID
	int32_t height_refs[] = {
		_param_ekf2_0_hgt_ref.get(),
		_param_ekf2_1_hgt_ref.get(),
		_param_ekf2_2_hgt_ref.get(),
		_param_ekf2_3_hgt_ref.get(),
		_param_ekf2_4_hgt_ref.get(),
		_param_ekf2_5_hgt_ref.get()
	};

	int32_t gnss_ctrls[] = {
		_param_ekf2_0_gps_ctrl.get(),
		_param_ekf2_1_gps_ctrl.get(),
		_param_ekf2_2_gps_ctrl.get(),
		_param_ekf2_3_gps_ctrl.get(),
		_param_ekf2_4_gps_ctrl.get(),
		_param_ekf2_5_gps_ctrl.get()
	};

	int32_t mag_types[] = {
		_param_ekf2_0_mag_type.get(),
		_param_ekf2_1_mag_type.get(),
		_param_ekf2_2_mag_type.get(),
		_param_ekf2_3_mag_type.get(),
		_param_ekf2_4_mag_type.get(),
		_param_ekf2_5_mag_type.get()
	};

	// Validate instance
	if (_instance >= 0 && _instance < 6) {
		// Log current values before applying
		PX4_INFO("  Before applying custom params for instance %d:", _instance);
		PX4_INFO("    height_ref=%d, gnss_ctrl=%d, mag_type=%d",
			 (int)_params->height_sensor_ref, (int)_params->gnss_ctrl, (int)_params->mag_fusion_type);

		// Apply custom parameters
		_params->height_sensor_ref = height_refs[_instance];
		_params->gnss_ctrl     = gnss_ctrls[_instance];
		_params->mag_fusion_type        = mag_types[_instance];

		// Log after applying
		PX4_INFO("  After applying custom params for instance %d:", _instance);
		PX4_INFO("    height_ref=%d, gnss_ctrl=%d, mag_type=%d",
			 (int)_params->height_sensor_ref, (int)_params->gnss_ctrl, (int)_params->mag_fusion_type);
	}
}
