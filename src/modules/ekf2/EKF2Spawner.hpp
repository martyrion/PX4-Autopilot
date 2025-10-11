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

#ifndef EKF2_SPAWNER_HPP
#define EKF2_SPAWNER_HPP

#include "EKF2.hpp"
#include <px4_platform_common/atomic.h>
#include <lib/mathlib/mathlib.h>

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
#include "EKF2Selector.hpp"
#endif

static constexpr uint8_t MAX_NUM_IMUS = 4;
static constexpr uint8_t MAX_NUM_MAGS = 4;

class EKF2Spawner
{
public:
	/**
	 * Spawn EKF2 instances based on configuration
	 */
	static int spawn(int argc, char *argv[],
			px4::atomic<EKF2 *> (&objects)[EKF2_MAX_INSTANCES]
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
			, px4::atomic<EKF2Selector *> &ekf2_selector
#endif
			);

private:
	/**
	 * Spawn a single EKF2 instance
	 */
	static bool spawnSingleInstance(px4::atomic<EKF2 *> (&objects)[EKF2_MAX_INSTANCES], bool replay_mode);

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	/**
	 * Spawn multiple EKF2 instances based on EKF2_INST_NO parameter
	 */
	static bool spawnMultiInstances(px4::atomic<EKF2 *> (&objects)[EKF2_MAX_INSTANCES],
					px4::atomic<EKF2Selector *> &ekf2_selector);
#endif
};

#endif // EKF2_SPAWNER_HPP
