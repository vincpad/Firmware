/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "VehicleAngularVelocity.hpp"

#include <px4_log.h>

using namespace time_literals;
using namespace matrix;

VehicleAngularVelocity::VehicleAngularVelocity() :
	ModuleParams(nullptr),
	WorkItem(px4::wq_configurations::rate_ctrl),
	_cycle_perf(perf_alloc(PC_ELAPSED, "vehicle_angular_velocity: cycle time")),
	_interval_perf(perf_alloc(PC_INTERVAL, "vehicle_angular_velocity: interval")),
	_sensor_gyro_latency_perf(perf_alloc(PC_ELAPSED, "vehicle_angular_velocity: sensor gyro latency"))
{
	_gyro_count = math::constrain(orb_group_count(ORB_ID(sensor_gyro)), 1, MAX_GYRO_COUNT);

	/* initialize thermal corrections as we might not immediately get a topic update (only non-zero values) */
	for (unsigned i = 0; i < 3; i++) {
		// used scale factors to unity
		_sensor_correction.gyro_scale_0[i] = 1.0f;
		_sensor_correction.gyro_scale_1[i] = 1.0f;
		_sensor_correction.gyro_scale_2[i] = 1.0f;
	}

	parameters_updated();
}

VehicleAngularVelocity::~VehicleAngularVelocity()
{
	stop();

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
	perf_free(_sensor_gyro_latency_perf);
}

void
VehicleAngularVelocity::start()
{
	_sensor_correction_sub.register_callback();

	selected_gyro_update();
}

void
VehicleAngularVelocity::stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto sub : _sensor_gyro_sub) {
		sub.unregister_callback();
	}
}

void
VehicleAngularVelocity::parameters_updated()
{
	/* get transformation matrix from sensor/board to body frame */
	_board_rotation = get_rot_matrix((enum Rotation)_param_sens_board_rot.get());

	/* fine tune the rotation */
	const Dcmf board_rotation_offset(Eulerf(
			math::radians(_param_sens_board_x_off.get()),
			math::radians(_param_sens_board_y_off.get()),
			math::radians(_param_sens_board_z_off.get())));

	_board_rotation = board_rotation_offset * _board_rotation;
}

void
VehicleAngularVelocity::parameter_update_poll()
{
	/* Check if parameters have changed */
	parameter_update_s param_update;

	if (_params_sub.update(&param_update)) {
		updateParams();
		parameters_updated();
	}
}

bool
VehicleAngularVelocity::selected_gyro_update()
{
	// check if the selected gyro has updated first
	_sensor_correction_sub.update(&_sensor_correction);

	/* update the latest gyro selection */
	if (_selected_gyro != _sensor_correction.selected_gyro_instance) {
		if (_sensor_correction.selected_gyro_instance < _gyro_count) {
			// clear all registered callbacks
			for (auto sub : _sensor_gyro_sub) {
				sub.unregister_callback();
			}

			const int gyro_new = _sensor_correction.selected_gyro_instance;

			if (_sensor_gyro_sub[gyro_new].register_callback()) {
				PX4_DEBUG("selected gyro changed %d -> %d", _selected_gyro, gyro_new);
				_selected_gyro = gyro_new;

				return true;
			}
		}
	}

	return false;
}

void
VehicleAngularVelocity::Run()
{
	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	parameter_update_poll();
	selected_gyro_update();

	_sensor_bias_sub.update(&_sensor_bias);

	sensor_gyro_s gyro;

	if (_sensor_gyro_sub[_selected_gyro].update(&gyro)) {
		perf_set_elapsed(_sensor_gyro_latency_perf, hrt_elapsed_time(&gyro.timestamp));

		// get the raw gyro data and correct for thermal errors
		Vector3f rates;

		if (_selected_gyro == 0) {
			rates(0) = (gyro.x - _sensor_correction.gyro_offset_0[0]) * _sensor_correction.gyro_scale_0[0];
			rates(1) = (gyro.y - _sensor_correction.gyro_offset_0[1]) * _sensor_correction.gyro_scale_0[1];
			rates(2) = (gyro.z - _sensor_correction.gyro_offset_0[2]) * _sensor_correction.gyro_scale_0[2];

		} else if (_selected_gyro == 1) {
			rates(0) = (gyro.x - _sensor_correction.gyro_offset_1[0]) * _sensor_correction.gyro_scale_1[0];
			rates(1) = (gyro.y - _sensor_correction.gyro_offset_1[1]) * _sensor_correction.gyro_scale_1[1];
			rates(2) = (gyro.z - _sensor_correction.gyro_offset_1[2]) * _sensor_correction.gyro_scale_1[2];

		} else if (_selected_gyro == 2) {
			rates(0) = (gyro.x - _sensor_correction.gyro_offset_2[0]) * _sensor_correction.gyro_scale_2[0];
			rates(1) = (gyro.y - _sensor_correction.gyro_offset_2[1]) * _sensor_correction.gyro_scale_2[1];
			rates(2) = (gyro.z - _sensor_correction.gyro_offset_2[2]) * _sensor_correction.gyro_scale_2[2];

		} else {
			rates(0) = gyro.x;
			rates(1) = gyro.y;
			rates(2) = gyro.z;
		}

		// rotate corrected measurements from sensor to body frame
		rates = _board_rotation * rates;

		// correct for in-run bias errors
		rates(0) -= _sensor_bias.gyro_x_bias;
		rates(1) -= _sensor_bias.gyro_y_bias;
		rates(2) -= _sensor_bias.gyro_z_bias;

		vehicle_angular_velocity_s vrates{};
		vrates.timestamp_sample = gyro.timestamp;
		vrates.rollspeed = rates(0);
		vrates.pitchspeed = rates(1);
		vrates.yawspeed = rates(2);
		vrates.timestamp = hrt_absolute_time();

		_vehicle_angular_velocity_pub.publish(vrates);
	}

	perf_end(_cycle_perf);
}

int VehicleAngularVelocity::print_status()
{
	PX4_INFO("selected gyro: %d", _selected_gyro);

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	perf_print_counter(_sensor_gyro_latency_perf);

	return 0;
}
