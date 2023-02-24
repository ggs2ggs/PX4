/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file KF_xyzb_coupled_moving.cpp
 * @brief Filter to estimate the pose of moving targets. State: [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]
 *
 * @author Jonas Perolini <jonas.perolini@epfl.ch>
 *
 */

#include "KF_xyzb_coupled_moving.h"
#include "python_derivation/generated/coupled_moving_xyzb/predictCov.h"
#include "python_derivation/generated/coupled_moving_xyzb/computeInnovCov.h"
#include "python_derivation/generated/coupled_moving_xyzb/syncState.h"

namespace landing_target_estimator
{

void KF_xyzb_coupled_moving::predictState(float dt, matrix::Vector<float, 3> acc)
{

	// [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]
	const float tmp0 = 0.5f * dt * dt;

	_state(0, 0) = _state(0, 0) + _state(3, 0) * dt + _state(9, 0) * tmp0 - tmp0 * acc(0);
	_state(1, 0) = _state(1, 0) + _state(10, 0) * tmp0 + _state(4, 0) * dt - tmp0 * acc(1);
	_state(2, 0) = _state(11, 0) * tmp0 + _state(2, 0) + _state(5, 0) * dt - tmp0 * acc(2);
	_state(3, 0) = _state(3, 0) + _state(9, 0) * dt - acc(0) * dt;
	_state(4, 0) = _state(10, 0) * dt + _state(4, 0) - acc(1) * dt;
	_state(5, 0) = _state(11, 0) * dt + _state(5, 0) - acc(2) * dt;
}

void KF_xyzb_coupled_moving::predictCov(float dt)
{
	matrix::Matrix<float, 12, 12> cov_updated;
	sym::Predictcov(dt, _input_var, _bias_var, _acc_var, _covariance, &cov_updated);
	_covariance = cov_updated;
}


bool KF_xyzb_coupled_moving::update()
{
	// Avoid zero-division
	if ((float)abs(_innov_cov) < (float)1e-6) {
		return false;
	}

	const float beta = _innov / _innov_cov * _innov;

	// 5% false alarm probability
	if (beta > 3.84f) {
		return false;
	}

	const matrix::Matrix<float, 12, 1> kalmanGain = _covariance * _meas_matrix.transpose() / _innov_cov;

	_state = _state + kalmanGain * _innov;
	_covariance = _covariance - kalmanGain * _meas_matrix * _covariance;

	return true;
}

void KF_xyzb_coupled_moving::setH(matrix::Vector<float, 15> h_meas)
{
	// h_meas = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz, vtx, vty, vtz]
	// idx    = [0,   1,  2,      3,      4,      5,  6,  7,  8,   9,  10,  11,  12,  13,  14]
	_meas_matrix(0, 0) = h_meas(0);
	_meas_matrix(0, 1) = h_meas(1);
	_meas_matrix(0, 2) = h_meas(2);
	_meas_matrix(0, 3) = h_meas(3);
	_meas_matrix(0, 4) = h_meas(4);
	_meas_matrix(0, 5) = h_meas(5);
	_meas_matrix(0, 6) = h_meas(6);
	_meas_matrix(0, 7) = h_meas(7);
	_meas_matrix(0, 8) = h_meas(8);
	_meas_matrix(0, 9) = h_meas(9);
	_meas_matrix(0, 10) = h_meas(10);
	_meas_matrix(0, 11) = h_meas(11);
}

void KF_xyzb_coupled_moving::syncState(float dt, matrix::Vector<float, 3> acc)
{
	matrix::Matrix<float, 12, 1> sync_stat_updated;
	sym::Syncstate(dt, _state, acc, &sync_stat_updated);
	_sync_state = sync_stat_updated;
}

float KF_xyzb_coupled_moving::computeInnovCov(float meas_unc)
{
	float innov_cov_updated;
	sym::Computeinnovcov(meas_unc, _covariance, _meas_matrix, &innov_cov_updated);
	_innov_cov = innov_cov_updated;

	return _innov_cov;
}

float KF_xyzb_coupled_moving::computeInnov(float meas)
{
	/* z - H*x */
	_innov = meas - (_meas_matrix * _sync_state)(0, 0);
	return _innov;
}

} // namespace landing_target_estimator