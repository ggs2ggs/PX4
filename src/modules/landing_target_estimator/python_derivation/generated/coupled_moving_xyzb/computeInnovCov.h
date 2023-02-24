// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <matrix/math.hpp>

namespace sym
{

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: computeInnovCov
 *
 * Args:
 *     meas_unc: Scalar
 *     covariance: Matrix12_12
 *     meas_matrix: Matrix1_12
 *
 * Outputs:
 *     innov_cov_updated: Scalar
 */
template <typename Scalar>
void Computeinnovcov(const Scalar meas_unc, const matrix::Matrix<Scalar, 12, 12> &covariance,
		     const matrix::Matrix<Scalar, 1, 12> &meas_matrix,
		     Scalar *const innov_cov_updated = nullptr)
{
	// Total ops: 300

	// Input arrays

	// Intermediate terms (0)

	// Output terms (1)
	if (innov_cov_updated != nullptr) {
		Scalar &_innov_cov_updated = (*innov_cov_updated);

		_innov_cov_updated =
			meas_matrix(0, 0) *
			(covariance(0, 0) * meas_matrix(0, 0) + covariance(1, 0) * meas_matrix(0, 1) +
			 covariance(10, 0) * meas_matrix(0, 10) + covariance(11, 0) * meas_matrix(0, 11) +
			 covariance(2, 0) * meas_matrix(0, 2) + covariance(3, 0) * meas_matrix(0, 3) +
			 covariance(4, 0) * meas_matrix(0, 4) + covariance(5, 0) * meas_matrix(0, 5) +
			 covariance(6, 0) * meas_matrix(0, 6) + covariance(7, 0) * meas_matrix(0, 7) +
			 covariance(8, 0) * meas_matrix(0, 8) + covariance(9, 0) * meas_matrix(0, 9)) +
			meas_matrix(0, 1) *
			(covariance(0, 1) * meas_matrix(0, 0) + covariance(1, 1) * meas_matrix(0, 1) +
			 covariance(10, 1) * meas_matrix(0, 10) + covariance(11, 1) * meas_matrix(0, 11) +
			 covariance(2, 1) * meas_matrix(0, 2) + covariance(3, 1) * meas_matrix(0, 3) +
			 covariance(4, 1) * meas_matrix(0, 4) + covariance(5, 1) * meas_matrix(0, 5) +
			 covariance(6, 1) * meas_matrix(0, 6) + covariance(7, 1) * meas_matrix(0, 7) +
			 covariance(8, 1) * meas_matrix(0, 8) + covariance(9, 1) * meas_matrix(0, 9)) +
			meas_matrix(0, 10) *
			(covariance(0, 10) * meas_matrix(0, 0) + covariance(1, 10) * meas_matrix(0, 1) +
			 covariance(10, 10) * meas_matrix(0, 10) + covariance(11, 10) * meas_matrix(0, 11) +
			 covariance(2, 10) * meas_matrix(0, 2) + covariance(3, 10) * meas_matrix(0, 3) +
			 covariance(4, 10) * meas_matrix(0, 4) + covariance(5, 10) * meas_matrix(0, 5) +
			 covariance(6, 10) * meas_matrix(0, 6) + covariance(7, 10) * meas_matrix(0, 7) +
			 covariance(8, 10) * meas_matrix(0, 8) + covariance(9, 10) * meas_matrix(0, 9)) +
			meas_matrix(0, 11) *
			(covariance(0, 11) * meas_matrix(0, 0) + covariance(1, 11) * meas_matrix(0, 1) +
			 covariance(10, 11) * meas_matrix(0, 10) + covariance(11, 11) * meas_matrix(0, 11) +
			 covariance(2, 11) * meas_matrix(0, 2) + covariance(3, 11) * meas_matrix(0, 3) +
			 covariance(4, 11) * meas_matrix(0, 4) + covariance(5, 11) * meas_matrix(0, 5) +
			 covariance(6, 11) * meas_matrix(0, 6) + covariance(7, 11) * meas_matrix(0, 7) +
			 covariance(8, 11) * meas_matrix(0, 8) + covariance(9, 11) * meas_matrix(0, 9)) +
			meas_matrix(0, 2) *
			(covariance(0, 2) * meas_matrix(0, 0) + covariance(1, 2) * meas_matrix(0, 1) +
			 covariance(10, 2) * meas_matrix(0, 10) + covariance(11, 2) * meas_matrix(0, 11) +
			 covariance(2, 2) * meas_matrix(0, 2) + covariance(3, 2) * meas_matrix(0, 3) +
			 covariance(4, 2) * meas_matrix(0, 4) + covariance(5, 2) * meas_matrix(0, 5) +
			 covariance(6, 2) * meas_matrix(0, 6) + covariance(7, 2) * meas_matrix(0, 7) +
			 covariance(8, 2) * meas_matrix(0, 8) + covariance(9, 2) * meas_matrix(0, 9)) +
			meas_matrix(0, 3) *
			(covariance(0, 3) * meas_matrix(0, 0) + covariance(1, 3) * meas_matrix(0, 1) +
			 covariance(10, 3) * meas_matrix(0, 10) + covariance(11, 3) * meas_matrix(0, 11) +
			 covariance(2, 3) * meas_matrix(0, 2) + covariance(3, 3) * meas_matrix(0, 3) +
			 covariance(4, 3) * meas_matrix(0, 4) + covariance(5, 3) * meas_matrix(0, 5) +
			 covariance(6, 3) * meas_matrix(0, 6) + covariance(7, 3) * meas_matrix(0, 7) +
			 covariance(8, 3) * meas_matrix(0, 8) + covariance(9, 3) * meas_matrix(0, 9)) +
			meas_matrix(0, 4) *
			(covariance(0, 4) * meas_matrix(0, 0) + covariance(1, 4) * meas_matrix(0, 1) +
			 covariance(10, 4) * meas_matrix(0, 10) + covariance(11, 4) * meas_matrix(0, 11) +
			 covariance(2, 4) * meas_matrix(0, 2) + covariance(3, 4) * meas_matrix(0, 3) +
			 covariance(4, 4) * meas_matrix(0, 4) + covariance(5, 4) * meas_matrix(0, 5) +
			 covariance(6, 4) * meas_matrix(0, 6) + covariance(7, 4) * meas_matrix(0, 7) +
			 covariance(8, 4) * meas_matrix(0, 8) + covariance(9, 4) * meas_matrix(0, 9)) +
			meas_matrix(0, 5) *
			(covariance(0, 5) * meas_matrix(0, 0) + covariance(1, 5) * meas_matrix(0, 1) +
			 covariance(10, 5) * meas_matrix(0, 10) + covariance(11, 5) * meas_matrix(0, 11) +
			 covariance(2, 5) * meas_matrix(0, 2) + covariance(3, 5) * meas_matrix(0, 3) +
			 covariance(4, 5) * meas_matrix(0, 4) + covariance(5, 5) * meas_matrix(0, 5) +
			 covariance(6, 5) * meas_matrix(0, 6) + covariance(7, 5) * meas_matrix(0, 7) +
			 covariance(8, 5) * meas_matrix(0, 8) + covariance(9, 5) * meas_matrix(0, 9)) +
			meas_matrix(0, 6) *
			(covariance(0, 6) * meas_matrix(0, 0) + covariance(1, 6) * meas_matrix(0, 1) +
			 covariance(10, 6) * meas_matrix(0, 10) + covariance(11, 6) * meas_matrix(0, 11) +
			 covariance(2, 6) * meas_matrix(0, 2) + covariance(3, 6) * meas_matrix(0, 3) +
			 covariance(4, 6) * meas_matrix(0, 4) + covariance(5, 6) * meas_matrix(0, 5) +
			 covariance(6, 6) * meas_matrix(0, 6) + covariance(7, 6) * meas_matrix(0, 7) +
			 covariance(8, 6) * meas_matrix(0, 8) + covariance(9, 6) * meas_matrix(0, 9)) +
			meas_matrix(0, 7) *
			(covariance(0, 7) * meas_matrix(0, 0) + covariance(1, 7) * meas_matrix(0, 1) +
			 covariance(10, 7) * meas_matrix(0, 10) + covariance(11, 7) * meas_matrix(0, 11) +
			 covariance(2, 7) * meas_matrix(0, 2) + covariance(3, 7) * meas_matrix(0, 3) +
			 covariance(4, 7) * meas_matrix(0, 4) + covariance(5, 7) * meas_matrix(0, 5) +
			 covariance(6, 7) * meas_matrix(0, 6) + covariance(7, 7) * meas_matrix(0, 7) +
			 covariance(8, 7) * meas_matrix(0, 8) + covariance(9, 7) * meas_matrix(0, 9)) +
			meas_matrix(0, 8) *
			(covariance(0, 8) * meas_matrix(0, 0) + covariance(1, 8) * meas_matrix(0, 1) +
			 covariance(10, 8) * meas_matrix(0, 10) + covariance(11, 8) * meas_matrix(0, 11) +
			 covariance(2, 8) * meas_matrix(0, 2) + covariance(3, 8) * meas_matrix(0, 3) +
			 covariance(4, 8) * meas_matrix(0, 4) + covariance(5, 8) * meas_matrix(0, 5) +
			 covariance(6, 8) * meas_matrix(0, 6) + covariance(7, 8) * meas_matrix(0, 7) +
			 covariance(8, 8) * meas_matrix(0, 8) + covariance(9, 8) * meas_matrix(0, 9)) +
			meas_matrix(0, 9) *
			(covariance(0, 9) * meas_matrix(0, 0) + covariance(1, 9) * meas_matrix(0, 1) +
			 covariance(10, 9) * meas_matrix(0, 10) + covariance(11, 9) * meas_matrix(0, 11) +
			 covariance(2, 9) * meas_matrix(0, 2) + covariance(3, 9) * meas_matrix(0, 3) +
			 covariance(4, 9) * meas_matrix(0, 4) + covariance(5, 9) * meas_matrix(0, 5) +
			 covariance(6, 9) * meas_matrix(0, 6) + covariance(7, 9) * meas_matrix(0, 7) +
			 covariance(8, 9) * meas_matrix(0, 8) + covariance(9, 9) * meas_matrix(0, 9)) +
			meas_unc;
	}
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym