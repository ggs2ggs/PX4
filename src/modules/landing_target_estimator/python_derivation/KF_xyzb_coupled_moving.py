#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Copyright (c) 2022 PX4 Development Team
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
    3. Neither the name PX4 nor the names of its contributors may be
    used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

File: KF_xyzb_coupled_moving.py
Description: Derive the Kalman filter equations for moving targets with coupled dynamics using symforce
Author: Jonas Perolini <jonas.perolini@epfl.ch>
"""

import symforce.symbolic as sf

def generate_px4_function(function_name, output_names):
    from symforce.codegen import Codegen, CppConfig
    import os
    import fileinput

    codegen = Codegen.function(
            function_name,
            output_names=output_names,
            config=CppConfig())
    metadata = codegen.generate_function(
            output_dir="src/modules/landing_target_estimator/python_derivation/generated/coupled_moving_xyzb",
            skip_directory_nesting=True)

    print("Files generated in {}:\n".format(metadata.output_dir))
    for f in metadata.generated_files:
        print("  |- {}".format(os.path.relpath(f, metadata.output_dir)))

    # Replace cstdlib and Eigen functions by PX4 equivalents
    with fileinput.FileInput(os.path.abspath(metadata.generated_files[0]), inplace=True) as file:
        for line in file:
            line = line.replace("std::max", "math::max")
            line = line.replace("std::min", "math::min")
            line = line.replace("Eigen", "matrix")
            line = line.replace("matrix/Dense", "matrix/math.hpp")
            print(line, end='')

#-------------------------------- COUPLED DYNAMICS, STATIC TARGET ------------------------------------------ #

class State:
    rx = 0
    ry = 1
    rz = 2
    rx_dot = 3
    ry_dot = 4
    rz_dot = 5
    bx = 6
    by = 7
    bz = 8
    atx = 9,
    aty = 10,
    atz = 11,
    n_states = 12

class Input:
    ax = 0
    ay = 1
    az = 2
    n_inputs = 3

class Directions:
    x = 0,
    y = 1,
    z = 2,
    nb_directions = 3

class VState(sf.Matrix):
    SHAPE = (State.n_states, 1)

class MState(sf.Matrix):
    SHAPE = (State.n_states, State.n_states)

class VInput(sf.Matrix):
    SHAPE = (Input.n_inputs, 1)

class VMeas(sf.Matrix):
    SHAPE = (1, State.n_states)

class MDirections(sf.Matrix):
    SHAPE = (Directions.nb_directions, Directions.nb_directions)


def predictState(dt: sf.Scalar, state: VState, acc: VInput) -> (VState):

    identity = sf.Matrix([  [1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])

    Phi = sf.Matrix.block_matrix([   [1 * identity, dt * identity, 0 * identity, 0.5*dt*dt * identity],
                        [0 * identity, 1 * identity,  0 * identity, dt * identity       ],
                        [0 * identity, 0 * identity,  1 * identity, 0  * identity       ],
                        [0 * identity, 0 * identity,  0 * identity, 1  * identity       ]])


    G = sf.Matrix.block_matrix([ [-0.5*dt*dt * identity], [-dt * identity], [0 * identity], [0 * identity]])

    return (Phi * state + G*acc).simplify()

def syncState(dt: sf.Scalar, state: VState, acc: VInput) -> (VState):

    identity = sf.Matrix([  [1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])

    Phi = sf.Matrix.block_matrix([   [1 * identity, dt * identity, 0 * identity, 0.5*dt*dt * identity],
                        [0 * identity, 1 * identity,  0 * identity, dt * identity       ],
                        [0 * identity, 0 * identity,  1 * identity, 0  * identity       ],
                        [0 * identity, 0 * identity,  0 * identity, 1  * identity       ]])


    G = sf.Matrix.block_matrix([ [-0.5*dt*dt * identity], [-dt * identity], [0 * identity], [0 * identity]])

    return (Phi.inv() * (state - G*acc)).simplify()

def predictCov(dt: sf.Scalar, input_var: MDirections, bias_var: MDirections, acc_var: MDirections, covariance: MState) -> (MState):
    identity = sf.Matrix([  [1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])

    Phi = sf.Matrix.block_matrix([   [1 * identity, dt * identity, 0 * identity, 0.5*dt*dt * identity],
                        [0 * identity, 1 * identity,  0 * identity, dt * identity       ],
                        [0 * identity, 0 * identity,  1 * identity, 0  * identity       ],
                        [0 * identity, 0 * identity,  0 * identity, 1  * identity       ]])

    G = sf.Matrix.block_matrix([ [-0.5*dt*dt * identity], [-dt * identity], [0 * identity], [0 * identity]])

    Q_acc_diag = sf.Matrix([  [acc_var[0,0], 0, 0],
                        [0, acc_var[1,1], 0],
                        [0, 0, acc_var[2,2]]])

    Q_acc = sf.Matrix.block_matrix([   [0 * identity, 0 * identity, 0 * identity, 0 * identity],
                    [0 * identity, 0 * identity,  0 * identity, 0 * identity       ],
                    [0 * identity, 0 * identity,  0 * identity, 0  * identity       ],
                    [0 * identity, 0 * identity,  0 * identity,  Q_acc_diag * identity       ]])

    Q_bias_diag = sf.Matrix([  [bias_var[0,0], 0, 0],
                            [0, bias_var[1,1], 0],
                            [0, 0, bias_var[2,2]]])

    Q_bias = sf.Matrix.block_matrix([   [0 * identity, 0 * identity, 0 * identity, 0 * identity],
                    [0 * identity, 0 * identity,  0 * identity, 0 * identity       ],
                    [0 * identity, 0 * identity,  Q_bias_diag, 0  * identity       ],
                    [0 * identity, 0 * identity,  0 * identity, 0  * identity       ]])

    return G*input_var*G.T + Q_bias + Q_acc + Phi*covariance*Phi.T


def computeInnovCov(meas_unc: sf.Scalar, covariance: MState, meas_matrix: VMeas) -> (sf.Scalar):
    return (meas_matrix*covariance*meas_matrix.T)[0,0] + meas_unc

generate_px4_function(predictState, output_names=["state_pred"])
generate_px4_function(syncState, output_names=["state_updated"])
generate_px4_function(predictCov, output_names=["cov_updated"])
generate_px4_function(computeInnovCov, output_names=["innov_cov_updated"])