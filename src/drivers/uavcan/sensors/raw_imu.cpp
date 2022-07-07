/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @author Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#include "raw_imu.hpp"
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>

const char *const UavcanRawIMUBridge::NAME = "raw_imu";

UavcanRawIMUBridge::UavcanRawIMUBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_raw_imu", ORB_ID(sensor_accel)),
	_sub_imu_data(node)
{ }

int UavcanRawIMUBridge::init()
{
	int res = _sub_imu_data.start(ImuCbBinder(this, &UavcanRawIMUBridge::imu_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void UavcanRawIMUBridge::imu_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::RawIMU> &msg)
{
	uavcan_bridge::Channel *channel = get_channel_for_node(msg.getSrcNodeID().get());

	hrt_abstime timestamp_sample = msg.timestamp.usec;

	if (timestamp_sample > hrt_absolute_time()) {
		timestamp_sample = hrt_absolute_time();
		PX4_WARN("raw_imu timestamp_sample: %llu > hrt_absolute_time: %llu", (unsigned long long)msg.timestamp.usec, (unsigned long long)hrt_absolute_time());
	}

	if (channel == nullptr) {
		// Something went wrong - no channel to publish on; return
		return;
	}

	// Cast our generic CDev pointer to the sensor-specific driver class
	PX4Accelerometer *accel = (PX4Accelerometer *)channel->h_driver;

	if (accel == nullptr) {
		return;
	}

	accel->set_error_count(0);
	accel->update(timestamp_sample, msg.accelerometer_latest[0], msg.accelerometer_latest[1], msg.accelerometer_latest[2]);

	// Cast our generic CDev pointer to the sensor-specific driver class
	PX4Gyroscope *gyro = (PX4Gyroscope *)channel->h_driver_secondary;

	if (gyro == nullptr) {
		return;
	}

	gyro->update(timestamp_sample,
		     msg.rate_gyro_latest[0],
		     msg.rate_gyro_latest[1],
		     msg.rate_gyro_latest[2]);
}

int UavcanRawIMUBridge::init_driver(uavcan_bridge::Channel *channel)
{
	// update device id as we now know our device node_id
	DeviceId device_id{_device_id};

	device_id.devid_s.address = static_cast<uint8_t>(channel->node_id);

	device_id.devid_s.devtype = DRV_ACC_DEVTYPE_UAVCAN;
	channel->h_driver = new PX4Accelerometer(device_id.devid);

	if (channel->h_driver == nullptr) {
		return PX4_ERROR;
	}

	PX4Accelerometer *accel = (PX4Accelerometer *)channel->h_driver;

	channel->instance = accel->get_instance();

	if (channel->instance < 0) {
		PX4_ERR("UavcanAccel: Unable to get a class instance");
		delete accel;
		channel->h_driver = nullptr;
		return PX4_ERROR;
	}

	device_id.devid_s.devtype = DRV_GYR_DEVTYPE_UAVCAN;
	channel->h_driver_secondary = new PX4Gyroscope(device_id.devid);

	if (channel->h_driver_secondary == nullptr) {
		return PX4_ERROR;
	}

	PX4Gyroscope *gyro = (PX4Gyroscope *)channel->h_driver_secondary;

	channel->instance_secondary = gyro->get_instance();

	if (channel->instance_secondary < 0) {
		PX4_ERR("UavcanGyro: Unable to get a class instance_secondary");
		delete gyro;
		channel->h_driver_secondary = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}
