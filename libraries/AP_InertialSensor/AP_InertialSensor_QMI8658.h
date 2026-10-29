/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  driver for the QST QMI8658A 6-axis IMU over SPI

  The FIFO on this part has to be put into a read mode with a CTRL9
  handshake for every read and drops incoming samples while in that
  mode, so this driver uses the SyncSample locking mechanism instead:
  STATUSINT is polled faster than the ODR, and a burst read ending at
  GZ_H fetches one locked sample. The on-chip sample counter is used to
  discard repeats and to count samples missed between polls.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_QMI8658 : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation);

    void start() override;
    bool update() override;

private:
    AP_InertialSensor_QMI8658(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev,
                              enum Rotation rotation);

    bool init();
    bool hardware_init();
    bool write_checked(uint8_t reg, uint8_t value);
    void poll_data();

    AP_HAL::OwnPtr<AP_HAL::Device> dev;
    enum Rotation rotation;

    uint32_t last_timestamp;
    bool have_timestamp;
    uint32_t missed_samples;
    uint32_t last_report_ms;

    float temp_sum;
    uint8_t temp_count;
};
