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
  QST QMI8658A driver. Register map and timings from the QMI8658A
  datasheet Rev A (13-52-25).
 */

#include "AP_InertialSensor_QMI8658.h"

#include <utility>

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#ifdef HAL_QMI8658_DEBUG
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>
#endif

extern const AP_HAL::HAL& hal;

#define QMI8658_REG_WHO_AM_I        0x00
#define QMI8658_REG_REVISION_ID     0x01
#define QMI8658_REG_CTRL1           0x02
#define QMI8658_REG_CTRL2           0x03
#define QMI8658_REG_CTRL3           0x04
#define QMI8658_REG_CTRL5           0x06
#define QMI8658_REG_CTRL7           0x08
#define QMI8658_REG_FIFO_CTRL       0x14
#define QMI8658_REG_STATUSINT       0x2D
#define QMI8658_REG_TIMESTAMP_L     0x30
#define QMI8658_REG_RESET           0x60

#define QMI8658_WHO_AM_I            0x05
#define QMI8658_RESET_CMD           0xB0
// section 7.4 of the datasheet gives 0x0B instead, so try that as well
#define QMI8658_RESET_CMD_ALT       0x0B

// CTRL1: 4-wire SPI, address auto increment, INT pins high-Z, internal
// oscillator on. BE is left at its reset default: on the clone fitted to
// falcon, a soft reset followed by CTRL1=0x40 (BE cleared) left the SPI
// interface unreadable until power cycled. That clone outputs little-endian
// data regardless of BE
#define QMI8658_CTRL1_ADDR_AI       (1U<<6)
#define QMI8658_CTRL1_BE            (1U<<5)
#define QMI8658_CTRL1_VALUE         (QMI8658_CTRL1_ADDR_AI | QMI8658_CTRL1_BE)

// CTRL2: accel +-16g, ODR code 3 (896.8Hz in 6DOF mode)
#define QMI8658_CTRL2_VALUE         ((0x3U<<4) | 0x3U)
// CTRL3: gyro +-2048dps, ODR code 3 (896.8Hz)
#define QMI8658_CTRL3_VALUE         ((0x7U<<4) | 0x3U)
// CTRL7: SyncSample locking, gyro and accel enabled
#define QMI8658_CTRL7_VALUE         0x83

#define QMI8658_STATUSINT_AVAIL     (1U<<0)
#define QMI8658_STATUSINT_LOCKED    (1U<<1)

// in 6DOF mode the ODR is derived from the gyro natural frequency (+-2%)
#define QMI8658_SAMPLE_RATE_HZ      897
#define QMI8658_ACCEL_SCALE         (GRAVITY_MSS / 2048.0f)
#define QMI8658_GYRO_SCALE          (radians(1.0f) / 16.0f)

// Data_Lock_Delay for ODR code 3 is 6us
#define QMI8658_LOCK_DELAY_US       6
// system turn on time is 15ms max after reset
#define QMI8658_RESET_DELAY_MS      20
// gyro turn on time is 150ms + 3/ODR
#define QMI8658_GYRO_STARTUP_MS     160

// poll at a bit over twice the ODR so every sample is seen
#define QMI8658_POLL_PERIOD_US      500

#define QMI8658_TEMP_AVG_SAMPLES    64

struct PACKED QMI8658Sample {
    uint8_t timestamp[3];
    le16_t temp;
    le16_t accel[3];
    le16_t gyro[3];
};

AP_InertialSensor_QMI8658::AP_InertialSensor_QMI8658(AP_InertialSensor &imu,
                                                     AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                     enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(_dev))
    , rotation(_rotation)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_QMI8658::probe(AP_InertialSensor &imu,
                                 AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                 enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = NEW_NOTHROW AP_InertialSensor_QMI8658(imu, std::move(dev), rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_InertialSensor_QMI8658::init()
{
    dev->set_read_flag(0x80);
    return hardware_init();
}

bool AP_InertialSensor_QMI8658::write_checked(uint8_t reg, uint8_t value)
{
    // configuration registers only accept single cycle writes
    if (!dev->write_register(reg, value, true)) {
        return false;
    }
    uint8_t v = 0;
    if (dev->read_registers(reg, &v, 1) && v == value) {
        return true;
    }
#ifdef HAL_QMI8658_DEBUG
    AP_BoardConfig::config_error("QMI reg %02x wr %02x rd %02x", reg, value, v);
#endif
    return false;
}

bool AP_InertialSensor_QMI8658::hardware_init()
{
    WITH_SEMAPHORE(dev->get_semaphore());

    dev->setup_checked_registers(5, 20);
    dev->set_speed(AP_HAL::Device::SPEED_LOW);

    // the part keeps its configuration across an MCU reboot, so reset it
    // and retry if it does not identify itself
    uint8_t whoami = 0;
    bool found = false;
    for (uint8_t tries = 0; tries < 5; tries++) {
        if (dev->read_registers(QMI8658_REG_WHO_AM_I, &whoami, 1) &&
            whoami == QMI8658_WHO_AM_I) {
            found = true;
            break;
        }
        dev->write_register(QMI8658_REG_RESET, (tries & 1) ? QMI8658_RESET_CMD_ALT : QMI8658_RESET_CMD);
        hal.scheduler->delay(QMI8658_RESET_DELAY_MS);
    }
    if (!found) {
#ifdef HAL_QMI8658_DEBUG
        // bring-up aid: halt and report what the part actually answered
        uint8_t regs[9] {};
        for (uint8_t i = 0; i < ARRAY_SIZE(regs); i++) {
            dev->read_registers(i, &regs[i], 1);
        }
        AP_BoardConfig::config_error("QMI 00-08:%02x%02x%02x%02x%02x%02x%02x%02x%02x",
                                     regs[0], regs[1], regs[2], regs[3], regs[4],
                                     regs[5], regs[6], regs[7], regs[8]);
#endif
        return false;
    }

    // no soft reset once the part identifies itself: every config register
    // below is rewritten anyway
    uint8_t revision = 0;
    dev->read_registers(QMI8658_REG_REVISION_ID, &revision, 1);

    if (!write_checked(QMI8658_REG_CTRL1, QMI8658_CTRL1_VALUE) ||
        !write_checked(QMI8658_REG_CTRL2, QMI8658_CTRL2_VALUE) ||
        !write_checked(QMI8658_REG_CTRL3, QMI8658_CTRL3_VALUE) ||
        !write_checked(QMI8658_REG_CTRL5, 0) ||
        !dev->write_register(QMI8658_REG_FIFO_CTRL, 0) ||
        !write_checked(QMI8658_REG_CTRL7, QMI8658_CTRL7_VALUE)) {
        DEV_PRINTF("QMI8658: config failed\n");
        return false;
    }

    hal.scheduler->delay(QMI8658_GYRO_STARTUP_MS);

    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    DEV_PRINTF("QMI8658: rev 0x%02x\n", revision);
    return true;
}

void AP_InertialSensor_QMI8658::start()
{
    const uint32_t devid = dev->get_bus_id_devtype(DEVTYPE_INS_QMI8658);
    if (!_imu.register_accel(accel_instance, QMI8658_SAMPLE_RATE_HZ, devid) ||
        !_imu.register_gyro(gyro_instance, QMI8658_SAMPLE_RATE_HZ, devid)) {
        return;
    }

    set_accel_orientation(accel_instance, rotation);
    set_gyro_orientation(gyro_instance, rotation);

    dev->register_periodic_callback(QMI8658_POLL_PERIOD_US,
        FUNCTOR_BIND_MEMBER(&AP_InertialSensor_QMI8658::poll_data, void));
}

bool AP_InertialSensor_QMI8658::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

void AP_InertialSensor_QMI8658::poll_data()
{
    uint8_t status;
    if (!dev->read_registers(QMI8658_REG_STATUSINT, &status, 1)) {
        return;
    }
    if (!(status & QMI8658_STATUSINT_AVAIL)) {
        return;
    }
    // the STATUSINT read above starts the lock; it completes within
    // Data_Lock_Delay
    if (!(status & QMI8658_STATUSINT_LOCKED)) {
        hal.scheduler->delay_microseconds(QMI8658_LOCK_DELAY_US);
    }

    // reading through GZ_H releases the lock
    struct QMI8658Sample s;
    if (!dev->read_registers(QMI8658_REG_TIMESTAMP_L, (uint8_t *)&s, sizeof(s))) {
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
        return;
    }

    const uint32_t timestamp = s.timestamp[0] | (s.timestamp[1] << 8) | (s.timestamp[2] << 16);
    if (have_timestamp) {
        const uint32_t delta = (timestamp - last_timestamp) & 0xFFFFFFU;
        if (delta == 0) {
            // same sample as last poll
            return;
        }
        missed_samples += delta - 1;
    }
#ifdef HAL_QMI8658_DEBUG
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_report_ms > 10000) {
        last_report_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "QMI8658: missed %u", unsigned(missed_samples));
    }
#endif
    last_timestamp = timestamp;
    have_timestamp = true;

    const uint64_t sample_us = AP_HAL::micros64();

    Vector3f accel{float(int16_t(le16toh(s.accel[0]))),
                   float(int16_t(le16toh(s.accel[1]))),
                   float(int16_t(le16toh(s.accel[2])))};
    Vector3f gyro{float(int16_t(le16toh(s.gyro[0]))),
                  float(int16_t(le16toh(s.gyro[1]))),
                  float(int16_t(le16toh(s.gyro[2])))};
    accel *= QMI8658_ACCEL_SCALE;
    gyro *= QMI8658_GYRO_SCALE;

    _rotate_and_correct_accel(accel_instance, accel);
    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_accel_raw_sample(accel_instance, accel, sample_us);
    _notify_new_gyro_raw_sample(gyro_instance, gyro, sample_us);

    // T = TEMP_H + TEMP_L/256, refreshed at 8Hz
    temp_sum += int16_t(le16toh(s.temp)) * (1.0f / 256.0f);
    if (++temp_count == QMI8658_TEMP_AVG_SAMPLES) {
        _publish_temperature(accel_instance, temp_sum / temp_count);
        temp_sum = 0;
        temp_count = 0;
    }

    if (!dev->check_next_register()) {
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
    }
}
