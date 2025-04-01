/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <ctype.h>
#include <systemlib/err.h>

#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/param.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>

#include <pthread.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_gnss_relative.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/monitoring.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_preflight_mag.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/gps_inject_data.h>
#include <uORB/topics/takeoff_status.h>

#include <lib/mathlib/mathlib.h>
#include <systemlib/err.h>
#include <parameters/param.h>

#define MIN_SETPOINT_ACCEL -50
#define MIN_INIT_PITCH_THRESHOLD 0.3491  // Adjusted to approximately 20 degrees in radians
#define MIN_INIT_ROLL_THRESHOLD 0.3491   // Adjusted to approximately 20 degrees in radians
#define MIN_INIT_VELZ_THRESHOLD 0.25     // Multicopter vertical velocity threshold for landing detection

extern "C" __EXPORT int monitoring_main(int argc, char *argv[]);


class Monitoring : public ModuleBase<Monitoring>, public ModuleParams
{
private:
    enum MonitoringStatus {
        SAFETY_LOCK_STATUS = 0,          // Safety lock status
        ARM_STATUS,                      // Arming status

        OFFBOARD_MODE,                   // Offboard mode active
        MANUAL_MODE,                     // Manual mode active
        AUTO_MODE,                       // Auto mode active
        FAIL_SAFE_MODE,                  // Fail-safe mode active

        BATTERY_PROBLEM,                 // When the battery is below 25%

        GPS_JAMMING,                     // GPS jamming detected
        RTKGPS_BASE_RECV,                // RTK GPS base received
        RTKGPS_FIXED_MODE,               // RTK GPS fixed mode

        INIT_PITCH_PROBLEM,              // Initialization pitch problem
        INIT_ROLL_PROBLEM,               // Initialization roll problem
        INIT_VELZ_PROBLEM,               // Initialization velocity Z problem

        PERI_5V_POWER_PROBLEM,           // Peripheral 5V power problem

    };

public:
    Monitoring();

    virtual ~Monitoring() = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static Monitoring *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::run() */
    void run() override;

    /** @see ModuleBase::print_status() */
    int print_status() override;

private:
    void update_subscribe();
    void publish_monitoring();

    uint32_t status1();
    uint32_t status2(); // RESERVED

    // Subscriptions
    uORB::Subscription                      _vehicle_status_sub = {ORB_ID(vehicle_status)};
    uORB::Subscription                      _vehicle_attitude_sub = {ORB_ID(vehicle_attitude)};
    uORB::Subscription                      _local_pos_sub = {ORB_ID(vehicle_local_position)};
    uORB::Subscription                      _global_pos_sub = {ORB_ID(vehicle_global_position)};
    uORB::Subscription                      _sensor_gps_sub = {ORB_ID(sensor_gps)};
    uORB::Subscription                      _rtk_gps_sub = {ORB_ID(sensor_gnss_relative)};
    uORB::Subscription                      _system_power_sub = {ORB_ID(system_power)};
    uORB::Subscription                      _battery_status_sub = {ORB_ID(battery_status)};
    uORB::Subscription                      _vehicle_lpos_setpoint_sub = {ORB_ID(vehicle_local_position_setpoint)};
    uORB::Subscription                      _gps_inject_data_sub = {ORB_ID(gps_inject_data)};
    uORB::Subscription                      _takeoff_status_sub = {ORB_ID(takeoff_status)};

    // Publications
    uORB::Publication<monitoring_s>         _monitoring_pub{ORB_ID(monitoring)};

    struct vehicle_status_s                 _status{};
    struct vehicle_attitude_s               _att{};
    struct vehicle_local_position_s         _local_pos{};
    struct vehicle_global_position_s        _global_pos{};
    struct sensor_gnss_relative_s           _rtk_gps{};
    struct sensor_gps_s                     _gps{};
    struct sensor_combined_s                _sensors{};
    struct system_power_s                   _system_power{};
    struct battery_status_s                 _battery_status{};
    struct takeoff_status_s                 _takeoff_status{};

    struct monitoring_s                     _monitoring{};

    float _roll;
    float _pitch;
};
