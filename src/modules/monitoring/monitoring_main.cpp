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

#include "monitoring_main.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>


int Monitoring::print_status()
{
    PX4_INFO("Monitoring TOW : %" PRIu32, _monitoring.tow);
    PX4_INFO("RTK TOW : %" PRIu64, _rtk_gps.time_utc_usec);
    PX4_INFO("Battery : %d", _monitoring.battery);
    PX4_INFO("Position : %.2f, %.2f, %.2f", (double)_monitoring.pos_x, (double)_monitoring.pos_y, (double)_monitoring.pos_z);
    PX4_INFO("status : %" PRIx32, _monitoring.status1);
    PX4_INFO("RTK flag(%d), sat(%d)", _gps.fix_type, _gps.satellites_used);
    PX4_INFO("Velocity: %.2f, %.2f, %.2f", (double)_local_pos.vx, (double)_local_pos.vy, (double)_local_pos.vz);
    PX4_INFO("accLength : %.3f, ", (double)_rtk_gps.accuracy_length);

    return 0;
}

int Monitoring::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        print_usage("not running");
        return 1;
    }

    return print_usage("unknown command");
}


int Monitoring::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("monitoring",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  1900,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        PX4_WARN("task start failed");
        _task_id = -1;
        return -errno;
    }

    return 0;
}

Monitoring *Monitoring::instantiate(int argc, char *argv[])
{
    Monitoring *instance = new Monitoring();

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
    }

    return instance;
}

Monitoring::Monitoring(): ModuleParams(nullptr)
{
}

void Monitoring::run()
{
    // Example: run the loop synchronized to the sensor_combined topic publication
    int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

    px4_pollfd_struct_t fds[1] {};
    fds[0].fd = sensor_combined_sub;
    fds[0].events = POLLIN;

    while (!should_exit()) {

        // wait for up to 50ms for data
        int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 50);

        if (pret == 0) {
            // Timeout: let the loop run anyway, don't do `continue` here

        } else if (pret < 0) {
            // this is undesirable but not much we can do
            PX4_ERR("poll error %d, %d", pret, errno);
            px4_usleep(50000);
            continue;

        } else if (fds[0].revents & POLLIN) {
            struct sensor_combined_s sensor_combined;
            orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);

            /* update subscribe */
            update_subscribe();

            /* publish monitoring */
            publish_monitoring();
        }
    }

    orb_unsubscribe(sensor_combined_sub);
}

int Monitoring::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("module", "monitoring");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int monitoring_main(int argc, char *argv[])
{
    return Monitoring::main(argc, argv);
}

void Monitoring::update_subscribe() {
    /* sensor_gnss_relative */
    if (_vehicle_status_sub.updated())      _vehicle_status_sub.copy(&_status);
    if (_vehicle_attitude_sub.updated())    _vehicle_attitude_sub.copy(&_att);
    if (_global_pos_sub.updated())           _global_pos_sub.copy(&_global_pos);
    if (_rtk_gps_sub.updated())             _rtk_gps_sub.copy(&_rtk_gps);
    if (_sensor_gps_sub.updated())          _sensor_gps_sub.copy(&_gps);
    if (_system_power_sub.updated())        _system_power_sub.copy(&_system_power);
    if (_battery_status_sub.updated())      _battery_status_sub.copy(&_battery_status);
    if (_takeoff_status_sub.updated())      _takeoff_status_sub.copy(&_takeoff_status);
    if (_local_pos_sub.updated()){
        _local_pos_sub.copy(&_local_pos);
        const matrix::Eulerf euler = matrix::Quatf(_att.q);
        _roll = euler.phi();
        _pitch = euler.theta();
    }

    if (_vehicle_lpos_setpoint_sub.updated()){
        vehicle_local_position_setpoint_s setpoint;

        _vehicle_lpos_setpoint_sub.copy(&setpoint);
        // Lock down on receiving abnormal values in local position setpoint acceleration
        if (setpoint.acceleration[2] < MIN_SETPOINT_ACCEL && _local_pos.z > -1.0f &&
            _status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
            _status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {

            vehicle_command_s vcmd{};
            vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION;
            vcmd.param1 = 1.0f;
            vcmd.param2 = 0.0f;

            uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
            vcmd.source_system = vehicle_status_sub.get().system_id;
            vcmd.target_system = vehicle_status_sub.get().system_id;
            vcmd.source_component = vehicle_status_sub.get().component_id;
            vcmd.target_component = vehicle_status_sub.get().component_id;

            uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
            vcmd.timestamp = hrt_absolute_time();
            vcmd_pub.publish(vcmd);
        }
    }
}

void Monitoring::publish_monitoring(){
    _monitoring.tow  = (uint32_t)(_rtk_gps.time_utc_usec / 1000);
    _monitoring.pos_x = _local_pos.x;
    _monitoring.pos_y = _local_pos.y;
    _monitoring.pos_z = _local_pos.z;
    _monitoring.head = _local_pos.heading;

    _monitoring.lat = _global_pos.lat;
    _monitoring.lon = _global_pos.lon;
    _monitoring.alt = _global_pos.alt;

    _monitoring.ref_lat = _local_pos.ref_lat;
    _monitoring.ref_lon = _local_pos.ref_lon;
    _monitoring.ref_alt = _local_pos.ref_alt;

    _monitoring.roll = _roll;
    _monitoring.pitch = _pitch;

    _monitoring.status1 = status1();
    _monitoring.status2 = status2();
    _monitoring.satellites_used = _gps.satellites_used;
    _monitoring.battery = _battery_status.remaining > 0.0f ? _battery_status.remaining * 100.0f : 0;

    _monitoring.rtk_n = _rtk_gps.position[0];
    _monitoring.rtk_e = _rtk_gps.position[1];
    _monitoring.rtk_d = _rtk_gps.position[2];
    _monitoring.nav_state = _status.nav_state;

    _monitoring.timestamp = hrt_absolute_time();
    _monitoring_pub.publish(_monitoring);
}

uint32_t Monitoring::status1()
{
    uint32_t status = 0;

    status |= !_status.safety_off ? 1 << SAFETY_LOCK_STATUS : 0;
    status |= _status.arming_state == vehicle_status_s::ARMING_STATE_ARMED ? 1 << ARM_STATUS : 0;
    status |= _status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL ? 1  << MANUAL_MODE : 0;
    status |= _status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD ? 1  << OFFBOARD_MODE : 0;
    status |= _status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ? 1  << AUTO_MODE : 0;
    status |= _status.failsafe == true ? 1 << FAIL_SAFE_MODE : 0;

    float bat_remain = _monitoring.battery = _battery_status.remaining > 0.0f ? _battery_status.remaining*100.0f : 0;
    status |= bat_remain < 25.0f ? 1 << BATTERY_PROBLEM : 0;

    status |= _gps.jamming_state > 1 ? 1 << GPS_JAMMING : 0;

    if (_gps_inject_data_sub.updated()) {
        status |= 1 << RTKGPS_BASE_RECV;
    } else {
        status &= ~(1 << RTKGPS_BASE_RECV);
    }

    status |= _gps.fix_type == 6 ? 1 << RTKGPS_FIXED_MODE : 0;

    if ( _system_power.voltage5v_v < 4.9f || _system_power.voltage5v_v > 5.4f ) {
        status |= 1 << PERI_5V_POWER_PROBLEM;
    }

    if (_takeoff_status.takeoff_state <= takeoff_status_s::TAKEOFF_STATE_READY_FOR_TAKEOFF) {
        if (fabsf(_pitch) < static_cast<float>(MIN_INIT_PITCH_THRESHOLD)) {
            status |= 1 << INIT_PITCH_PROBLEM;
        }

        if (fabsf(_roll) < static_cast<float>(MIN_INIT_ROLL_THRESHOLD)) {
            status |= 1 << INIT_ROLL_PROBLEM;
        }

        if (-_local_pos.vz < static_cast<float>(MIN_INIT_VELZ_THRESHOLD)) {
            status |= 1 << INIT_VELZ_PROBLEM;
        }
    } else {
        status &= ~(1 << INIT_PITCH_PROBLEM);
        status &= ~(1 << INIT_ROLL_PROBLEM);
        status &= ~(1 << INIT_VELZ_PROBLEM);
    }

    return status;
}

uint32_t Monitoring::status2()
{
    // RESERVED
    return 0;
}
