// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_Gear.h
/// @brief	Parachute release library

#ifndef AP_GEAR_H
#define AP_GEAR_H

#include <AP_Param.h>
#include <AP_Common.h>
#include <AP_InertialNav.h>     // Inertial Navigation library

#define AP_GEAR_TRIGGER_TYPE_RELAY_0       0
#define AP_GEAR_TRIGGER_TYPE_RELAY_1       1
#define AP_GEAR_TRIGGER_TYPE_RELAY_2       2
#define AP_GEAR_TRIGGER_TYPE_RELAY_3       3
#define AP_GEAR_TRIGGER_TYPE_SERVO         10

#define AP_GEAR_RELEASE_DELAY_MS           500    // delay in milliseconds between call to release() and when servo or relay actually moves.  Allows for warning to user
#define AP_GEAR_RELEASE_DURATION_MS       1000    // when parachute is released, servo or relay stay at their released position/value for 1000ms (1second)

#define AP_GEAR_SERVO_ON_PWM_DEFAULT      1900    // default PWM value to move servo to when shutter is activated
#define AP_GEAR_SERVO_OFF_PWM_DEFAULT     1100    // default PWM value to move servo to when shutter is deactivated

#define AP_GEAR_ALT_MIN_DEFAULT            300      // default min altitude in meters the vehicle will retract / release
#define AP_GEAR_SPEED_MAX_DEFAULT           50    // default max horizontal speed in m/s the vehicle will retract / release
#define AP_GEAR_AUTO_TIMEOUT_DEFAULT      2000

// in ./ArduCopter/defines.h - how to get from here?
// #define DATA_GEAR_DISABLED      52
// #define DATA_GEAR_ENABLED       53
// #define DATA_GEAR_AUTO          54
// #define DATA_GEAR_UP            55
// #define DATA_GEAR_DOWN          56

/// @class	AP_Gear
/// @brief	Class managing the release and retract of landing gear
class AP_Gear {

public:

    /// Constructor
    AP_Gear(const AP_InertialNav* inav) :
        _inav(inav),
        _released(false),
        _retract(false),
        _switch_time(0),
        _gearmode(2)
    {
        // setup parameter defaults
        AP_Param::setup_object_defaults(this, var_info);
    }

    /// enabled - enable or disable landing gear operation
    void enabled(bool on_off);

    /// enabled - returns true if landing gear operation is enabled
    bool enabled() const { return _enabled; }

    /// autonomous mode - retract above x meters and release below with speed < y
    void autonomous();

    /// enabled - returns true if landing gear operation is enabled
    bool released() const { return _released; }

    /// release - release landing gear
    void release();

    /// retract - retract landing gear
    void retract();

    /// update - shuts off the trigger should be called at about 10hz
    void update(int8_t control_mode);

    /// alt_min - returns the min altitude above home the vehicle should have before landing gear is retracted
    ///   0 = no auto retract
    int16_t alt_min() const { return _alt_min; }

    static const struct AP_Param::GroupInfo        var_info[];

private:
    // Parameters
    AP_Int8     _enabled;       // 1 if parachute release is enabled
    AP_Int16    _servo_on_pwm;  // PWM value to move servo to when shutter is activated
    AP_Int16    _servo_off_pwm; // PWM value to move servo to when shutter is deactivated
    AP_Int16    _alt_min;       // min altitude the vehicle should have before landing gear is retracted
    AP_Int16    _speed_max;     // max speed the vehile should have before landing gear is released

    // internal variables
    const AP_InertialNav *const _inav;
    bool        _released;      // true if the parachute has been released
    bool        _retract;
    uint32_t    _switch_time;
    uint8_t     _gearmode;      // 0 manual release, 1 manual retract, 2 autonomous
};

#endif /* AP_GEAR_H */