#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <angles/angles.h>
#include <jaguar/diff_drive.h>

using can::JaguarBridge;

namespace jaguar {

template <typename T>
inline T sgn(T x)
{
    if      (x > 0) return  1;
    else if (x < 0) return -1;
    else            return  0;
}

DiffDriveRobot::DiffDriveRobot(DiffDriveSettings const &settings, RobotOdom &odom)
    : bridge_(settings.port)
    , jag_broadcast_(bridge_)
    , jag_left_(bridge_, settings.id_left)
    , jag_right_(bridge_, settings.id_right)
    , diag_init_(false)
    , accel_max_(settings.accel_max_mps2)
    , odom_(odom)
{
    block(
        jag_left_.config_brake_set(settings.brake),
        jag_right_.config_brake_set(settings.brake)
    );

    // Configure the Jaguars to use optical encoders. They are used as both a
    // speed reference for velocity control and position reference for
    // odometry. As such, they must be configured for position control even
    // though we are are using speed control mode.
    block(
        jag_left_.position_set_reference(PositionReference::kQuadratureEncoder),
        jag_right_.position_set_reference(PositionReference::kQuadratureEncoder)
    );

    block(
        jag_left_.periodic_config_odom(0,
            boost::bind(&RobotOdom::update, &odom_, RobotOdom::kLeft,  _1, _2)),
        jag_right_.periodic_config_odom(0,
            boost::bind(&RobotOdom::update, &odom_, RobotOdom::kRight, _1, _2))
    );

    speed_init();
    diag_init();

    // This is necessary for the Jaguars to work after a fresh boot, even if
    // we never called system_halt() or system_reset().
    jag_broadcast_.system_resume();
}

DiffDriveRobot::~DiffDriveRobot(void)
{
}

void DiffDriveRobot::drive(double v, double omega)
{
    double const v_left  = v - 0.5 * wheel_sep_ * omega;
    double const v_right = v + 0.5 * wheel_sep_ * omega;
    drive_raw(v_left, v_right);
}

void DiffDriveRobot::drive_raw(double v_left, double v_right)
{
    target_rpm_left_  = v_left  * 60 / wheel_circum_;
    target_rpm_right_ = v_right * 60 / wheel_circum_;
}

void DiffDriveRobot::drive_spin(double dt)
{
    double const residual_rpm_left  = target_rpm_left_  - current_rpm_left_;
    double const residual_rpm_right = target_rpm_right_ - current_rpm_right_;

    // Cap the acceleration at the limiting value.
    double const drpm_max = accel_max_ * dt * 60 / wheel_circum_;

    if (fabs(residual_rpm_left) <= drpm_max) {
        current_rpm_left_ = target_rpm_left_;
    } else {
        current_rpm_left_ += sgn(residual_rpm_left) * drpm_max;
    }

    if (fabs(residual_rpm_right) <= drpm_max) {
        current_rpm_right_ = target_rpm_right_;
    } else {
        current_rpm_right_ += sgn(residual_rpm_right) * drpm_max;
    }

    block(
        jag_left_.speed_set(current_rpm_left_),
        jag_right_.speed_set(current_rpm_right_)
    );
}

void DiffDriveRobot::drive_brake(bool braking)
{
    jaguar::BrakeCoastSetting::Enum value;
    if (braking) {
        value = jaguar::BrakeCoastSetting::kOverrideBrake;
    } else {
        value = jaguar::BrakeCoastSetting::kOverrideCoast;
    }

    block(
        jag_left_.config_brake_set(value),
        jag_right_.config_brake_set(value)
    );
}

void DiffDriveRobot::diag_set_rate(uint8_t rate_ms)
{
    block(
        jag_left_.periodic_enable(1, rate_ms),
        jag_right_.periodic_enable(1, rate_ms)
    );
}

void DiffDriveRobot::heartbeat(void)
{
    jag_broadcast_.heartbeat();
}


void DiffDriveRobot::diag_attach(
    boost::function<DiagnosticsCallback> callback_left,
    boost::function<DiagnosticsCallback> callback_right)
{
    diag_left_signal_.connect(callback_left);
    diag_right_signal_.connect(callback_right);
}

void DiffDriveRobot::estop_attach(boost::function<EStopCallback> callback)
{
    estop_signal_.connect(callback);
}

/*
 * Diagnostics
 */
void DiffDriveRobot::diag_init(void)
{
    block(
        jag_left_.periodic_config_diag(1,
            boost::bind(&DiffDriveRobot::diag_update, this,
                kLeft, boost::ref(diag_left_), _1, _2, _3, _4)
        ),
        jag_right_.periodic_config_diag(1,
            boost::bind(&DiffDriveRobot::diag_update, this,
                kRight, boost::ref(diag_right_), _1, _2, _3, _4)
        )
    );

    // TODO: Make this a parameter.
    block(
        jag_left_.periodic_enable(1, 500),
        jag_right_.periodic_enable(1, 500)
    );
}

void DiffDriveRobot::diag_update(
    Side side, Diagnostics &diag,
    LimitStatus::Enum limits, Fault::Enum faults,
    double voltage, double temperature)
{
    bool const estop_before = diag_left_.stopped || diag_right_.stopped;

    // TODO: Check for a fault.
    diag.stopped = !(limits & 0x03);
    diag.voltage = voltage;
    diag.temperature = temperature;

    bool const estop_after = diag_left_.stopped || diag_right_.stopped;

    // Only trigger an e-stop callback if the state changed. We don't know the
    // initial state, so the first update always triggers a callback.
    if (!diag_init_ || estop_after != estop_before) {
        estop_signal_(estop_after);
    }
    diag_init_ = true;

    // Other diagnostics (i.e. bus voltage and temperature) use separate left
    // and right callbacks.
    if (side == kLeft) {
        diag_left_signal_(voltage, temperature);
    } else if (side == kRight) {
        diag_right_signal_(voltage, temperature);
    }
}

/*
 * Speed Control
 */
void DiffDriveRobot::speed_set_p(double p)
{
    block(
        jag_left_.speed_set_p(p),
        jag_right_.speed_set_p(p)
    );
}

void DiffDriveRobot::speed_set_i(double i)
{
    block(
        jag_left_.speed_set_i(i),
        jag_right_.speed_set_i(i)
    );
}

void DiffDriveRobot::speed_set_d(double d)
{
    block(
        jag_left_.speed_set_d(d),
        jag_right_.speed_set_d(d)
    );
}

void DiffDriveRobot::speed_init(void)
{
    block(
        jag_left_.speed_set_reference(SpeedReference::kQuadratureEncoder),
        jag_right_.speed_set_reference(SpeedReference::kQuadratureEncoder)
    );
    block(
        jag_left_.speed_enable(),
        jag_right_.speed_enable()
    );
}

/*
 * Helper Methods
 */
void DiffDriveRobot::block(can::TokenPtr t1, can::TokenPtr t2)
{
    t1->block();
    t2->block();
}

};
