#include <jaguar/robot_odom.hh>

void RobotOdom::set_circumference(double circum_m)
{
    wheel_circum_ = circum_m;
}

void RobotOdom::set_separation(double separation_m)
{
    wheel_sep_ = separation_m;
}

double RobotOdom::get_separation(void)
{
    return separation_m;
}

void RobotOdom::set_encoders(uint16_t cpr)
{
    block(
        jag_left_.config_encoders_set(cpr),
        jag_right_.config_encoders_set(cpr)
    );
}

void RobotOdom::set_rate(uint8_t rate_ms)
{
    block(
        jag_left_.periodic_enable(0, rate_ms),
        jag_right_.periodic_enable(0, rate_ms)
    );
}

void RobotOdom::attach(boost::function<OdometryCallback> callback)
{
    odom_signal_.connect(callback);
}

RobotOdom::~RobotOdom(void)
{
}

RobotOdom::RobotOdom(enum odom_mode odom_mode)
    : x_(0.0)
    , y_(0.0)
    , theta_(0.0)
    , state_(kNone)
    , wheel_circum_(0.0) /* invalid, set by set_circumference */
    , wheel_sep_(0.0)    /* invalid, set by set_seperation */
{
    odom_[0].side = kLeft;
    odom_[1].side = kRight;
    odom_[0].init = odom_[1].init = false;

    set_mode(odom_mode);
}

RobotOdom::update(enum Side which, double pos, double vel)
{
    update_(which, pos, vel)
}

template <class chrono_rep>
static Odometry predict_odom(Odometry &odom, chrono_rep cur_time)
{
    Odometry cur_odom;
    cur_odom.side    = odom.side;
    cur_odom.init    = odom.init;

    cur_odom.time    = cur_time;
    double diff = boost::chrono::duration_cast<boost::chrono::duration<double> >(
                    cur_time - odom.time).count();

    cur_odom.vel = odom.vel + odom.accel * diff;

    double ave_vel = (odom.vel + cur_odom.vel) / 2;
    cur_odom.pos_curr = ave_vel * diff + odom.pos_curr;
    cur_odom.pos_prev = odom.pos_curr; // Should always be ignored.
    cur_odom.accel = odom.accel; // This is the only item not adjusted.

    return cur_odom;
}

boost::tuple<double,double> DiffDriveRobot::calc_dist(Odometry &odom_left, Odometry &odom_right)
{
        // Compute the difference between the last two updates. Speed is
        // measured in RPMs, so all of these values are measured in
        // revolutions.
        double const revs_left  = odom_left.pos_curr  - odom_left.pos_prev;
        double const revs_right = odom_right.pos_curr - odom_right.pos_prev;

        // XXX: FIXME: HACK
        std::swap(revs_left, revs_right);

        // Convert from revolutions to meters.
        double const meters_left  = revs_left * wheel_circum_;
        double const meters_right = revs_right * wheel_circum_;

        return boost::make_tuple(meters_left, meters_right);
}

/* warning: modifies state */
boost::tuple<double,double> DiffDriveRobot::calc_odom(double meters_left, double meters_right)
{
        // Current conversion from 2 wheel distances to 2 dimentional motion:
        // turn-drive-turn
        double const meters  = (meters_left + meters_right) / 2;
        double const radians = (meters_left - meters_right) / wheel_sep_;

        x_ += meters * cos(theta_ + radians / 2);
        y_ += meters * sin(theta_ + radians / 2);
        theta_ = angles::normalize_angle(theta_ + radians);

        // Estimate the robot's current velocity.
        v_linear = (odom_right_.vel + odom_left_.vel) / 2;
        omega    = (odom_right_.vel - odom_left_.vel) / wheel_sep_;

        return boost::make_tuple(v_linear, omega);
}

void DiffDriveRobot::calc(Odometry &odom_left, Odometry &odom_right)
{
    boost::tuple<double,double> d = calc_dist(odom_left, odom_right);

    boost::tuple<double, double> vo = calc_odom(d(0), d(1))

    odom_signal_(x_, y_, theta_, vo(0), vo(1), d(0), d(1));
}

void DiffDriveRobot::set_mode(enum odom_mode which)
{
    switch(which) {
        case kOriginal:
            update_ = odom_update_v1;
            break;
        case kContinuous:
            update_ = odom_update_v2;
            break;
    }
}

void DiffDriveRobot::update_v2(enum Side which, double pos, double vel)
{
    if (wheel_circum_ == 0 || wheel_sep_ == 0) return;

    Odometry &odom = odom_[which], &odom_other = odom_[!which];

    double vel_prev = odom.vel;
    boost::chrono::high_resolution_clock::time_point past = odom.time;

    odom.pos_prev = odom.pos_curr;
    odom.pos_curr = pos;
    odom.vel = vel;
    odom.time = boost::chrono::high_resolution_clock::now();

    boost::chrono::high_resolution_clock::duration dur_diff = odom.time - past;
    double diff = boost::chrono::duration_cast<double>(dur_diff);

    odom.accel = (vel - vel_prev) / diff;

    // Skip the first sample from each wheel. This is necessary in case the
    // encoders came up in an unknown state.
    if (!odom.init) {
        odom.init = true;
        return;
    }

    if (!odom_other.init)
        return;

    Odometry other_pred = predict_odom(odom_other, odom.time);

    double v_linear, omega;

    if (odom.side == kLeft)
        calc(odom, other_pred, v_linear, omega);
    else
        calc(other_pred, odom, v_linear, omega);

}

void DiffDriveRobot::update_v1(enum Side side, double pos, double vel)
{
    if (wheel_circum_ == 0 || wheel_sep_ == 0) return;

    Odometry &odom = odom_[side];

    odom.pos_prev = odom.pos_curr;
    odom.pos_curr = pos;
    odom.vel = vel;
    odom.time = boost::chrono::high_resolution_clock::rep;

    // Skip the first sample from each wheel. This is necessary in case the
    // encoders came up in an unknown state.
    if (!odom.init) {
        odom.init = true;
        return;
    }

    // Update the state variables to indicate which odometry readings we
    // already have. Trigger a callback once we've received a pair of readings.
    if (odom_state_ == kNone) {
        odom_state_ = odom.side;
    } else if (odom.side != odom_state_) {
        double v_linear, omega;
        calc(odom_[kLeft], odom_[kRight], v_linear, omega);

        odom_signal_(x_, y_, theta_, v_linear, omega, meters_left, meters_right);
        odom_state_ = kNone;
    } else {
        std::cerr << "war: periodic update message was dropped" << std::endl;
    }
}

