
#include <string>
#include <boost/signal.hpp>
#include <boost/version.hpp>
#include <boost/optional.hpp>
#include <boost/none.hpp>


#if BOOST_VERSION > 1048000
#define BOOST_HAS_CHRONO 1
#endif

#if BOOST_HAS_CHRONO
#include <boost/chrono/system_clocks.hpp>
#else
#include <ros/time.h>
#endif

namespace jaguar {

class RobotOdom {
public:
    enum odom_mode { kOrig, kContinuous };
    enum Side { kNone, kLeft, kRight };

    RobotOdom(enum odom_mode odom_mode);
    virtual ~RobotOdom(void);

    typedef void OdometryCallback(double, double, double, double, double, double, double);
    void set_circumference(double circum_m);
    void set_separation(double separation_m);
    double get_separation(void);
    void set_encoders(uint16_t cpr);
    void set_rate(uint8_t rate_ms);
    void set_mode(enum odom_mode which);
    void attach(boost::function<OdometryCallback> callback);

    typedef void (odom_update_fn)(Odometry &side_left, Odometry &side_right, double pos, double val);
    odom_update_fn update;

private:
    // Wheel Odometry
    struct Odometry {
        Side side;
        bool init;
        double pos_curr, pos_prev;
        double vel;
        double accel;
#if BOOST_HAS_CHRONO
        boost::chrono::high_resolution_clock::time_point time;
#else
        ros::Time time;
#endif
    };

#if 0
    void update_v1(Odometry &side_left, Odometry &side_right, double pos, double vel);
    void update_v2(Odometry &side_left, Odometry &side_right, double pos, double vel);
#else
    odom_update_fn update_v1;
    odom_update_fn update_v2;
#endif

    // Odometry
    double x_, y_, theta_;
    Side state_;
    double wheel_circum_, wheel_sep_;
    Odometry odom_[2];
    odom_update_fn *update_;
    boost::signal<OdometryCallback> signal_;

}; /* class RobotOdom */
}; /* namespace jaguar */
