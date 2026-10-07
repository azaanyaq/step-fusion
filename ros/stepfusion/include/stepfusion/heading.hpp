#pragma once

namespace stepfusion {

constexpr double kPi = 3.14159265358979323846;

// Wraps into [-pi, pi]
double wrap_angle(double angle_rad);

// -ive to line up with the gyro's rotation direction (same as Python)
double magnetometer_heading(double mag_x, double mag_y);

// 1D Kalman filter on heading: predict from gyro every sample, update from magnetometer at steps
class HeadingFilter {
public:
    HeadingFilter(double process_noise_q, double measurement_noise_r);

    void reset(double heading_rad, double variance);
    void predict(double yaw_rate, double dt);
    void update(double mag_heading_rad);

    double heading() const { return heading_; }
    double variance() const { return variance_; }
    double last_gain() const { return last_gain_; }

private:
    double q_;
    double r_;
    double heading_ = 0.0;
    double variance_ = 0.0;
    double last_gain_ = 0.0;
};

}  // namespace stepfusion
