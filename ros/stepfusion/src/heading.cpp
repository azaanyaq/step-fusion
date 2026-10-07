#include "stepfusion/heading.hpp"

#include <cmath>

namespace stepfusion {

double wrap_angle(double angle_rad) {
    return std::remainder(angle_rad, 2.0 * kPi);
}

double magnetometer_heading(double mag_x, double mag_y) {
    return -std::atan2(mag_y, mag_x);
}

HeadingFilter::HeadingFilter(double process_noise_q, double measurement_noise_r)
    : q_(process_noise_q), r_(measurement_noise_r) {}

void HeadingFilter::reset(double heading_rad, double variance) {
    heading_ = heading_rad;
    variance_ = variance;
    last_gain_ = 0.0;
}

void HeadingFilter::predict(double yaw_rate, double dt) {
    heading_ += yaw_rate * dt;
    variance_ += q_;
}

void HeadingFilter::update(double mag_heading_rad) {
    // Wrapped innovation replaces np.unwrap (can't unwrap a signal that's still arriving)
    const double innovation = wrap_angle(mag_heading_rad - heading_);

    const double gain = variance_ / (variance_ + r_);
    heading_ += gain * innovation;
    variance_ = (1.0 - gain) * variance_;
    last_gain_ = gain;
}

}  // namespace stepfusion
