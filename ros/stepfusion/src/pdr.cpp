#include "stepfusion/pdr.hpp"

#include <cmath>

namespace stepfusion {

Pdr::Pdr(const PdrConfig& config)
    : config_(config),
      filter_(config.process_noise_q, config.measurement_noise_r),
      detector_(config.step) {}

void Pdr::add_mag(double /*t*/, double mag_x, double mag_y) {
    latest_mag_heading_ = magnetometer_heading(mag_x, mag_y);
    has_mag_ = true;
}

void Pdr::predict(double t, double yaw_rate) {
    if (!initialised_) {
        // Anchor to the first magnetometer heading, no prediction on the first sample (same as Python)
        if (!has_mag_) {
            return;
        }
        initial_heading_ = latest_mag_heading_;
        filter_.reset(initial_heading_, config_.measurement_noise_r); // P0 = R
        last_t_ = t;
        initialised_ = true;
        return;
    }

    const double dt = t - last_t_;
    last_t_ = t;
    gyro_integrated_ += yaw_rate * dt;
    filter_.predict(yaw_rate, dt);
}

StepEvent Pdr::step(double peak_time) {
    if (has_mag_) {
        filter_.update(latest_mag_heading_);
    }

    ++step_count_;
    const double fused_heading = filter_.heading();
    const double gyro_heading = raw_heading();

    fused_.x += config_.step_length_m * std::cos(fused_heading);
    fused_.y += config_.step_length_m * std::sin(fused_heading);
    raw_.x += config_.step_length_m * std::cos(gyro_heading);
    raw_.y += config_.step_length_m * std::sin(gyro_heading);

    StepEvent event;
    event.count = step_count_;
    event.peak_time = peak_time;
    event.fused_heading = fused_heading;
    event.raw_heading = gyro_heading;
    event.fused = fused_;
    event.raw = raw_;
    return event;
}

std::optional<StepEvent> Pdr::add_imu(double t, double yaw_rate, double acc_x, double acc_y, double acc_z) {
    predict(t, yaw_rate);
    if (!initialised_) {
        return std::nullopt;
    }

    const double magnitude = std::sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
    const std::optional<double> peak_time = detector_.add_sample(t, magnitude);
    if (!peak_time) {
        return std::nullopt;
    }
    return step(*peak_time); // Uses the heading now (when confirmed), not at the peak
}

double closing_distance(const Position& end) {
    return std::hypot(end.x, end.y);
}

}  // namespace stepfusion
