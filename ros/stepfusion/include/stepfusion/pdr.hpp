#pragma once

#include <optional>

#include "stepfusion/heading.hpp"
#include "stepfusion/step_detector.hpp"

namespace stepfusion {

struct PdrConfig {
    double process_noise_q = 0.0;
    double measurement_noise_r = 0.0; // Already includes R_MULTIPLIER
    double step_length_m = 0.73;      // Same fixed step length as Python
    StepDetectorConfig step;
};

struct Position {
    double x = 0.0;
    double y = 0.0;
};

struct StepEvent {
    int count = 0;
    double peak_time = 0.0;
    double fused_heading = 0.0;
    double raw_heading = 0.0;
    Position fused;
    Position raw;
};

// Live dead reckoning, builds the raw (gyro only) and fused (Kalman) paths side by side like main.py
class Pdr {
public:
    explicit Pdr(const PdrConfig& config);

    void add_mag(double t, double mag_x, double mag_y);
    std::optional<StepEvent> add_imu(double t, double yaw_rate, double acc_x, double acc_y, double acc_z);

    // The two halves of add_imu, separate so tests can use Python's step times
    void predict(double t, double yaw_rate);
    StepEvent step(double peak_time);

    bool initialised() const { return initialised_; }
    double fused_heading() const { return filter_.heading(); }
    double raw_heading() const { return gyro_integrated_ + initial_heading_; }
    const HeadingFilter& filter() const { return filter_; }
    const Position& fused_position() const { return fused_; }
    const Position& raw_position() const { return raw_; }
    int step_count() const { return step_count_; }

private:
    PdrConfig config_;
    HeadingFilter filter_;
    StepDetector detector_;

    bool has_mag_ = false;
    double latest_mag_heading_ = 0.0;

    bool initialised_ = false;
    double last_t_ = 0.0;
    double initial_heading_ = 0.0;
    double gyro_integrated_ = 0.0;

    Position fused_;
    Position raw_;
    int step_count_ = 0;
};

// Straight line distance from path's start -> end point (should be 0m)
double closing_distance(const Position& end);

}  // namespace stepfusion
