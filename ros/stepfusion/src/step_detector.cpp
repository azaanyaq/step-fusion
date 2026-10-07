#include "stepfusion/step_detector.hpp"

namespace stepfusion {

StepDetector::StepDetector(const StepDetectorConfig& config) : config_(config) {}

std::optional<double> StepDetector::add_sample(double t, double accel_magnitude) {
    if (!in_peak_) {
        if (accel_magnitude > config_.high_threshold) {
            in_peak_ = true;
            peak_value_ = accel_magnitude;
            peak_time_ = t;
        }
        return std::nullopt;
    }

    if (accel_magnitude > peak_value_) {
        peak_value_ = accel_magnitude;
        peak_time_ = t;
    }

    // Step only confirmed once the magnitude drops back down
    if (accel_magnitude < config_.low_threshold) {
        in_peak_ = false;
        const bool far_enough = !has_last_step_ || (peak_time_ - last_step_time_) >= config_.min_step_gap_s;
        if (far_enough) {
            has_last_step_ = true;
            last_step_time_ = peak_time_;
            return peak_time_;
        }
    }
    return std::nullopt;
}

}  // namespace stepfusion
