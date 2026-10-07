#pragma once

#include <optional>

namespace stepfusion {

struct StepDetectorConfig {
    double high_threshold = 2.0; // m/s^2, enter a peak
    double low_threshold = 1.3;  // m/s^2, leave a peak
    double min_step_gap_s = 0.4; // Steps can't happen faster than ~0.4s apart
};

// Live step detector with two thresholds (hysteresis), since find_peaks needs the whole recording
class StepDetector {
public:
    explicit StepDetector(const StepDetectorConfig& config);

    // Returns the peak time when a step is confirmed
    std::optional<double> add_sample(double t, double accel_magnitude);

private:
    StepDetectorConfig config_;
    bool in_peak_ = false;
    double peak_value_ = 0.0;
    double peak_time_ = 0.0;
    bool has_last_step_ = false;
    double last_step_time_ = 0.0;
};

}  // namespace stepfusion
