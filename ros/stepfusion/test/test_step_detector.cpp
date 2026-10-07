#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "stepfusion/step_detector.hpp"

using stepfusion::StepDetector;
using stepfusion::StepDetectorConfig;

namespace {

constexpr double kDt = 0.01; // 100Hz like the recordings

// Flat 1.0 baseline with a triangular spike at each peak time
std::vector<double> make_signal(const std::vector<double>& peak_times, double peak_value, double duration_s) {
    const int n = static_cast<int>(duration_s / kDt);
    std::vector<double> signal(n, 1.0);
    for (double peak : peak_times) {
        for (int i = 0; i < n; ++i) {
            const double distance = std::fabs(i * kDt - peak);
            if (distance < 0.1) {
                signal[i] = std::fmax(signal[i], peak_value - (peak_value - 1.0) * distance / 0.1);
            }
        }
    }
    return signal;
}

std::vector<double> run(StepDetector& detector, const std::vector<double>& signal) {
    std::vector<double> step_times;
    for (size_t i = 0; i < signal.size(); ++i) {
        if (const auto step = detector.add_sample(i * kDt, signal[i])) {
            step_times.push_back(*step);
        }
    }
    return step_times;
}

}  // namespace

TEST(StepDetector, FindsEveryClearPeakAtItsMaximum) {
    const std::vector<double> peaks = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0};
    StepDetector detector(StepDetectorConfig{});
    const std::vector<double> steps = run(detector, make_signal(peaks, 3.0, 6.0));

    ASSERT_EQ(steps.size(), peaks.size());
    for (size_t i = 0; i < peaks.size(); ++i) {
        EXPECT_NEAR(steps[i], peaks[i], kDt);
    }
}

TEST(StepDetector, IgnoresPeaksBelowHighThreshold) {
    StepDetector detector(StepDetectorConfig{});
    EXPECT_TRUE(run(detector, make_signal({0.5, 1.0, 1.5}, 1.8, 2.0)).empty());
}

TEST(StepDetector, EnforcesMinimumGapBetweenSteps) {
    StepDetector detector(StepDetectorConfig{});
    const std::vector<double> steps = run(detector, make_signal({0.5, 0.75}, 3.0, 1.5));
    ASSERT_EQ(steps.size(), 1u);
    EXPECT_NEAR(steps[0], 0.5, kDt);
}

TEST(StepDetector, HysteresisStopsWobbleCountingTwice) {
    StepDetector detector(StepDetectorConfig{});
    const std::vector<double> signal = {1.0, 2.1, 1.9, 2.4, 1.95, 2.2, 1.0, 1.0};
    const std::vector<double> steps = run(detector, signal);
    ASSERT_EQ(steps.size(), 1u);
    EXPECT_NEAR(steps[0], 3 * kDt, 1e-12);
}
