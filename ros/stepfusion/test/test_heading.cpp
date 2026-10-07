#include <gtest/gtest.h>

#include <cmath>

#include "stepfusion/heading.hpp"

using stepfusion::HeadingFilter;
using stepfusion::kPi;

TEST(WrapAngle, MapsIntoMinusPiToPi) {
    EXPECT_NEAR(stepfusion::wrap_angle(0.0), 0.0, 1e-12);
    EXPECT_NEAR(stepfusion::wrap_angle(1.5 * kPi), -0.5 * kPi, 1e-12);
    EXPECT_NEAR(stepfusion::wrap_angle(-1.5 * kPi), 0.5 * kPi, 1e-12);
    EXPECT_NEAR(stepfusion::wrap_angle(10.0 * kPi + 0.1), 0.1, 1e-12);
}

TEST(MagnetometerHeading, NegatedArctan) {
    EXPECT_NEAR(stepfusion::magnetometer_heading(1.0, 0.0), 0.0, 1e-12);
    EXPECT_NEAR(stepfusion::magnetometer_heading(0.0, 1.0), -0.5 * kPi, 1e-12);
}

TEST(HeadingFilter, PredictIntegratesConstantYawRate) {
    HeadingFilter filter(1e-4, 1.0);
    filter.reset(0.0, 0.5);
    for (int i = 0; i < 100; ++i) {
        filter.predict(0.5, 0.01); // 0.5 rad/s for 1s
    }
    EXPECT_NEAR(filter.heading(), 0.5, 1e-12);
    EXPECT_NEAR(filter.variance(), 0.5 + 100 * 1e-4, 1e-12);
}

TEST(HeadingFilter, EqualUncertaintyMovesHalfway) {
    HeadingFilter filter(0.0, 1.0);
    filter.reset(0.0, 1.0); // P == R so gain is 0.5
    filter.update(0.2);
    EXPECT_NEAR(filter.last_gain(), 0.5, 1e-12);
    EXPECT_NEAR(filter.heading(), 0.1, 1e-12);
    EXPECT_NEAR(filter.variance(), 0.5, 1e-12);
}

TEST(HeadingFilter, HugeRMeansMagnetometerIsIgnored) {
    HeadingFilter filter(1e-6, 1e12);
    filter.reset(0.3, 1e-3);
    filter.update(2.0);
    EXPECT_LT(filter.last_gain(), 1e-12);
    EXPECT_NEAR(filter.heading(), 0.3, 1e-12);
}

TEST(HeadingFilter, CorrectsTheShortWayRoundTheWrap) {
    const double deg = kPi / 180.0;
    HeadingFilter filter(0.0, 1.0);
    filter.reset(179.0 * deg, 1.0);
    filter.update(-179.0 * deg); // 2 deg ahead, not 358 behind
    EXPECT_NEAR(filter.heading(), 180.0 * deg, 1e-12);
}

TEST(HeadingFilter, CorrectsTheShortWayAfterSeveralTurns) {
    const double deg = kPi / 180.0;
    HeadingFilter filter(0.0, 1.0);
    filter.reset(6.0 * kPi + 10.0 * deg, 1.0);
    filter.update(12.0 * deg);
    EXPECT_NEAR(filter.heading(), 6.0 * kPi + 11.0 * deg, 1e-12);
}
