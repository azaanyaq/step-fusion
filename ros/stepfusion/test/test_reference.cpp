// Golden tests feed Python's step times so C++ should match Python exactly

#include <gtest/gtest.h>

#include <map>
#include <set>
#include <string>
#include <vector>

#include "stepfusion/csv.hpp"
#include "stepfusion/pdr.hpp"
#include "stepfusion/recording.hpp"

namespace {

const std::string kRepoRoot = STEPFUSION_REPO_ROOT;

constexpr double kTolerance = 1e-9;

struct Reference {
    std::map<std::string, double> summary;
    stepfusion::CsvTable heading;
    stepfusion::CsvTable steps;
    stepfusion::CsvTable path;
};

Reference load_reference(const std::string& name) {
    const std::string dir = kRepoRoot + "/test_data/reference/" + name;
    return {stepfusion::load_key_value_csv(dir + "/summary.csv"), stepfusion::load_csv(dir + "/heading.csv"),
            stepfusion::load_csv(dir + "/steps.csv"), stepfusion::load_csv(dir + "/path.csv")};
}

stepfusion::PdrConfig config_from(const Reference& reference) {
    stepfusion::PdrConfig config;
    config.process_noise_q = reference.summary.at("Q");
    config.measurement_noise_r = reference.summary.at("R");
    config.step_length_m = reference.summary.at("step_length_m");
    return config;
}

class GoldenTest : public ::testing::TestWithParam<std::string> {};

}  // namespace

TEST_P(GoldenTest, MatchesPythonGivenPythonsStepTimes) {
    const std::string name = GetParam();
    const Reference reference = load_reference(name);
    const stepfusion::Recording recording = stepfusion::load_recording(kRepoRoot + "/data/raw/" + name);

    std::set<int> step_indices;
    for (double index : reference.steps.column("index")) {
        step_indices.insert(static_cast<int>(index));
    }

    const std::vector<double> ref_kf = reference.heading.column("kf_heading");
    const std::vector<double> ref_raw_x = reference.path.column("raw_x");
    const std::vector<double> ref_raw_y = reference.path.column("raw_y");
    const std::vector<double> ref_kf_x = reference.path.column("kf_x");
    const std::vector<double> ref_kf_y = reference.path.column("kf_y");
    ASSERT_EQ(ref_kf.size(), recording.gyro.size());

    stepfusion::Pdr pdr(config_from(reference));
    int steps_taken = 0;

    for (size_t i = 0; i < recording.gyro.size(); ++i) {
        pdr.add_mag(recording.mag[i].t, recording.mag[i].x, recording.mag[i].y);
        pdr.predict(recording.gyro[i].t, recording.gyro[i].z);

        if (i > 0 && step_indices.count(static_cast<int>(i)) > 0) { // Python never updates at sample 0
            const stepfusion::StepEvent event = pdr.step(recording.gyro[i].t);
            ++steps_taken;
            ASSERT_NEAR(event.fused.x, ref_kf_x[steps_taken], kTolerance) << "step " << steps_taken;
            ASSERT_NEAR(event.fused.y, ref_kf_y[steps_taken], kTolerance) << "step " << steps_taken;
            ASSERT_NEAR(event.raw.x, ref_raw_x[steps_taken], kTolerance) << "step " << steps_taken;
            ASSERT_NEAR(event.raw.y, ref_raw_y[steps_taken], kTolerance) << "step " << steps_taken;
        }
        ASSERT_NEAR(pdr.fused_heading(), ref_kf[i], kTolerance) << "sample " << i;
    }

    EXPECT_EQ(steps_taken, static_cast<int>(reference.summary.at("n_steps")));
    EXPECT_NEAR(stepfusion::closing_distance(pdr.raw_position()), reference.summary.at("raw_closing_m"), kTolerance);
    EXPECT_NEAR(stepfusion::closing_distance(pdr.fused_position()), reference.summary.at("kf_closing_m"), kTolerance);
}

INSTANTIATE_TEST_SUITE_P(Recordings, GoldenTest, ::testing::Values("demo_4", "demo_5", "demo_6"));

TEST(LiveTest, Demo6StepCountCloseToCountedGroundTruth) {
    const Reference reference = load_reference("demo_6");
    stepfusion::Pdr pdr(config_from(reference));
    stepfusion::replay_recording(stepfusion::load_recording(kRepoRoot + "/data/raw/demo_6"), pdr);

    EXPECT_NEAR(pdr.step_count(), 150, 10); // 150 counted while walking
}
