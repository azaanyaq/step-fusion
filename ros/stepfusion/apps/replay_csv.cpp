// Runs the live C++ pipeline on one recording (no ROS) and prints it next to the Python results.
// Usage from repo root: replay_csv . demo_6

#include <cstdio>
#include <exception>
#include <map>
#include <string>
#include <vector>

#include "stepfusion/csv.hpp"
#include "stepfusion/pdr.hpp"
#include "stepfusion/recording.hpp"

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: %s <repo_root> <recording, e.g. demo_6>\n", argv[0]);
        return 1;
    }
    const std::string repo_root = argv[1];
    const std::string name = argv[2];

    try {
        // Q and R are the values Python estimated
        const std::map<std::string, double> reference =
            stepfusion::load_key_value_csv(repo_root + "/test_data/reference/" + name + "/summary.csv");

        stepfusion::PdrConfig config;
        config.process_noise_q = reference.at("Q");
        config.measurement_noise_r = reference.at("R");
        config.step_length_m = reference.at("step_length_m");

        stepfusion::Pdr pdr(config);
        const stepfusion::Recording recording = stepfusion::load_recording(repo_root + "/data/raw/" + name);
        const std::vector<stepfusion::StepEvent> steps = stepfusion::replay_recording(recording, pdr);

        const double raw_cd = stepfusion::closing_distance(pdr.raw_position());
        const double fused_cd = stepfusion::closing_distance(pdr.fused_position());

        std::printf("%s (live C++)       : %d steps, raw %.3f m, fused %.3f m, improvement %+.1f%%\n",
                    name.c_str(), pdr.step_count(), raw_cd, fused_cd, (raw_cd - fused_cd) / raw_cd * 100.0);
        std::printf("%s (Python, batch)  : %d steps, raw %.3f m, fused %.3f m, improvement %+.1f%%\n",
                    name.c_str(), static_cast<int>(reference.at("n_steps")), reference.at("raw_closing_m"),
                    reference.at("kf_closing_m"), reference.at("improvement_pct"));
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
