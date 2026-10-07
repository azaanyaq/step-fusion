#pragma once

#include <string>
#include <vector>

#include "stepfusion/csv.hpp"
#include "stepfusion/pdr.hpp"

namespace stepfusion {

struct Recording {
    std::vector<SensorSample> accel;
    std::vector<SensorSample> gyro;
    std::vector<SensorSample> mag;
};

Recording load_recording(const std::string& data_dir);

// Feeds a recording through in timestamp order (mag first on equal timestamps), returns every step
std::vector<StepEvent> replay_recording(const Recording& recording, Pdr& pdr);

}  // namespace stepfusion
