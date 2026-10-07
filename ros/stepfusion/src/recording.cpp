#include "stepfusion/recording.hpp"

#include <stdexcept>

namespace stepfusion {

Recording load_recording(const std::string& data_dir) {
    Recording recording;
    recording.accel = load_sensor_csv(data_dir + "/Accelerometer.csv");
    recording.gyro = load_sensor_csv(data_dir + "/Gyroscope.csv");
    recording.mag = load_sensor_csv(data_dir + "/Magnetometer.csv");
    return recording;
}

std::vector<StepEvent> replay_recording(const Recording& recording, Pdr& pdr) {
    const std::vector<SensorSample>& accel = recording.accel;
    const std::vector<SensorSample>& gyro = recording.gyro;
    const std::vector<SensorSample>& mag = recording.mag;

    if (accel.size() != gyro.size()) {
        throw std::runtime_error("accelerometer and gyroscope have different sample counts");
    }

    std::vector<StepEvent> steps;
    size_t m = 0;

    for (size_t i = 0; i < gyro.size(); ++i) {
        if (accel[i].t != gyro[i].t) {
            throw std::runtime_error("accelerometer and gyroscope timestamps differ");
        }

        while (m < mag.size() && mag[m].t <= gyro[i].t) {
            pdr.add_mag(mag[m].t, mag[m].x, mag[m].y);
            ++m;
        }

        const std::optional<StepEvent> event =
            pdr.add_imu(gyro[i].t, gyro[i].z, accel[i].x, accel[i].y, accel[i].z);
        if (event) {
            steps.push_back(*event);
        }
    }
    return steps;
}

}  // namespace stepfusion
