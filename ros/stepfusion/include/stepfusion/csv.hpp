#pragma once

#include <map>
#include <string>
#include <vector>

namespace stepfusion {

struct CsvTable {
    std::vector<std::string> columns;
    std::vector<std::vector<double>> rows;

    int column_index(const std::string& name) const; // -1 if missing
    std::vector<double> column(const std::string& name) const;
};

CsvTable load_csv(const std::string& path);

// For "key,value" files like the reference summary.csv
std::map<std::string, double> load_key_value_csv(const std::string& path);

struct SensorSample {
    double t; // seconds_elapsed
    double x;
    double y;
    double z;
};

// Columns looked up by name since Sensor Logger stores them as z, y, x
std::vector<SensorSample> load_sensor_csv(const std::string& path);

}  // namespace stepfusion
