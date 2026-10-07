#include "stepfusion/csv.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace stepfusion {

namespace {

std::vector<std::string> split_line(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

std::ifstream open_or_throw(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("could not open " + path);
    }
    return file;
}

}  // namespace

int CsvTable::column_index(const std::string& name) const {
    for (size_t i = 0; i < columns.size(); ++i) {
        if (columns[i] == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

std::vector<double> CsvTable::column(const std::string& name) const {
    const int index = column_index(name);
    if (index < 0) {
        throw std::runtime_error("no column named " + name);
    }

    std::vector<double> values;
    values.reserve(rows.size());
    for (const std::vector<double>& row : rows) {
        values.push_back(row[index]);
    }
    return values;
}

CsvTable load_csv(const std::string& path) {
    std::ifstream file = open_or_throw(path);
    CsvTable table;

    std::string line;
    if (!std::getline(file, line)) {
        throw std::runtime_error(path + " is empty");
    }
    table.columns = split_line(line);

    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        const std::vector<std::string> fields = split_line(line);
        if (fields.size() != table.columns.size()) {
            throw std::runtime_error(path + ": row has wrong number of fields: " + line);
        }

        std::vector<double> row;
        row.reserve(fields.size());
        for (const std::string& field : fields) {
            row.push_back(std::stod(field));
        }
        table.rows.push_back(row);
    }
    return table;
}

std::map<std::string, double> load_key_value_csv(const std::string& path) {
    std::ifstream file = open_or_throw(path);
    std::map<std::string, double> values;

    std::string line;
    std::getline(file, line); // Skip header
    while (std::getline(file, line)) {
        const std::vector<std::string> fields = split_line(line);
        if (fields.size() == 2) {
            values[fields[0]] = std::stod(fields[1]);
        }
    }
    return values;
}

std::vector<SensorSample> load_sensor_csv(const std::string& path) {
    const CsvTable table = load_csv(path);
    const int t_col = table.column_index("seconds_elapsed");
    const int x_col = table.column_index("x");
    const int y_col = table.column_index("y");
    const int z_col = table.column_index("z");
    if (t_col < 0 || x_col < 0 || y_col < 0 || z_col < 0) {
        throw std::runtime_error(path + " is missing a seconds_elapsed/x/y/z column");
    }

    std::vector<SensorSample> samples;
    samples.reserve(table.rows.size());
    for (const std::vector<double>& row : table.rows) {
        samples.push_back({row[t_col], row[x_col], row[y_col], row[z_col]});
    }
    return samples;
}

}  // namespace stepfusion
