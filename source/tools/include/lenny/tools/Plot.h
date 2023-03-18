#pragma once

#include <lenny/tools/Json.h>
#include <lenny/tools/Typedefs.h>

#include <array>
#include <fstream>
#include <string>
#include <vector>

namespace lenny::tools {

template <typename T>
class Plot {
public:
    //Line
    struct Line {
        std::string label;
        std::function<float(T)> getter;  //e.g. [](const Vector3d &d) { return (float)d.y; }
        std::optional<std::array<float, 3>> color = std::nullopt;
        int marker = -1;  //No marker
    };

    //Typedefs
    LENNY_GENERAGE_TYPEDEFS(Plot);
    typedef std::vector<tools::Plot<T>::UPtr> List;
    typedef std::function<void(List &plots, const std::string &title, const std::string &xLabel, const std::string &yLabel, int maxSize)> F_addPlot;

    //Constructor
    explicit Plot(const std::string &title, const std::string &xLabel, const std::string &yLabel, int maxSize = 1000)
        : title(title), xLabel(xLabel), yLabel(yLabel), maxSize(maxSize) {
        data.reserve(maxSize);
    }
    virtual ~Plot() = default;

    //Clear data
    void clearData() {
        data.clear();
        offset = 0;
    }

    //Add data entry
    void addData(float x, const T &y) {
        if (data.size() < maxSize) {
            data.push_back({x, y});
        } else {
            data[offset] = {x, y};
            offset = (offset + 1) % maxSize;
        }
    }

    //Add line specification
    void addLineSpec(const Line &lineSpec) {
        lineSpecs.push_back(lineSpec);
    }

    //Draw plot
    virtual void draw() = 0;

    //Save to file
    bool saveDataToFile(const std::string &filePath) const {
        //Check file extensions
        if (!tools::utils::checkFileExtension(filePath, "json")) {
            LENNY_LOG_WARNING("File `%s` should be a `json` file... Abort!", filePath.c_str())
            return false;
        }

        //Open file
        std::ofstream file(filePath);
        if (!file.is_open()) {
            LENNY_LOG_WARNING("File `%s` could not be opened... Abort!", filePath.c_str());
            return false;
        }

        //Setup json
        json js;

        //Add data
        for (const auto &[x, y] : data) {
            json js_tmp;
            js_tmp["x"] = x;
            js_tmp["y"] = y;
            js.push_back(js_tmp);
        }

        //Stream to file
        file << std::setw(2) << js << std::endl;

        //Close file
        file.close();

        //Wrap up
        LENNY_LOG_INFO("Successfully saved plot data into file `%s`", filePath.c_str());
        return true;
    }

    //Get data
    const std::vector<std::pair<float, T>> &getData() const {
        return data;
    }

protected:
    //Helpers
    int getSize() const {
        if (data.size() < maxSize)
            return data.size();
        return maxSize;
    }

    const std::pair<float, T> &getData(int idx) {
        if (data.size() < maxSize)
            return data[idx];
        return data[(offset + idx) % maxSize];
    }

    float getXBegin() const {
        if (data.size() == 0)
            return -1;
        if (data.size() < maxSize)
            return data[0].first;
        return data[offset].first;
    }

    float getXEnd() const {
        if (data.size() == 0)
            return 1;
        if (data.size() < maxSize)
            return data.back().first;
        return data[offset - 1].first;
    }

protected:
    //Helpers
    std::string title, xLabel, yLabel;  //Set by constructor
    int maxSize;                        //Set by constructor
    bool fitAxisX = true;
    bool fitAxisY = true;
    float scaleAxisX = 1.f;
    int offset = 0;
    int index = 0;

    //Data
    std::vector<std::pair<float, T>> data;
    std::vector<Line> lineSpecs;
};

}  // namespace lenny::tools