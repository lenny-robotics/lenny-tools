#pragma once

#include <lenny/tools/Utils.h>

#include <optional>

namespace lenny::tools {

template <typename T>
class Trajectory {
public:
    Trajectory() = default;
    Trajectory(const Trajectory& other) {
        lastIndex = other.lastIndex;
        entries.clear();
        entries.reserve(other.entries.size());
        for (int i = 0; i < (int)other.entries.size(); i++)
            entries.push_back(other.entries.at(i));
    }
    virtual ~Trajectory() = default;

    void clear() {
        lastIndex = 0;
        entries.clear();
    }

    void addEntry(const double& time, const T& value) {
        const int index = getFirstLargerIndex(time);
        entries.insert(entries.begin() + index, std::pair<T, double>{value, time});
    }

    const std::vector<std::pair<T, double>>& getEntries() const {
        return entries;
    }

    void getLinearInterpolation(T& value, const double& time) const {
        std::optional<T> testVal;
        checkTimeRange(testVal, time);
        if (testVal.has_value()) {
            value = testVal.value();
            return;
        }
        const int index = getFirstLargerIndex(time);
        const double t = compute_t(time, index);
        value = entries.at(index).first * t + entries.at(index - 1).first * (1.0 - t);
    }

    T getLinearInterpolation(const double& time) const {
        T value;
        getLinearInterpolation(value, time);
        return value;
    }

    double getClosestLinearlyInterpolatedTime(const T& value, std::optional<std::pair<double, double>> timeFrame = std::nullopt) const {
        const int size = (int)entries.size();
        if (size == 0)
            return 0.0;

        //Find indices for time frame
        std::pair<int, int> indexRange = {0, size - 1};
        if (timeFrame.has_value()) {
            if (timeFrame.value().first >= timeFrame.value().second) {
                timeFrame.value().first = entries.at(0).second;
                timeFrame.value().second = entries.at(size - 1).second;
            }

            utils::boundToRange(timeFrame.value().first, entries.at(0).second, entries.at(size - 1).second);
            utils::boundToRange(timeFrame.value().second, entries.at(0).second, entries.at(size - 1).second);
            indexRange.first = getFirstLargerIndex(timeFrame.value().first) - 1;
            indexRange.second = getFirstLargerIndex(timeFrame.value().second);
            utils::boundToRange(indexRange.first, 0, size - 1);
            utils::boundToRange(indexRange.second, 0, size - 1);

            if (indexRange.first > indexRange.second)
                return 0.0;
        }

        std::pair<int, int> indices = {indexRange.first, indexRange.second};
        double dist_min = HUGE_VALF;
        double lambda = 0.0;
        auto findIndices = [&](const int firstIndex, const int secondIndex) -> void {
            const auto [dist, lamb] = pointToLineSegmentDistance(value, entries.at(firstIndex).first, entries.at(secondIndex).first);
            if (dist <= dist_min) {
                dist_min = dist;
                lambda = lamb;
                indices = {firstIndex, secondIndex};
            }
        };

        for (int i = indexRange.first; i < indexRange.second; i++)
            findIndices(i, i + 1);

        if (indexRange.first == 0)
            findIndices(0, 0);

        if (indexRange.second == size - 1)
            findIndices(size - 1, size - 1);

        if (indices.first == indices.second)
            return entries.at(indices.first).second;

        const double t_im1 = entries.at(indices.first).second;
        const double t_i = entries.at(indices.second).second;
        return t_i * lambda + t_im1 * (1.0 - lambda);
    }

    /**
     * Catmull rom spline
     */
    void getSplineInterpolation(T& value, const double& time, const bool equalEndpointSlopes = false) const {
        std::optional<T> testVal;
        checkTimeRange(testVal, time);
        if (testVal.has_value()) {
            value = testVal.value();
            return;
        }

        const int index = getFirstLargerIndex(time);
        const double t = compute_t(time, index);

        const double t2 = t * t;
        const double t3 = t * t * t;

        const T& p1 = entries.at(index - 1).first;
        const T& p2 = entries.at(index).first;

        auto getSlopeEstimateAtEntry = [&](const int index, const bool equalEndpointSlopes) -> T {
            const int size = (int)entries.size();
            if (size < 2)
                return T(0);

            if (index == 0 || index == size - 1) {
                const T startSlope = (entries.at(1).first - entries.at(0).first) / (entries.at(1).second - entries.at(0).second);
                const T endSlope = (entries.at(size - 1).first - entries.at(size - 2).first) / (entries.at(size - 1).second - entries.at(size - 2).second);

                if (equalEndpointSlopes)
                    return 0.5 * (startSlope + endSlope);

                if (index == 0)
                    return startSlope;

                return endSlope;
            }

            const T slopeBefore = (entries.at(index).first - entries.at(index - 1).first) / (entries.at(index).second - entries.at(index - 1).second);
            const T slopeAfter = (entries.at(index + 1).first - entries.at(index).first) / (entries.at(index + 1).second - entries.at(index).second);

            return 0.5 * (slopeBefore + slopeAfter);
        };

        const T m1 = getSlopeEstimateAtEntry(index - 1, equalEndpointSlopes) * (entries.at(index).second - entries.at(index - 1).second);
        const T m2 = getSlopeEstimateAtEntry(index, equalEndpointSlopes) * (entries.at(index).second - entries.at(index - 1).second);

        value = p1 * (2.0 * t3 - 3.0 * t2 + 1.0) + m1 * (t3 - 2.0 * t2 + t) + p2 * (-2.0 * t3 + 3.0 * t2) + m2 * (t3 - t2);
    }

    T getSplineInterpolation(const double& time, const bool equalEndpointSlopes = false) const {
        T value;
        getSplineInterpolation(value, time, equalEndpointSlopes);
        return value;
    }

protected:
    void checkTimeRange(std::optional<T>& value, const double& time) const {
        value = std::nullopt;
        const int size = (int)entries.size();
        if (size == 0)
            value = T(0);
        else if (time <= entries.at(0).second)
            value = entries.at(0).first;
        else if (time >= entries.at(size - 1).second)
            value = entries.at(size - 1).first;
    }

    double compute_t(const double& time, const int& index) const {
        return (time - entries.at(index - 1).second) / (entries.at(index).second - entries.at(index - 1).second);
    }

    int getFirstLargerIndex(const double& time) const {
        const int size = (int)entries.size();
        if (size == 0)
            return 0;

        if (time < entries.at((lastIndex + size - 1) % size).second)
            lastIndex = 0;

        for (int i = 0; i < size; i++) {
            const int index = (i + lastIndex) % size;
            if (time < entries.at(index).second) {
                lastIndex = index;
                return index;
            }
        }

        return size;
    }

    //Returns [distance, lambda]
    inline static std::pair<double, double> pointToLineSegmentDistance(const double& P0, const double& P1, const double& P2) {
        double lambda = fabs(P2 - P1) > 1e-6 ? (P1 - P0) / fabs(P2 - P1) : 0.0;
        utils::boundToRange(lambda, 0.0, 1.0);
        const double P12 = P1 + lambda * (P2 - P1);
        return {fabs(P0 - P12), lambda};
    }

    //Returns [distance, lambda]
    inline static std::pair<double, double> pointToLineSegmentDistance(const Eigen::Matrix<double, -1, 1>& P0, const Eigen::Matrix<double, -1, 1>& P1,
                                                                       const Eigen::Matrix<double, -1, 1>& P2) {
        double lambda = (P2 - P1).squaredNorm() > 1e-6 ? -((P1 - P0).dot(P2 - P1)) / (P2 - P1).squaredNorm() : 0.0;
        utils::boundToRange(lambda, 0.0, 1.0);
        const Eigen::Matrix<double, -1, 1> P12 = P1 + lambda * (P2 - P1);
        return {(P0 - P12).norm(), lambda};
    }

protected:
    std::vector<std::pair<T, double>> entries;
    mutable int lastIndex = 0;
};

template <class T>
inline std::ostream& operator<<(std::ostream& os, const Trajectory<T>& trajectory) {
    os << "Start of Trajectory:" << std::endl;
    for (const auto& entry : trajectory.getEntries())
        os << "\t" << entry.second << ":\t" << entry.first << std::endl;
    os << "End of Trajectory";
    return os;
}

typedef Trajectory<double> Trajectory1d;
typedef Trajectory<Eigen::Vector3d> Trajectory3d;
typedef Trajectory<Eigen::Vector6d> Trajectory6d;
typedef Trajectory<Eigen::VectorXd> TrajectoryXd;

}  // namespace lenny::tools