#pragma once

#include <lenny/tools/Definitions.h>

#include <map>
#include <optional>

namespace lenny::tools {

template <typename T>
class Trajectory {
public:
    Trajectory() = default;
    ~Trajectory() = default;

    /**
     * Helpers
     */
    void clear() {
        entries.clear();
    }

    void addEntry(const double& time, const T& value) {
        entries.insert({time, value});
    }

    const std::map<double, T>& getEntries() const {
        return entries;
    }

    /**
     * Linear interpolation
     */
    void getLinearInterpolation(T& value, const double& time) const {
        auto i_lower(entries.end()), i_upper(entries.end());
        if (!getIterators(i_lower, i_upper, value, time))
            return;

        const double t = (time - i_lower->first) / (i_upper->first - i_lower->first);
        value = i_upper->second * t + i_lower->second * (1.0 - t);
    }

    T getLinearInterpolation(const double& time) const {
        T value;
        getLinearInterpolation(value, time);
        return value;
    }

    /**
     * Catmull rom spline
     */
    void getSplineInterpolation(T& value, const double& time) const {
        auto i_lower(entries.end()), i_upper(entries.end());
        if (!getIterators(i_lower, i_upper, value, time))
            return;

        auto getSlopeEstimate = [&](const std::map<double, T>::const_iterator& iter) -> T {
            auto iter_before(iter), iter_after(iter);
            iter_before--;
            iter_after++;
            if (iter_before != entries.end() && iter_after != entries.end()) {  //Both valid
                const T slope_before = (iter->second - iter_before->second) / (iter->first - iter_before->first);
                const T slope_after = (iter_after->second - iter->second) / (iter_after->first - iter->first);
                return 0.5 * (slope_before + slope_after);
            } else if (iter_before == entries.end() && iter_after != entries.end()) {  //iter_after valid
                return (iter_after->second - iter->second) / (iter_after->first - iter->first);
            } else if (iter_before != entries.end() && iter_after == entries.end()) {  //iter_before valid
                return (iter->second - iter_before->second) / (iter->first - iter_before->first);
            }
            return T(0);  //Both invalid
        };

        const double dt = (i_upper->first - i_lower->first);
        const double t = (time - i_lower->first) / dt;
        const double t2 = t * t;
        const double t3 = t * t * t;

        const T& p1 = i_lower->second;
        const T& p2 = i_upper->second;

        const T m1 = getSlopeEstimate(i_lower) * dt;
        const T m2 = getSlopeEstimate(i_upper) * dt;

        value = p1 * (2.0 * t3 - 3.0 * t2 + 1.0) + m1 * (t3 - 2.0 * t2 + t) + p2 * (-2.0 * t3 + 3.0 * t2) + m2 * (t3 - t2);
    }

    T getSplineInterpolation(const double& time) const {
        T value;
        getSplineInterpolation(value, time);
        return value;
    }

protected:
    //Returns true in case both iterators are valid, and false if not and sets appropriate value
    bool getIterators(std::map<double, T>::const_iterator& i_lower, std::map<double, T>::const_iterator& i_upper, T& value, const double& time) const {
        i_upper = entries.upper_bound(time);
        i_lower = i_upper;
        i_lower--;
        if (i_upper != entries.end() && i_lower == entries.end()) {  //Upper valid
            value = i_upper->second;
            return false;
        } else if (i_upper == entries.end() && i_lower != entries.end()) {  //Lower valid
            value = i_lower->second;
            return false;
        } else if (i_upper == entries.end() && i_lower == entries.end()) {  //Both invalid
            value = T(0);
            return false;
        }
        return true;
    }

private:
    std::map<double, T> entries;
};

template <class T>
inline std::ostream& operator<<(std::ostream& os, const Trajectory<T>& trajectory) {
    os << "Start of Trajectory:" << std::endl;
    for (const auto& [time, value] : trajectory.getEntries())
        os << "-> Value at time " << time << ": " << std::endl << value << std::endl;
    os << "End of Trajectory";
    return os;
}

typedef Trajectory<double> Trajectory1d;
typedef Trajectory<Eigen::Vector3d> Trajectory3d;
typedef Trajectory<Eigen::Vector6d> Trajectory6d;
typedef Trajectory<Eigen::VectorXd> TrajectoryXd;

}  // namespace lenny::tools