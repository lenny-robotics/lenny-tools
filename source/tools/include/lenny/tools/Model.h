#pragma once

#include <lenny/tools/Definitions.h>
#include <lenny/tools/Typedefs.h>

#include <functional>
#include <optional>

namespace lenny::tools {

class Model {
public:
    //--- Typedefs
    LENNY_GENERAGE_TYPEDEFS(Model);
    typedef std::function<void(tools::Model::UPtr &model, const std::string &filePath)> F_loadModel;

    //--- Constructor
    Model(const std::string &filePath) : filePath(filePath) {}
    virtual ~Model() = default;

    //--- Drawing
    virtual void draw(const Eigen::Vector3d &position, const Eigen::QuaternionD &orientation, const Eigen::Vector3d &scale,
                      const std::optional<Eigen::Vector3d> &color, const double &alpha) const {
        if (f_drawCallback)
            f_drawCallback(filePath, position, orientation, scale, color, alpha);
    }

    //--- Interaction
    struct HitInfo {
        Eigen::Vector3d point, normal;
        double t;
    };
    virtual std::optional<HitInfo> hitByRay(const Eigen::Vector3d &position, const Eigen::QuaternionD &orientation, const Eigen::Vector3d &scale,
                                            const Ray &ray) const {
        return std::nullopt;
    }

public:
    const std::string filePath;
    inline static std::function<void(const std::string &filePath, const Eigen::Vector3d &position, const Eigen::QuaternionD &orientation,
                                     const Eigen::Vector3d &scale, const std::optional<Eigen::Vector3d> &color, const double &alpha)>
        f_drawCallback = nullptr;
};

}  // namespace lenny::tools