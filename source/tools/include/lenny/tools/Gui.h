#pragma once

#include <lenny/tools/Transformation.h>

#include <memory>

namespace lenny::tools {

class Gui {
public:
    //--- Pointer
    typedef std::unique_ptr<Gui> UPtr;

    //--- Constructor
    Gui() = default;
    virtual ~Gui() = default;

    //--- Handling
    virtual bool Begin(const char* name, bool* p_open = nullptr, int flags = 0) const {
        return false;
    }
    virtual void End() const {}

    virtual bool TreeNode(const char* label) const {
        return false;
    }
    virtual void TreePop() const {}

    virtual void PushItemWidth(float item_width) const {}
    virtual void PopItemWidth() const {}

    virtual void SameLine(float offset_from_start_x = 0.f, float spacing = -1.f) const {}
    virtual void NewLine() const {}

    //--- Sliders
    virtual bool Slider(const char* label, double& value, const double& min, const double& max, const char* format = nullptr, int flags = 0) const {
        return false;
    }
    virtual bool Slider(const char* label, float& value, const float& min, const float& max, const char* format = nullptr, int flags = 0) const {
        return false;
    }
    virtual bool Slider(const char* label, int& value, const int& min, const int& max, const char* format = nullptr, int flags = 0) const {
        return false;
    }
    virtual bool Slider(const char* label, uint& value, const uint& min, const uint& max, const char* format = nullptr, int flags = 0) const {
        return false;
    }
    virtual bool Slider(const char* label, Eigen::Vector2d& value, const Eigen::Vector2d& min, const Eigen::Vector2d& max, const char* format = nullptr,
                        int flags = 0) const {
        return false;
    }
    virtual bool Slider(const char* label, Eigen::Vector3d& value, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const char* format = nullptr,
                        int flags = 0) const {
        return false;
    }
    virtual bool Slider(const char* label, Eigen::VectorXd& value, const Eigen::VectorXd& min, const Eigen::VectorXd& max, const char* format = nullptr,
                        int flags = 0) const {
        return false;
    }
    virtual bool Slider(const char* label, Eigen::Matrix3d& value, const char* format = nullptr, int flags = 0) const {
        return false;
    }
    virtual bool Slider(const char* label, Eigen::QuaternionD& value, const char* format = nullptr, int flags = 0) const {
        return false;
    }

    //--- Inputs
    virtual bool Input(const char* label, double& value, const char* format = "%.6f", int flags = 0) const {
        return false;
    }
    virtual bool Input(const char* label, float& value, const char* format = "%.6f", int flags = 0) const {
        return false;
    }
    virtual bool Input(const char* label, int& value, const char* format = "%.6f", int flags = 0) const {
        return false;
    }
    virtual bool Input(const char* label, uint& value, const char* format = "%.6f", int flags = 0) const {
        return false;
    }
    virtual bool Input(const char* label, Eigen::Vector2d& value, const char* format = "%.6f", int flags = 0) const {
        return false;
    }
    virtual bool Input(const char* label, Eigen::Vector3d& value, const char* format = "%.6f", int flags = 0) const {
        return false;
    }
    virtual bool Input(const char* label, Eigen::VectorXd& value, const char* format = "%.6f", int flags = 0) const {
        return false;
    }
    virtual bool Input(const char* label, Eigen::Matrix3d& value, const char* format = "%.6f", int flags = 0) const {
        return false;
    }
    virtual bool Input(const char* label, Eigen::QuaternionD& value, const char* format = "%.6f", int flags = 0) const {
        return false;
    }
    virtual bool Input(const char* label, Transformation& trafo, const char* format = "%.6f") const {
        return false;
    }

    //--- Checkbox and buttons
    virtual bool Checkbox(const char* label, bool& value) const {
        return false;
    }

    virtual bool Button(const char* label) const {
        return false;
    }
    virtual bool ToggleButton(const char* str_id, bool& v) const {
        return false;
    }

    //--- Text
    virtual void Text(const char* fmt, ...) const {}
    virtual void TextColored(const Eigen::Vector4d& color, const char* fmt, ...) const {}

    //--- Color picker
    virtual bool ColorPicker3(const char* label, Eigen::Vector3d& color) const {
        return false;
    }
    virtual bool ColorPicker4(const char* label, Eigen::Vector4d& color) const {
        return false;
    }

public:
    //--- Static member
    static UPtr I;
};

}  // namespace lenny::tools
