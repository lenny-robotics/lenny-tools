#pragma once

#include <lenny/tools/Definitions.h>
#include <lenny/tools/Timer.h>

namespace lenny::tools {

class Animator {
public:
    Animator(const uint& numSteps, const double& deltaT);
    ~Animator() = default;

    void update();
    void restart();
    double getTotalTime() const;
    double getCurrentPercentage() const;
    void setCurrentTimeFromPercentage(const double& percentage);
    void drawGui();

public:
    std::reference_wrapper<const uint> numSteps;
    std::reference_wrapper<const double> deltaT;

    bool run = false;
    double waitTime = 1.0;  //[s]
    double currentTime = 0.0;

private:
    tools::Timer timer;
};

}  // namespace lenny::tools