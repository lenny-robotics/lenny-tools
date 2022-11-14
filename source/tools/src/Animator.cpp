#include <lenny/tools/Animator.h>
#include <lenny/tools/Gui.h>

namespace lenny::tools {

Animator::Animator(const uint& numSteps, const double& deltaT) : numSteps(numSteps), deltaT(deltaT) {}

void Animator::update() {
    if (!run)
        return;

    currentTime += timer.time();
    timer.restart();
    if (currentTime >= getTotalTime() + waitTime)
        restart();
}

void Animator::restart() {
    currentTime = -waitTime;
    timer.restart();
}

double Animator::getTotalTime() const {
    return (double)numSteps * deltaT;
}

double Animator::getCurrentPercentage() const {
    return currentTime / getTotalTime();
}

void Animator::setCurrentTimeFromPercentage(const double& percentage) {
    currentTime = percentage * getTotalTime();
}

void Animator::drawGui() {
    using tools::Gui;
    if (Gui::I->TreeNode("Animation")) {
        Gui::I->Checkbox("Run", run);
        if (Gui::I->Button("Restart"))
            restart();
        Gui::I->Slider("Current Time", currentTime, -waitTime, getTotalTime() + waitTime);
        Gui::I->Slider("Wait Time", waitTime, 0.0, 5.0);

        Gui::I->TreePop();
    }
}

}  // namespace lenny::tools