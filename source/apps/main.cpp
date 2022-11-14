#include "Test_EulerAngle.h"
#include "Test_FiniteDifference.h"
#include "Test_Json.h"
#include "Test_Logger.h"
#include "Test_Tensor.h"
#include "Test_Timer.h"
#include "Test_Trajectory.h"
#include "Test_Transformation.h"
#include "Test_Utils.h"

int main() {
    test_utils();
    test_timer();
    test_logger();
    test_json();
    test_transformation();
    test_finitedifference();
    test_trajectory();
    test_tensor();
    test_eulerangle();

    return 0;
}