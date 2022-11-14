#pragma once

#include <lenny/tools/Json.h>

#include <fstream>
#include <iostream>

namespace test1::test2 {
struct JsonTest {
    enum TYPE { INVALID = -1, BUMBLEBEE = 0, ANT = 1 };

    TYPE type = BUMBLEBEE;
    std::string name = "Test";
    int intVal = 36;
    double doubVal = 0.654;
    Eigen::Vector3d position = Eigen::Vector3d(0.1, 0.2, 0.3);
    Eigen::QuaternionD orientation = Eigen::QuaternionD(lenny::tools::utils::rotX(PI / 4));
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Random(5, 3);

    void setNewValues() {
        type = ANT;
        name = "Bla";
        intVal = 65486;
        doubVal = 54.45457;
        position = Eigen::Vector3d::Random();
        orientation = Eigen::QuaternionD::UnitRandom();
        matrix = Eigen::MatrixXd::Random(8, 6);
    }

    bool compare(const JsonTest& test) {
        if (this->type != test.type)
            return false;
        if (this->name.compare(test.name) != 0)
            return false;
        if (abs(this->intVal - test.intVal) > 0)
            return false;
        if (fabs(this->doubVal - test.doubVal) > 1e-5)
            return false;
        if (!this->position.isApprox(test.position))
            return false;
        if (!this->orientation.isApprox(test.orientation))
            return false;
        if (!this->matrix.isApprox(test.matrix))
            return false;
        return true;
    }

    static void to_json(json& j, const JsonTest& o) {
        TO_JSON(o, type)
        TO_JSON(o, name)
        TO_JSON(o, intVal)
        TO_JSON(o, doubVal)
        TO_JSON(o, position)
        TO_JSON(o, orientation)
        TO_JSON(o, matrix)
    }

    static void from_json(const json& j, JsonTest& o) {
        FROM_JSON(o, type)
        FROM_JSON(o, name)
        FROM_JSON(o, intVal)
        FROM_JSON(o, doubVal)
        FROM_JSON(o, position)
        FROM_JSON(o, orientation)
        FROM_JSON(o, matrix)
    }
};

NLOHMANN_JSON_SERIALIZE_ENUM(JsonTest::TYPE, {{JsonTest::INVALID, nullptr}, {JsonTest::BUMBLEBEE, "BUMBLEBEE"}, {JsonTest::ANT, "ANT"}})
}  // namespace test1::test2

//------------------------------------------------------------------------------------------------------------------

void test_json() {
    const std::string filepath = LENNY_PROJECT_FOLDER "/logs/Test.json";
    test1::test2::JsonTest test_save, test_load;

    {  //Save to file
        std::ofstream file(filepath);
        if (!file.is_open())
            LENNY_LOG_ERROR("File %s could not be opened", filepath.c_str());

        json js;
        test1::test2::JsonTest::to_json(js, test_save);
        file << std::setw(2) << js << std::endl;
        file.close();

        std::cout << js << std::endl;
    }

    {
        test_load.setNewValues();

        json js_tmp;
        test1::test2::JsonTest::to_json(js_tmp, test_load);
        std::cout << js_tmp << std::endl;

        if (!test_save.compare(test_load))
            std::cout << "First test passed!" << std::endl;
        else
            std::cout << "First test failed" << std::endl;

        std::ifstream file(filepath);
        if (!file.is_open())
            LENNY_LOG_ERROR("File %s could not be opened", filepath.c_str());

        json js;
        file >> js;
        test1::test2::JsonTest::from_json(js, test_load);
        file.close();

        std::cout << js << std::endl;

        if (test_save.compare(test_load))
            std::cout << "Second test passed!" << std::endl;
        else
            std::cout << "Second test failed" << std::endl;
    }
}