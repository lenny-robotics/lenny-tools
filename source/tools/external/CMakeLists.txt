# set new cmake policy
set_new_cmake_policy()

# eigen
if (NOT TARGET eigen)
    FetchContent_Declare(
            eigen #
            URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip #
            URL_HASH MD5=994092410ba29875184f7725e0371596 #
    )
    fetch_repository(eigen)
    add_library(eigen INTERFACE)
    if(NOT TARGET Eigen3::Eigen)
        add_library(Eigen3::Eigen ALIAS eigen)
    endif()
    target_include_directories(eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})
endif ()

# magic_enum
if (NOT TARGET magic_enum)
    add_subdirectory(magic_enum)
endif ()

# json
if (NOT TARGET nlohmann_json)
    FetchContent_Declare(
            json #
            URL https://github.com/nlohmann/json/releases/download/v3.10.5/include.zip #
            URL_HASH MD5=3cea575f0b559e2609507a5ee6469dea #
    )
    fetch_repository(json)
    add_library(nlohmann_json INTERFACE)
    add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
    target_include_directories(nlohmann_json SYSTEM INTERFACE $<BUILD_INTERFACE:${json_SOURCE_DIR}/include>)
endif ()

# googletest
if (NOT TARGET googletest)
    FetchContent_Declare(
            googletest #
            URL https://github.com/google/googletest/archive/refs/tags/v1.13.0.zip #
            URL_HASH MD5=A1279C6FB5BF7D4A5E0D0B2A4ADB39AC #
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE) #For Windows: Prevent overriding the parent project's compiler/linker settings
    option(BUILD_GMOCK "Builds the googlemock subproject" OFF)
    option(INSTALL_GTEST "Enable installation of googletest" OFF)
    option(GTEST_HAS_ABSL "Use Abseil and RE2. Requires Abseil and RE2 to be separately added to the build." OFF)
    message(STATUS "Fetching repository 'googletest' ...")
    FetchContent_MakeAvailable(googletest)
endif ()