cmake_minimum_required(VERSION 4.0)
project(helios-physics LANGUAGES CXX)

if(PROJECT_IS_TOP_LEVEL)
    set(CMAKE_CXX_STANDARD 23)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

include(CTest)
include(FetchContent)

if(CMAKE_IFC_OUTPUT_DIRECTORY)
    file(MAKE_DIRECTORY "${CMAKE_IFC_OUTPUT_DIRECTORY}")
endif()

set(HELIOS_MATH_LOCAL_PATH "${CMAKE_CURRENT_LIST_DIR}/../helios-math" CACHE PATH "Local path to helios-math repository")
set(HELIOS_ENGINE_LOCAL_PATH "${CMAKE_CURRENT_LIST_DIR}/../helios-engine" CACHE PATH "Local path to helios-engine repository")
set(HELIOS_ECS_LOCAL_PATH "${CMAKE_CURRENT_LIST_DIR}/../helios-ecs" CACHE PATH "Local path to helios-ecs repository")

if(NOT TARGET helios::math)
    if(EXISTS "${HELIOS_MATH_LOCAL_PATH}/CMakeLists.txt")
        add_subdirectory("${HELIOS_MATH_LOCAL_PATH}" "${CMAKE_BINARY_DIR}/_deps/helios-math-build")
    else()
        FetchContent_Declare(helios_math
                GIT_REPOSITORY https://github.com/thorstensuckow/helios-math.git
                GIT_TAG main
        )
        FetchContent_MakeAvailable(helios_math)
    endif()
endif()

if(NOT TARGET helios::ecs)
    if(EXISTS "${HELIOS_ECS_LOCAL_PATH}/CMakeLists.txt")
        add_subdirectory("${HELIOS_ECS_LOCAL_PATH}" "${CMAKE_BINARY_DIR}/_deps/helios-ecs-build")
    else()
        FetchContent_Declare(helios_ecs
                GIT_REPOSITORY https://github.com/thorstensuckow/helios-ecs.git
                GIT_TAG main
        )
        FetchContent_MakeAvailable(helios_ecs)
    endif()
endif()

if(NOT TARGET helios::engine)
    if(EXISTS "${HELIOS_ENGINE_LOCAL_PATH}/CMakeLists.txt")
        add_subdirectory("${HELIOS_ENGINE_LOCAL_PATH}" "${CMAKE_BINARY_DIR}/_deps/helios-engine-build")
    else()
        FetchContent_Declare(helios_engine
                GIT_REPOSITORY https://github.com/thorstensuckow/helios-engine.git
                GIT_TAG main
        )
        FetchContent_MakeAvailable(helios_engine)
    endif()
endif()

file(GLOB_RECURSE CPP_FILES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

file(GLOB_RECURSE MODULE_INTERFACE_FILES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.ixx")

list(FILTER CPP_FILES EXCLUDE REGEX "/CMakeFiles/|/CMakeTmp/|/cmake_build_|/build/")
list(FILTER MODULE_INTERFACE_FILES EXCLUDE REGEX "/CMakeFiles/|/CMakeTmp/|/cmake_build_|/build/")

add_library(helios_physics STATIC)
add_library(helios::physics ALIAS helios_physics)
set_property(TARGET helios_physics PROPERTY CXX_SCAN_FOR_MODULES ON)
target_compile_features(helios_physics PUBLIC cxx_std_23)

target_sources(helios_physics PRIVATE ${CPP_FILES})

if(MODULE_INTERFACE_FILES)
    target_sources(helios_physics PUBLIC
            FILE_SET modules TYPE CXX_MODULES
            BASE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src"
            FILES ${MODULE_INTERFACE_FILES}
    )
endif()

target_link_libraries(helios_physics
        PUBLIC
        $<BUILD_INTERFACE:helios::math>
        $<BUILD_INTERFACE:helios::engine>
        $<BUILD_INTERFACE:helios::ecs>
        $<INSTALL_INTERFACE:helios::math>
        $<INSTALL_INTERFACE:helios::engine>
        $<INSTALL_INTERFACE:helios::ecs>
)

target_include_directories(helios_physics PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

set(HELIOS_PHYSICS_BUILD_TESTS_DEFAULT OFF)
if(PROJECT_IS_TOP_LEVEL)
    set(HELIOS_PHYSICS_BUILD_TESTS_DEFAULT ON)
endif()
option(HELIOS_PHYSICS_BUILD_TESTS "Build helios-physics tests" ${HELIOS_PHYSICS_BUILD_TESTS_DEFAULT})

if(HELIOS_PHYSICS_BUILD_TESTS AND BUILD_TESTING)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG v1.17.0
    )
    FetchContent_MakeAvailable(googletest)

    file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.test.cpp")

    add_executable(helios-physics-tests ${TEST_SOURCES})
    target_link_libraries(helios-physics-tests PRIVATE helios_physics GTest::gtest_main)

    include(GoogleTest)
    gtest_discover_tests(helios-physics-tests)
endif()
