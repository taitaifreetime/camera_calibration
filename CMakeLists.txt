cmake_minimum_required(VERSION 3.8)
project(camera_calibration)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME}
    src/calibration.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
)

include_directories(
  include
)
add_executable(calibrate_monocular src/calibrate_monocular.cpp src/calibration.cpp)
target_link_libraries(calibrate_monocular
    ${OpenCV_LIBS}
    yaml-cpp
)
# target_compile_definitions(calibrate_monocular PRIVATE DEBUG)

add_executable(calibrate_stereo src/calibrate_stereo.cpp src/calibration.cpp)
target_link_libraries(calibrate_stereo
    ${OpenCV_LIBS}
    yaml-cpp
)
# target_compile_definitions(calibrate_stereo PRIVATE DEBUG)

add_executable(disparity_mapping src/disparity_mapping.cpp src/calibration.cpp include/utils/utils.hpp)
target_link_libraries(disparity_mapping
    ${OpenCV_LIBS}
    yaml-cpp
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)
ament_package()
