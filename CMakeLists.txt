cmake_minimum_required(VERSION 3.8)
project(camera_calibration)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME}
    src/camera_calibration/calibration.cpp
    src/utils/computer_vision.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    yaml-cpp
)

include_directories(
  include
)
add_executable(calibrate_monocular src/calibrate_monocular.cpp)
target_link_libraries(calibrate_monocular ${PROJECT_NAME})
# target_compile_definitions(calibrate_monocular PRIVATE DEBUG)

add_executable(calibrate_stereo src/calibrate_stereo.cpp)
target_link_libraries(calibrate_stereo ${PROJECT_NAME})
# target_compile_definitions(calibrate_stereo PRIVATE DEBUG)

add_executable(rectification_mapping src/rectification_mapping.cpp)
target_link_libraries(rectification_mapping ${PROJECT_NAME})
target_compile_definitions(rectification_mapping PRIVATE DEBUG)

add_executable(disparity_mapping src/disparity_mapping.cpp)
target_link_libraries(disparity_mapping ${PROJECT_NAME})
target_compile_definitions(disparity_mapping PRIVATE DEBUG)

add_executable(capture_monocular src/capture_monocular.cpp)
target_link_libraries(capture_monocular ${PROJECT_NAME})
# target_compile_definitions(capture_monocular PRIVATE DEBUG)

add_executable(capture_stereo src/capture_stereo.cpp)
target_link_libraries(capture_stereo ${PROJECT_NAME})
# target_compile_definitions(capture_stereo PRIVATE DEBUG)

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
