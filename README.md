# Camera Calibration in C++

## Build
```
mkdir build && cd build 
cmake ..
make
```

## Usage 
```
./calibrate_monocular checker_width checker_height square_size[m] ../images/dir 

./calibrate_stereo checker_width checker_height square_size[m] ../images/dir1 ../images/dir2

./disparity_mapping left_camera_parameter_file right_camera_parameter_file left_camera_id right_camera_id left_camera_fps right_camera_fps
```

## Tips
- If the RMS error is large for stereo camera calibration, even though each monocular camera calibration is successful, you should check the checkerboard detection results. The order of the checkerboard squares may differ between the two cameras. If such a difference exists, delete the set of the images and calibrate again.

## Sample calibrated parameters
See 
- [monocular camera parameter](./monocular_camera_parameters.yaml) 
- stereo camera
    - [right camera parameter](./right_camera_parameters.yaml)
    - [left camera parameter](./left_camera_parameters.yaml)