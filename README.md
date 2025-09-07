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

./rectification_mapping left_camera_parameter_file right_camera_parameter_file left_camera_id right_camera_id fps left_flip_method right_flip_method

./disparity_mapping left_camera_parameter_file right_camera_parameter_file left_camera_id right_camera_id fps left_flip_method right_flip_method
```

## Store calibration images
```
# push key s to save

./capture_monocular camera_id width height fps flip_method ../images/dir1

./capture_stereo camera1_id camera2_id width height fps flip_method1 flip_method2 ../images/dir1 ../images/dir2
```

## Tips
- If the RMS error is large for stereo camera calibration, even though each monocular camera calibration is successful, you should check the checkerboard detection results. The order of the checkerboard squares may differ between the two cameras. If such a difference exists, delete the set of the images and calibrate again.

## Sample calibrated parameters
See 
- [monocular camera parameter](./monocular_camera_parameters.yaml) 
- stereo camera
    - [right camera parameter](./right_camera_parameters.yaml)
    - [left camera parameter](./left_camera_parameters.yaml)