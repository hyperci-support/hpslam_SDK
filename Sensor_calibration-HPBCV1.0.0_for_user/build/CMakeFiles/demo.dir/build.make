# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/leo/project/usb_camera_imu/yuanxingcamera_demo/Sensor_calibration-HPBCV1.0.0_for_user

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leo/project/usb_camera_imu/yuanxingcamera_demo/Sensor_calibration-HPBCV1.0.0_for_user/build

# Include any dependencies generated for this target.
include CMakeFiles/demo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/demo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/demo.dir/flags.make

CMakeFiles/demo.dir/main.cpp.o: CMakeFiles/demo.dir/flags.make
CMakeFiles/demo.dir/main.cpp.o: ../main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/leo/project/usb_camera_imu/yuanxingcamera_demo/Sensor_calibration-HPBCV1.0.0_for_user/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/demo.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/demo.dir/main.cpp.o -c /home/leo/project/usb_camera_imu/yuanxingcamera_demo/Sensor_calibration-HPBCV1.0.0_for_user/main.cpp

CMakeFiles/demo.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demo.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/leo/project/usb_camera_imu/yuanxingcamera_demo/Sensor_calibration-HPBCV1.0.0_for_user/main.cpp > CMakeFiles/demo.dir/main.cpp.i

CMakeFiles/demo.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demo.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/leo/project/usb_camera_imu/yuanxingcamera_demo/Sensor_calibration-HPBCV1.0.0_for_user/main.cpp -o CMakeFiles/demo.dir/main.cpp.s

CMakeFiles/demo.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/demo.dir/main.cpp.o.requires

CMakeFiles/demo.dir/main.cpp.o.provides: CMakeFiles/demo.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/demo.dir/build.make CMakeFiles/demo.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/demo.dir/main.cpp.o.provides

CMakeFiles/demo.dir/main.cpp.o.provides.build: CMakeFiles/demo.dir/main.cpp.o

# Object files for target demo
demo_OBJECTS = \
"CMakeFiles/demo.dir/main.cpp.o"

# External object files for target demo
demo_EXTERNAL_OBJECTS =

demo: CMakeFiles/demo.dir/main.cpp.o
demo: CMakeFiles/demo.dir/build.make
demo: /usr/local/lib/libopencv_videostab.so.2.4.9
demo: /usr/local/lib/libopencv_video.so.2.4.9
demo: /usr/local/lib/libopencv_ts.a
demo: /usr/local/lib/libopencv_superres.so.2.4.9
demo: /usr/local/lib/libopencv_stitching.so.2.4.9
demo: /usr/local/lib/libopencv_photo.so.2.4.9
demo: /usr/local/lib/libopencv_ocl.so.2.4.9
demo: /usr/local/lib/libopencv_objdetect.so.2.4.9
demo: /usr/local/lib/libopencv_nonfree.so.2.4.9
demo: /usr/local/lib/libopencv_ml.so.2.4.9
demo: /usr/local/lib/libopencv_legacy.so.2.4.9
demo: /usr/local/lib/libopencv_imgproc.so.2.4.9
demo: /usr/local/lib/libopencv_highgui.so.2.4.9
demo: /usr/local/lib/libopencv_gpu.so.2.4.9
demo: /usr/local/lib/libopencv_flann.so.2.4.9
demo: /usr/local/lib/libopencv_features2d.so.2.4.9
demo: /usr/local/lib/libopencv_core.so.2.4.9
demo: /usr/local/lib/libopencv_contrib.so.2.4.9
demo: /usr/local/lib/libopencv_calib3d.so.2.4.9
demo: ../lib/libSensor.so
demo: /usr/local/lib/libopencv_nonfree.so.2.4.9
demo: /usr/local/lib/libopencv_ocl.so.2.4.9
demo: /usr/local/lib/libopencv_gpu.so.2.4.9
demo: /usr/local/lib/libopencv_photo.so.2.4.9
demo: /usr/local/lib/libopencv_objdetect.so.2.4.9
demo: /usr/local/lib/libopencv_legacy.so.2.4.9
demo: /usr/local/lib/libopencv_video.so.2.4.9
demo: /usr/local/lib/libopencv_ml.so.2.4.9
demo: /usr/local/lib/libopencv_calib3d.so.2.4.9
demo: /usr/local/lib/libopencv_features2d.so.2.4.9
demo: /usr/local/lib/libopencv_highgui.so.2.4.9
demo: /usr/local/lib/libopencv_imgproc.so.2.4.9
demo: /usr/local/lib/libopencv_flann.so.2.4.9
demo: /usr/local/lib/libopencv_core.so.2.4.9
demo: CMakeFiles/demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable demo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/demo.dir/build: demo
.PHONY : CMakeFiles/demo.dir/build

CMakeFiles/demo.dir/requires: CMakeFiles/demo.dir/main.cpp.o.requires
.PHONY : CMakeFiles/demo.dir/requires

CMakeFiles/demo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/demo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/demo.dir/clean

CMakeFiles/demo.dir/depend:
	cd /home/leo/project/usb_camera_imu/yuanxingcamera_demo/Sensor_calibration-HPBCV1.0.0_for_user/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leo/project/usb_camera_imu/yuanxingcamera_demo/Sensor_calibration-HPBCV1.0.0_for_user /home/leo/project/usb_camera_imu/yuanxingcamera_demo/Sensor_calibration-HPBCV1.0.0_for_user /home/leo/project/usb_camera_imu/yuanxingcamera_demo/Sensor_calibration-HPBCV1.0.0_for_user/build /home/leo/project/usb_camera_imu/yuanxingcamera_demo/Sensor_calibration-HPBCV1.0.0_for_user/build /home/leo/project/usb_camera_imu/yuanxingcamera_demo/Sensor_calibration-HPBCV1.0.0_for_user/build/CMakeFiles/demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/demo.dir/depend

