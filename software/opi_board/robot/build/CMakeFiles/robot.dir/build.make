# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yong/robot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yong/robot/build

# Include any dependencies generated for this target.
include CMakeFiles/robot.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/robot.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/robot.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/robot.dir/flags.make

CMakeFiles/robot.dir/main.cpp.o: CMakeFiles/robot.dir/flags.make
CMakeFiles/robot.dir/main.cpp.o: ../main.cpp
CMakeFiles/robot.dir/main.cpp.o: CMakeFiles/robot.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yong/robot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/robot.dir/main.cpp.o"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/robot.dir/main.cpp.o -MF CMakeFiles/robot.dir/main.cpp.o.d -o CMakeFiles/robot.dir/main.cpp.o -c /home/yong/robot/main.cpp

CMakeFiles/robot.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robot.dir/main.cpp.i"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yong/robot/main.cpp > CMakeFiles/robot.dir/main.cpp.i

CMakeFiles/robot.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robot.dir/main.cpp.s"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yong/robot/main.cpp -o CMakeFiles/robot.dir/main.cpp.s

CMakeFiles/robot.dir/radar.cpp.o: CMakeFiles/robot.dir/flags.make
CMakeFiles/robot.dir/radar.cpp.o: ../radar.cpp
CMakeFiles/robot.dir/radar.cpp.o: CMakeFiles/robot.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yong/robot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/robot.dir/radar.cpp.o"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/robot.dir/radar.cpp.o -MF CMakeFiles/robot.dir/radar.cpp.o.d -o CMakeFiles/robot.dir/radar.cpp.o -c /home/yong/robot/radar.cpp

CMakeFiles/robot.dir/radar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robot.dir/radar.cpp.i"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yong/robot/radar.cpp > CMakeFiles/robot.dir/radar.cpp.i

CMakeFiles/robot.dir/radar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robot.dir/radar.cpp.s"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yong/robot/radar.cpp -o CMakeFiles/robot.dir/radar.cpp.s

CMakeFiles/robot.dir/apriltagDetect.cpp.o: CMakeFiles/robot.dir/flags.make
CMakeFiles/robot.dir/apriltagDetect.cpp.o: ../apriltagDetect.cpp
CMakeFiles/robot.dir/apriltagDetect.cpp.o: CMakeFiles/robot.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yong/robot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/robot.dir/apriltagDetect.cpp.o"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/robot.dir/apriltagDetect.cpp.o -MF CMakeFiles/robot.dir/apriltagDetect.cpp.o.d -o CMakeFiles/robot.dir/apriltagDetect.cpp.o -c /home/yong/robot/apriltagDetect.cpp

CMakeFiles/robot.dir/apriltagDetect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robot.dir/apriltagDetect.cpp.i"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yong/robot/apriltagDetect.cpp > CMakeFiles/robot.dir/apriltagDetect.cpp.i

CMakeFiles/robot.dir/apriltagDetect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robot.dir/apriltagDetect.cpp.s"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yong/robot/apriltagDetect.cpp -o CMakeFiles/robot.dir/apriltagDetect.cpp.s

CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe0.c.o: CMakeFiles/robot.dir/flags.make
CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe0.c.o: ../tof/nlink_linktrack_nodeframe0.c
CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe0.c.o: CMakeFiles/robot.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yong/robot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe0.c.o"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe0.c.o -MF CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe0.c.o.d -o CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe0.c.o -c /home/yong/robot/tof/nlink_linktrack_nodeframe0.c

CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe0.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe0.c.i"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/yong/robot/tof/nlink_linktrack_nodeframe0.c > CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe0.c.i

CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe0.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe0.c.s"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/yong/robot/tof/nlink_linktrack_nodeframe0.c -o CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe0.c.s

CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe1.c.o: CMakeFiles/robot.dir/flags.make
CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe1.c.o: ../tof/nlink_linktrack_nodeframe1.c
CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe1.c.o: CMakeFiles/robot.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yong/robot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe1.c.o"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe1.c.o -MF CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe1.c.o.d -o CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe1.c.o -c /home/yong/robot/tof/nlink_linktrack_nodeframe1.c

CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe1.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe1.c.i"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/yong/robot/tof/nlink_linktrack_nodeframe1.c > CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe1.c.i

CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe1.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe1.c.s"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/yong/robot/tof/nlink_linktrack_nodeframe1.c -o CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe1.c.s

CMakeFiles/robot.dir/tof/nlink_tofsense_frame0.c.o: CMakeFiles/robot.dir/flags.make
CMakeFiles/robot.dir/tof/nlink_tofsense_frame0.c.o: ../tof/nlink_tofsense_frame0.c
CMakeFiles/robot.dir/tof/nlink_tofsense_frame0.c.o: CMakeFiles/robot.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yong/robot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object CMakeFiles/robot.dir/tof/nlink_tofsense_frame0.c.o"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/robot.dir/tof/nlink_tofsense_frame0.c.o -MF CMakeFiles/robot.dir/tof/nlink_tofsense_frame0.c.o.d -o CMakeFiles/robot.dir/tof/nlink_tofsense_frame0.c.o -c /home/yong/robot/tof/nlink_tofsense_frame0.c

CMakeFiles/robot.dir/tof/nlink_tofsense_frame0.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/robot.dir/tof/nlink_tofsense_frame0.c.i"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/yong/robot/tof/nlink_tofsense_frame0.c > CMakeFiles/robot.dir/tof/nlink_tofsense_frame0.c.i

CMakeFiles/robot.dir/tof/nlink_tofsense_frame0.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/robot.dir/tof/nlink_tofsense_frame0.c.s"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/yong/robot/tof/nlink_tofsense_frame0.c -o CMakeFiles/robot.dir/tof/nlink_tofsense_frame0.c.s

CMakeFiles/robot.dir/tof/nlink_tofsensem_frame0.c.o: CMakeFiles/robot.dir/flags.make
CMakeFiles/robot.dir/tof/nlink_tofsensem_frame0.c.o: ../tof/nlink_tofsensem_frame0.c
CMakeFiles/robot.dir/tof/nlink_tofsensem_frame0.c.o: CMakeFiles/robot.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yong/robot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object CMakeFiles/robot.dir/tof/nlink_tofsensem_frame0.c.o"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/robot.dir/tof/nlink_tofsensem_frame0.c.o -MF CMakeFiles/robot.dir/tof/nlink_tofsensem_frame0.c.o.d -o CMakeFiles/robot.dir/tof/nlink_tofsensem_frame0.c.o -c /home/yong/robot/tof/nlink_tofsensem_frame0.c

CMakeFiles/robot.dir/tof/nlink_tofsensem_frame0.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/robot.dir/tof/nlink_tofsensem_frame0.c.i"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/yong/robot/tof/nlink_tofsensem_frame0.c > CMakeFiles/robot.dir/tof/nlink_tofsensem_frame0.c.i

CMakeFiles/robot.dir/tof/nlink_tofsensem_frame0.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/robot.dir/tof/nlink_tofsensem_frame0.c.s"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/yong/robot/tof/nlink_tofsensem_frame0.c -o CMakeFiles/robot.dir/tof/nlink_tofsensem_frame0.c.s

CMakeFiles/robot.dir/tof/nlink_utils.c.o: CMakeFiles/robot.dir/flags.make
CMakeFiles/robot.dir/tof/nlink_utils.c.o: ../tof/nlink_utils.c
CMakeFiles/robot.dir/tof/nlink_utils.c.o: CMakeFiles/robot.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yong/robot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object CMakeFiles/robot.dir/tof/nlink_utils.c.o"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/robot.dir/tof/nlink_utils.c.o -MF CMakeFiles/robot.dir/tof/nlink_utils.c.o.d -o CMakeFiles/robot.dir/tof/nlink_utils.c.o -c /home/yong/robot/tof/nlink_utils.c

CMakeFiles/robot.dir/tof/nlink_utils.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/robot.dir/tof/nlink_utils.c.i"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/yong/robot/tof/nlink_utils.c > CMakeFiles/robot.dir/tof/nlink_utils.c.i

CMakeFiles/robot.dir/tof/nlink_utils.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/robot.dir/tof/nlink_utils.c.s"
	/usr/bin/aarch64-linux-gnu-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/yong/robot/tof/nlink_utils.c -o CMakeFiles/robot.dir/tof/nlink_utils.c.s

# Object files for target robot
robot_OBJECTS = \
"CMakeFiles/robot.dir/main.cpp.o" \
"CMakeFiles/robot.dir/radar.cpp.o" \
"CMakeFiles/robot.dir/apriltagDetect.cpp.o" \
"CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe0.c.o" \
"CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe1.c.o" \
"CMakeFiles/robot.dir/tof/nlink_tofsense_frame0.c.o" \
"CMakeFiles/robot.dir/tof/nlink_tofsensem_frame0.c.o" \
"CMakeFiles/robot.dir/tof/nlink_utils.c.o"

# External object files for target robot
robot_EXTERNAL_OBJECTS =

robot: CMakeFiles/robot.dir/main.cpp.o
robot: CMakeFiles/robot.dir/radar.cpp.o
robot: CMakeFiles/robot.dir/apriltagDetect.cpp.o
robot: CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe0.c.o
robot: CMakeFiles/robot.dir/tof/nlink_linktrack_nodeframe1.c.o
robot: CMakeFiles/robot.dir/tof/nlink_tofsense_frame0.c.o
robot: CMakeFiles/robot.dir/tof/nlink_tofsensem_frame0.c.o
robot: CMakeFiles/robot.dir/tof/nlink_utils.c.o
robot: CMakeFiles/robot.dir/build.make
robot: /usr/lib/aarch64-linux-gnu/libopencv_stitching.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_alphamat.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_aruco.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_barcode.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_bgsegm.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_bioinspired.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_ccalib.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_dnn_objdetect.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_dnn_superres.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_dpm.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_face.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_freetype.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_fuzzy.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_hdf.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_hfs.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_img_hash.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_intensity_transform.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_line_descriptor.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_mcc.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_quality.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_rapid.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_reg.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_rgbd.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_saliency.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_shape.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_stereo.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_structured_light.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_superres.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_surface_matching.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_tracking.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_videostab.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_viz.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_wechat_qrcode.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_xobjdetect.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_xphoto.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_datasets.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_plot.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_text.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_ml.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_phase_unwrapping.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_optflow.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_ximgproc.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_video.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_photo.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.5.4d
robot: /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.5.4d
robot: CMakeFiles/robot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yong/robot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX executable robot"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/robot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/robot.dir/build: robot
.PHONY : CMakeFiles/robot.dir/build

CMakeFiles/robot.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/robot.dir/cmake_clean.cmake
.PHONY : CMakeFiles/robot.dir/clean

CMakeFiles/robot.dir/depend:
	cd /home/yong/robot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yong/robot /home/yong/robot /home/yong/robot/build /home/yong/robot/build /home/yong/robot/build/CMakeFiles/robot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/robot.dir/depend

