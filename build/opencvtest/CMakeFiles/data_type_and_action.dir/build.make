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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/leon/graspdemo/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leon/graspdemo/build

# Include any dependencies generated for this target.
include opencvtest/CMakeFiles/data_type_and_action.dir/depend.make

# Include the progress variables for this target.
include opencvtest/CMakeFiles/data_type_and_action.dir/progress.make

# Include the compile flags for this target's objects.
include opencvtest/CMakeFiles/data_type_and_action.dir/flags.make

opencvtest/CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.o: opencvtest/CMakeFiles/data_type_and_action.dir/flags.make
opencvtest/CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.o: /home/leon/graspdemo/src/opencvtest/src/3data_type_and_action.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/leon/graspdemo/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object opencvtest/CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.o"
	cd /home/leon/graspdemo/build/opencvtest && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.o -c /home/leon/graspdemo/src/opencvtest/src/3data_type_and_action.cpp

opencvtest/CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.i"
	cd /home/leon/graspdemo/build/opencvtest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/leon/graspdemo/src/opencvtest/src/3data_type_and_action.cpp > CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.i

opencvtest/CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.s"
	cd /home/leon/graspdemo/build/opencvtest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/leon/graspdemo/src/opencvtest/src/3data_type_and_action.cpp -o CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.s

opencvtest/CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.o.requires:
.PHONY : opencvtest/CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.o.requires

opencvtest/CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.o.provides: opencvtest/CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.o.requires
	$(MAKE) -f opencvtest/CMakeFiles/data_type_and_action.dir/build.make opencvtest/CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.o.provides.build
.PHONY : opencvtest/CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.o.provides

opencvtest/CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.o.provides.build: opencvtest/CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.o

# Object files for target data_type_and_action
data_type_and_action_OBJECTS = \
"CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.o"

# External object files for target data_type_and_action
data_type_and_action_EXTERNAL_OBJECTS =

/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: opencvtest/CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.o
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: opencvtest/CMakeFiles/data_type_and_action.dir/build.make
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libcv_bridge.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_common_planning_interface_objects.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_planning_scene_interface.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_move_group_interface.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_warehouse.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libwarehouse_ros.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_pick_place_planner.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_move_group_capabilities_base.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_rdf_loader.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_kinematics_plugin_loader.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_robot_model_loader.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_constraint_sampler_manager_loader.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_planning_pipeline.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_trajectory_execution_manager.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_plan_execution.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_planning_scene_monitor.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_collision_plugin_loader.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_lazy_free_space_updater.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_point_containment_filter.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_occupancy_map_monitor.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_pointcloud_octomap_updater_core.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_semantic_world.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_exceptions.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_background_processing.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_kinematics_base.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_robot_model.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_transforms.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_robot_state.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_robot_trajectory.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_planning_interface.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_collision_detection.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_collision_detection_fcl.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_kinematic_constraints.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_planning_scene.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_constraint_samplers.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_planning_request_adapter.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_profiler.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_trajectory_processing.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_distance_field.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_kinematics_metrics.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmoveit_dynamics_solver.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libeigen_conversions.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libgeometric_shapes.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/liboctomap.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/liboctomath.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libkdl_parser.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/liborocos-kdl.so.1.3.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/liburdf.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/librosconsole_bridge.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/librandom_numbers.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libsrdfdom.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libimage_transport.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libclass_loader.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/libPocoFoundation.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libdl.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libroslib.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/librospack.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libtf2_ros.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libactionlib.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libmessage_filters.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libtf2.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libvisp_bridge.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/x86_64-linux-gnu/libvisp_vs.so.3.1.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/x86_64-linux-gnu/libvisp_visual_features.so.3.1.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/x86_64-linux-gnu/libvisp_vision.so.3.1.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/x86_64-linux-gnu/libvisp_tt_mi.so.3.1.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/x86_64-linux-gnu/libvisp_tt.so.3.1.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/x86_64-linux-gnu/libvisp_me.so.3.1.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/x86_64-linux-gnu/libvisp_mbt.so.3.1.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/x86_64-linux-gnu/libvisp_klt.so.3.1.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/x86_64-linux-gnu/libvisp_blob.so.3.1.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/x86_64-linux-gnu/libvisp_sensor.so.3.1.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/x86_64-linux-gnu/libvisp_robot.so.3.1.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/x86_64-linux-gnu/libvisp_io.so.3.1.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/x86_64-linux-gnu/libvisp_imgproc.so.3.1.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/x86_64-linux-gnu/libvisp_gui.so.3.1.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/x86_64-linux-gnu/libvisp_detection.so.3.1.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/x86_64-linux-gnu/libvisp_core.so.3.1.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/x86_64-linux-gnu/libvisp_ar.so.3.1.0
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libcamera_calibration_parsers.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libroscpp.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/librosconsole.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/liblog4cxx.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/librostime.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /opt/ros/indigo/lib/libcpp_common.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action: opencvtest/CMakeFiles/data_type_and_action.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action"
	cd /home/leon/graspdemo/build/opencvtest && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/data_type_and_action.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
opencvtest/CMakeFiles/data_type_and_action.dir/build: /home/leon/graspdemo/devel/lib/opencvtest/data_type_and_action
.PHONY : opencvtest/CMakeFiles/data_type_and_action.dir/build

opencvtest/CMakeFiles/data_type_and_action.dir/requires: opencvtest/CMakeFiles/data_type_and_action.dir/src/3data_type_and_action.cpp.o.requires
.PHONY : opencvtest/CMakeFiles/data_type_and_action.dir/requires

opencvtest/CMakeFiles/data_type_and_action.dir/clean:
	cd /home/leon/graspdemo/build/opencvtest && $(CMAKE_COMMAND) -P CMakeFiles/data_type_and_action.dir/cmake_clean.cmake
.PHONY : opencvtest/CMakeFiles/data_type_and_action.dir/clean

opencvtest/CMakeFiles/data_type_and_action.dir/depend:
	cd /home/leon/graspdemo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leon/graspdemo/src /home/leon/graspdemo/src/opencvtest /home/leon/graspdemo/build /home/leon/graspdemo/build/opencvtest /home/leon/graspdemo/build/opencvtest/CMakeFiles/data_type_and_action.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : opencvtest/CMakeFiles/data_type_and_action.dir/depend

