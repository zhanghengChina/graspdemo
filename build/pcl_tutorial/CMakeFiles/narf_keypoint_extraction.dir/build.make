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
include pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/depend.make

# Include the progress variables for this target.
include pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/progress.make

# Include the compile flags for this target's objects.
include pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/flags.make

pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.o: pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/flags.make
pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.o: /home/leon/graspdemo/src/pcl_tutorial/src/8narf_keypoint_extraction.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/leon/graspdemo/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.o"
	cd /home/leon/graspdemo/build/pcl_tutorial && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.o -c /home/leon/graspdemo/src/pcl_tutorial/src/8narf_keypoint_extraction.cpp

pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.i"
	cd /home/leon/graspdemo/build/pcl_tutorial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/leon/graspdemo/src/pcl_tutorial/src/8narf_keypoint_extraction.cpp > CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.i

pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.s"
	cd /home/leon/graspdemo/build/pcl_tutorial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/leon/graspdemo/src/pcl_tutorial/src/8narf_keypoint_extraction.cpp -o CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.s

pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.o.requires:
.PHONY : pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.o.requires

pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.o.provides: pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.o.requires
	$(MAKE) -f pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/build.make pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.o.provides.build
.PHONY : pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.o.provides

pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.o.provides.build: pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.o

# Object files for target narf_keypoint_extraction
narf_keypoint_extraction_OBJECTS = \
"CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.o"

# External object files for target narf_keypoint_extraction
narf_keypoint_extraction_EXTERNAL_OBJECTS =

/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.o
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/build.make
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libpcl_ros_filters.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libpcl_ros_io.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libpcl_ros_tf.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_common.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_octree.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_io.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_kdtree.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_search.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_sample_consensus.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_filters.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_features.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_keypoints.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_segmentation.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_visualization.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_outofcore.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_registration.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_recognition.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_surface.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_people.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_tracking.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libpcl_apps.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/libqhull.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libOpenNI.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libvtkCommon.so.5.8.0
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libvtkRendering.so.5.8.0
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libvtkHybrid.so.5.8.0
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libvtkCharts.so.5.8.0
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libdynamic_reconfigure_config_init_mutex.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libnodeletlib.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libbondcpp.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libclass_loader.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/libPocoFoundation.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/libdl.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libroslib.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/librospack.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/librosbag.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/librosbag_storage.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libroslz4.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/liblz4.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libtopic_tools.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libtf.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libtf2_ros.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libactionlib.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libmessage_filters.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libtf2.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libroscpp.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/librosconsole.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/liblog4cxx.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/librostime.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /opt/ros/indigo/lib/libcpp_common.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction: pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction"
	cd /home/leon/graspdemo/build/pcl_tutorial && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/narf_keypoint_extraction.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/build: /home/leon/graspdemo/devel/lib/pcl_tutorial/narf_keypoint_extraction
.PHONY : pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/build

pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/requires: pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/src/8narf_keypoint_extraction.cpp.o.requires
.PHONY : pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/requires

pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/clean:
	cd /home/leon/graspdemo/build/pcl_tutorial && $(CMAKE_COMMAND) -P CMakeFiles/narf_keypoint_extraction.dir/cmake_clean.cmake
.PHONY : pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/clean

pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/depend:
	cd /home/leon/graspdemo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leon/graspdemo/src /home/leon/graspdemo/src/pcl_tutorial /home/leon/graspdemo/build /home/leon/graspdemo/build/pcl_tutorial /home/leon/graspdemo/build/pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : pcl_tutorial/CMakeFiles/narf_keypoint_extraction.dir/depend

