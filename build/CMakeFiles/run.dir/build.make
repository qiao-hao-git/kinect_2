# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/q/下载/My_Kinect

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/q/下载/My_Kinect/build

# Include any dependencies generated for this target.
include CMakeFiles/run.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/run.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/run.dir/flags.make

CMakeFiles/run.dir/main.cpp.o: CMakeFiles/run.dir/flags.make
CMakeFiles/run.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/q/下载/My_Kinect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/run.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run.dir/main.cpp.o -c /home/q/下载/My_Kinect/main.cpp

CMakeFiles/run.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/q/下载/My_Kinect/main.cpp > CMakeFiles/run.dir/main.cpp.i

CMakeFiles/run.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/q/下载/My_Kinect/main.cpp -o CMakeFiles/run.dir/main.cpp.s

CMakeFiles/run.dir/src/K4a_Grabber.cpp.o: CMakeFiles/run.dir/flags.make
CMakeFiles/run.dir/src/K4a_Grabber.cpp.o: ../src/K4a_Grabber.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/q/下载/My_Kinect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/run.dir/src/K4a_Grabber.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run.dir/src/K4a_Grabber.cpp.o -c /home/q/下载/My_Kinect/src/K4a_Grabber.cpp

CMakeFiles/run.dir/src/K4a_Grabber.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run.dir/src/K4a_Grabber.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/q/下载/My_Kinect/src/K4a_Grabber.cpp > CMakeFiles/run.dir/src/K4a_Grabber.cpp.i

CMakeFiles/run.dir/src/K4a_Grabber.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run.dir/src/K4a_Grabber.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/q/下载/My_Kinect/src/K4a_Grabber.cpp -o CMakeFiles/run.dir/src/K4a_Grabber.cpp.s

# Object files for target run
run_OBJECTS = \
"CMakeFiles/run.dir/main.cpp.o" \
"CMakeFiles/run.dir/src/K4a_Grabber.cpp.o"

# External object files for target run
run_EXTERNAL_OBJECTS =

run: CMakeFiles/run.dir/main.cpp.o
run: CMakeFiles/run.dir/src/K4a_Grabber.cpp.o
run: CMakeFiles/run.dir/build.make
run: /usr/local/lib/libopencv_gapi.so.4.5.1
run: /usr/local/lib/libopencv_stitching.so.4.5.1
run: /usr/local/lib/libopencv_alphamat.so.4.5.1
run: /usr/local/lib/libopencv_aruco.so.4.5.1
run: /usr/local/lib/libopencv_bgsegm.so.4.5.1
run: /usr/local/lib/libopencv_bioinspired.so.4.5.1
run: /usr/local/lib/libopencv_ccalib.so.4.5.1
run: /usr/local/lib/libopencv_dnn_objdetect.so.4.5.1
run: /usr/local/lib/libopencv_dnn_superres.so.4.5.1
run: /usr/local/lib/libopencv_dpm.so.4.5.1
run: /usr/local/lib/libopencv_face.so.4.5.1
run: /usr/local/lib/libopencv_freetype.so.4.5.1
run: /usr/local/lib/libopencv_fuzzy.so.4.5.1
run: /usr/local/lib/libopencv_hdf.so.4.5.1
run: /usr/local/lib/libopencv_hfs.so.4.5.1
run: /usr/local/lib/libopencv_img_hash.so.4.5.1
run: /usr/local/lib/libopencv_intensity_transform.so.4.5.1
run: /usr/local/lib/libopencv_line_descriptor.so.4.5.1
run: /usr/local/lib/libopencv_mcc.so.4.5.1
run: /usr/local/lib/libopencv_quality.so.4.5.1
run: /usr/local/lib/libopencv_rapid.so.4.5.1
run: /usr/local/lib/libopencv_reg.so.4.5.1
run: /usr/local/lib/libopencv_rgbd.so.4.5.1
run: /usr/local/lib/libopencv_saliency.so.4.5.1
run: /usr/local/lib/libopencv_stereo.so.4.5.1
run: /usr/local/lib/libopencv_structured_light.so.4.5.1
run: /usr/local/lib/libopencv_superres.so.4.5.1
run: /usr/local/lib/libopencv_surface_matching.so.4.5.1
run: /usr/local/lib/libopencv_tracking.so.4.5.1
run: /usr/local/lib/libopencv_videostab.so.4.5.1
run: /usr/local/lib/libopencv_viz.so.4.5.1
run: /usr/local/lib/libopencv_xfeatures2d.so.4.5.1
run: /usr/local/lib/libopencv_xobjdetect.so.4.5.1
run: /usr/local/lib/libopencv_xphoto.so.4.5.1
run: /usr/local/lib/libpcl_surface.so
run: /usr/local/lib/libpcl_keypoints.so
run: /usr/local/lib/libpcl_tracking.so
run: /usr/local/lib/libpcl_recognition.so
run: /usr/local/lib/libpcl_stereo.so
run: /usr/local/lib/libpcl_outofcore.so
run: /usr/local/lib/libpcl_people.so
run: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
run: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
run: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
run: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
run: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0
run: /usr/lib/x86_64-linux-gnu/libqhull_r.so
run: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libfreetype.so
run: /usr/lib/x86_64-linux-gnu/libz.so
run: /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libjpeg.so
run: /usr/lib/x86_64-linux-gnu/libpng.so
run: /usr/lib/x86_64-linux-gnu/libtiff.so
run: /usr/lib/x86_64-linux-gnu/libexpat.so
run: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
run: /usr/local/lib/libopencv_shape.so.4.5.1
run: /usr/local/lib/libopencv_highgui.so.4.5.1
run: /usr/local/lib/libopencv_datasets.so.4.5.1
run: /usr/local/lib/libopencv_plot.so.4.5.1
run: /usr/local/lib/libopencv_text.so.4.5.1
run: /usr/local/lib/libopencv_ml.so.4.5.1
run: /usr/local/lib/libopencv_phase_unwrapping.so.4.5.1
run: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libGLEW.so
run: /usr/lib/x86_64-linux-gnu/libSM.so
run: /usr/lib/x86_64-linux-gnu/libICE.so
run: /usr/lib/x86_64-linux-gnu/libX11.so
run: /usr/lib/x86_64-linux-gnu/libXext.so
run: /usr/lib/x86_64-linux-gnu/libXt.so
run: /usr/local/lib/libopencv_optflow.so.4.5.1
run: /usr/local/lib/libopencv_ximgproc.so.4.5.1
run: /usr/local/lib/libopencv_video.so.4.5.1
run: /usr/local/lib/libopencv_dnn.so.4.5.1
run: /usr/local/lib/libopencv_videoio.so.4.5.1
run: /usr/local/lib/libopencv_imgcodecs.so.4.5.1
run: /usr/local/lib/libopencv_objdetect.so.4.5.1
run: /usr/local/lib/libopencv_calib3d.so.4.5.1
run: /usr/local/lib/libopencv_features2d.so.4.5.1
run: /usr/local/lib/libopencv_flann.so.4.5.1
run: /usr/local/lib/libopencv_photo.so.4.5.1
run: /usr/local/lib/libopencv_imgproc.so.4.5.1
run: /usr/local/lib/libopencv_core.so.4.5.1
run: /usr/local/lib/libpcl_registration.so
run: /usr/local/lib/libpcl_visualization.so
run: /usr/local/lib/libpcl_io.so
run: /usr/local/lib/libpcl_segmentation.so
run: /usr/local/lib/libpcl_features.so
run: /usr/local/lib/libpcl_filters.so
run: /usr/local/lib/libpcl_sample_consensus.so
run: /usr/local/lib/libpcl_search.so
run: /usr/local/lib/libpcl_octree.so
run: /usr/local/lib/libpcl_kdtree.so
run: /usr/local/lib/libpcl_ml.so
run: /usr/local/lib/libpcl_common.so
run: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libz.so
run: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libfreetype.so
run: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
run: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
run: CMakeFiles/run.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/q/下载/My_Kinect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable run"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/run.dir/build: run

.PHONY : CMakeFiles/run.dir/build

CMakeFiles/run.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/run.dir/cmake_clean.cmake
.PHONY : CMakeFiles/run.dir/clean

CMakeFiles/run.dir/depend:
	cd /home/q/下载/My_Kinect/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/q/下载/My_Kinect /home/q/下载/My_Kinect /home/q/下载/My_Kinect/build /home/q/下载/My_Kinect/build /home/q/下载/My_Kinect/build/CMakeFiles/run.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/run.dir/depend

