# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_SOURCE_DIR = /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas

# Include any dependencies generated for this target.
include CMakeFiles/LSD_MULTISCALE.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/LSD_MULTISCALE.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/LSD_MULTISCALE.dir/flags.make

CMakeFiles/LSD_MULTISCALE.dir/DETECTION/lsd.o: CMakeFiles/LSD_MULTISCALE.dir/flags.make
CMakeFiles/LSD_MULTISCALE.dir/DETECTION/lsd.o: DETECTION/lsd.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/LSD_MULTISCALE.dir/DETECTION/lsd.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LSD_MULTISCALE.dir/DETECTION/lsd.o -c /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/DETECTION/lsd.cpp

CMakeFiles/LSD_MULTISCALE.dir/DETECTION/lsd.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LSD_MULTISCALE.dir/DETECTION/lsd.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/DETECTION/lsd.cpp > CMakeFiles/LSD_MULTISCALE.dir/DETECTION/lsd.i

CMakeFiles/LSD_MULTISCALE.dir/DETECTION/lsd.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LSD_MULTISCALE.dir/DETECTION/lsd.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/DETECTION/lsd.cpp -o CMakeFiles/LSD_MULTISCALE.dir/DETECTION/lsd.s

CMakeFiles/LSD_MULTISCALE.dir/DETECTION/mlsd.o: CMakeFiles/LSD_MULTISCALE.dir/flags.make
CMakeFiles/LSD_MULTISCALE.dir/DETECTION/mlsd.o: DETECTION/mlsd.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/LSD_MULTISCALE.dir/DETECTION/mlsd.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LSD_MULTISCALE.dir/DETECTION/mlsd.o -c /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/DETECTION/mlsd.cpp

CMakeFiles/LSD_MULTISCALE.dir/DETECTION/mlsd.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LSD_MULTISCALE.dir/DETECTION/mlsd.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/DETECTION/mlsd.cpp > CMakeFiles/LSD_MULTISCALE.dir/DETECTION/mlsd.i

CMakeFiles/LSD_MULTISCALE.dir/DETECTION/mlsd.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LSD_MULTISCALE.dir/DETECTION/mlsd.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/DETECTION/mlsd.cpp -o CMakeFiles/LSD_MULTISCALE.dir/DETECTION/mlsd.s

CMakeFiles/LSD_MULTISCALE.dir/DETECTION/detection.o: CMakeFiles/LSD_MULTISCALE.dir/flags.make
CMakeFiles/LSD_MULTISCALE.dir/DETECTION/detection.o: DETECTION/detection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/LSD_MULTISCALE.dir/DETECTION/detection.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LSD_MULTISCALE.dir/DETECTION/detection.o -c /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/DETECTION/detection.cpp

CMakeFiles/LSD_MULTISCALE.dir/DETECTION/detection.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LSD_MULTISCALE.dir/DETECTION/detection.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/DETECTION/detection.cpp > CMakeFiles/LSD_MULTISCALE.dir/DETECTION/detection.i

CMakeFiles/LSD_MULTISCALE.dir/DETECTION/detection.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LSD_MULTISCALE.dir/DETECTION/detection.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/DETECTION/detection.cpp -o CMakeFiles/LSD_MULTISCALE.dir/DETECTION/detection.s

CMakeFiles/LSD_MULTISCALE.dir/interface.o: CMakeFiles/LSD_MULTISCALE.dir/flags.make
CMakeFiles/LSD_MULTISCALE.dir/interface.o: interface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/LSD_MULTISCALE.dir/interface.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LSD_MULTISCALE.dir/interface.o -c /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/interface.cpp

CMakeFiles/LSD_MULTISCALE.dir/interface.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LSD_MULTISCALE.dir/interface.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/interface.cpp > CMakeFiles/LSD_MULTISCALE.dir/interface.i

CMakeFiles/LSD_MULTISCALE.dir/interface.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LSD_MULTISCALE.dir/interface.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/interface.cpp -o CMakeFiles/LSD_MULTISCALE.dir/interface.s

CMakeFiles/LSD_MULTISCALE.dir/main_detection.o: CMakeFiles/LSD_MULTISCALE.dir/flags.make
CMakeFiles/LSD_MULTISCALE.dir/main_detection.o: main_detection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/LSD_MULTISCALE.dir/main_detection.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LSD_MULTISCALE.dir/main_detection.o -c /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/main_detection.cpp

CMakeFiles/LSD_MULTISCALE.dir/main_detection.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LSD_MULTISCALE.dir/main_detection.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/main_detection.cpp > CMakeFiles/LSD_MULTISCALE.dir/main_detection.i

CMakeFiles/LSD_MULTISCALE.dir/main_detection.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LSD_MULTISCALE.dir/main_detection.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/main_detection.cpp -o CMakeFiles/LSD_MULTISCALE.dir/main_detection.s

# Object files for target LSD_MULTISCALE
LSD_MULTISCALE_OBJECTS = \
"CMakeFiles/LSD_MULTISCALE.dir/DETECTION/lsd.o" \
"CMakeFiles/LSD_MULTISCALE.dir/DETECTION/mlsd.o" \
"CMakeFiles/LSD_MULTISCALE.dir/DETECTION/detection.o" \
"CMakeFiles/LSD_MULTISCALE.dir/interface.o" \
"CMakeFiles/LSD_MULTISCALE.dir/main_detection.o"

# External object files for target LSD_MULTISCALE
LSD_MULTISCALE_EXTERNAL_OBJECTS =

LSD_MULTISCALE: CMakeFiles/LSD_MULTISCALE.dir/DETECTION/lsd.o
LSD_MULTISCALE: CMakeFiles/LSD_MULTISCALE.dir/DETECTION/mlsd.o
LSD_MULTISCALE: CMakeFiles/LSD_MULTISCALE.dir/DETECTION/detection.o
LSD_MULTISCALE: CMakeFiles/LSD_MULTISCALE.dir/interface.o
LSD_MULTISCALE: CMakeFiles/LSD_MULTISCALE.dir/main_detection.o
LSD_MULTISCALE: CMakeFiles/LSD_MULTISCALE.dir/build.make
LSD_MULTISCALE: /usr/local/lib64/libopencv_gapi.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_stitching.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_aruco.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_bgsegm.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_bioinspired.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_ccalib.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_cvv.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_dnn_objdetect.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_dpm.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_face.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_freetype.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_fuzzy.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_hdf.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_hfs.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_img_hash.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_line_descriptor.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_reg.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_rgbd.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_saliency.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_stereo.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_structured_light.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_superres.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_surface_matching.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_tracking.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_videostab.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_viz.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_xfeatures2d.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_xobjdetect.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_xphoto.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_shape.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_phase_unwrapping.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_optflow.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_ximgproc.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_datasets.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_plot.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_text.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_dnn.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_ml.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_video.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_objdetect.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_calib3d.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_features2d.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_flann.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_highgui.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_videoio.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_imgcodecs.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_photo.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_imgproc.so.4.0.1
LSD_MULTISCALE: /usr/local/lib64/libopencv_core.so.4.0.1
LSD_MULTISCALE: CMakeFiles/LSD_MULTISCALE.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable LSD_MULTISCALE"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/LSD_MULTISCALE.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/LSD_MULTISCALE.dir/build: LSD_MULTISCALE

.PHONY : CMakeFiles/LSD_MULTISCALE.dir/build

CMakeFiles/LSD_MULTISCALE.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/LSD_MULTISCALE.dir/cmake_clean.cmake
.PHONY : CMakeFiles/LSD_MULTISCALE.dir/clean

CMakeFiles/LSD_MULTISCALE.dir/depend:
	cd /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas /home/natalia/Documentos/Mestrado/trabalho_extracao/TP_Extração_2019/codigos/5_extrai_linhas/CMakeFiles/LSD_MULTISCALE.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/LSD_MULTISCALE.dir/depend
