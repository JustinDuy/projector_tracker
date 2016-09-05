# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.4

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dynadev/projector_tracker

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dynadev/projector_tracker/release

# Include any dependencies generated for this target.
include CMakeFiles/projector_calibration.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/projector_calibration.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/projector_calibration.dir/flags.make

CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.o: CMakeFiles/projector_calibration.dir/flags.make
CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.o: ../src/ProjectorCalibration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dynadev/projector_tracker/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.o -c /home/dynadev/projector_tracker/src/ProjectorCalibration.cpp

CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dynadev/projector_tracker/src/ProjectorCalibration.cpp > CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.i

CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dynadev/projector_tracker/src/ProjectorCalibration.cpp -o CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.s

CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.o.requires:

.PHONY : CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.o.requires

CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.o.provides: CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.o.requires
	$(MAKE) -f CMakeFiles/projector_calibration.dir/build.make CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.o.provides.build
.PHONY : CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.o.provides

CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.o.provides.build: CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.o


# Object files for target projector_calibration
projector_calibration_OBJECTS = \
"CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.o"

# External object files for target projector_calibration
projector_calibration_EXTERNAL_OBJECTS =

libprojector_calibration.a: CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.o
libprojector_calibration.a: CMakeFiles/projector_calibration.dir/build.make
libprojector_calibration.a: CMakeFiles/projector_calibration.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dynadev/projector_tracker/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libprojector_calibration.a"
	$(CMAKE_COMMAND) -P CMakeFiles/projector_calibration.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/projector_calibration.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/projector_calibration.dir/build: libprojector_calibration.a

.PHONY : CMakeFiles/projector_calibration.dir/build

CMakeFiles/projector_calibration.dir/requires: CMakeFiles/projector_calibration.dir/src/ProjectorCalibration.cpp.o.requires

.PHONY : CMakeFiles/projector_calibration.dir/requires

CMakeFiles/projector_calibration.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/projector_calibration.dir/cmake_clean.cmake
.PHONY : CMakeFiles/projector_calibration.dir/clean

CMakeFiles/projector_calibration.dir/depend:
	cd /home/dynadev/projector_tracker/release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dynadev/projector_tracker /home/dynadev/projector_tracker /home/dynadev/projector_tracker/release /home/dynadev/projector_tracker/release /home/dynadev/projector_tracker/release/CMakeFiles/projector_calibration.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/projector_calibration.dir/depend

