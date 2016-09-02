# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.0

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
CMAKE_SOURCE_DIR = /home/justin/projector_tracker

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/justin/projector_tracker/release

# Include any dependencies generated for this target.
include CMakeFiles/hw_interface.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/hw_interface.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/hw_interface.dir/flags.make

CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.o: CMakeFiles/hw_interface.dir/flags.make
CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.o: ../src/CameraInterface.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/justin/projector_tracker/release/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.o -c /home/justin/projector_tracker/src/CameraInterface.cpp

CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/justin/projector_tracker/src/CameraInterface.cpp > CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.i

CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/justin/projector_tracker/src/CameraInterface.cpp -o CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.s

CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.o.requires:
.PHONY : CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.o.requires

CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.o.provides: CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw_interface.dir/build.make CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.o.provides.build
.PHONY : CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.o.provides

CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.o.provides.build: CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.o

CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.o: CMakeFiles/hw_interface.dir/flags.make
CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.o: ../src/ProjectorInterface.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/justin/projector_tracker/release/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.o -c /home/justin/projector_tracker/src/ProjectorInterface.cpp

CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/justin/projector_tracker/src/ProjectorInterface.cpp > CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.i

CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/justin/projector_tracker/src/ProjectorInterface.cpp -o CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.s

CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.o.requires:
.PHONY : CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.o.requires

CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.o.provides: CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw_interface.dir/build.make CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.o.provides.build
.PHONY : CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.o.provides

CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.o.provides.build: CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.o

CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.o: CMakeFiles/hw_interface.dir/flags.make
CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.o: ../src/CameraProjectorInterface.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/justin/projector_tracker/release/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.o -c /home/justin/projector_tracker/src/CameraProjectorInterface.cpp

CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/justin/projector_tracker/src/CameraProjectorInterface.cpp > CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.i

CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/justin/projector_tracker/src/CameraProjectorInterface.cpp -o CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.s

CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.o.requires:
.PHONY : CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.o.requires

CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.o.provides: CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw_interface.dir/build.make CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.o.provides.build
.PHONY : CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.o.provides

CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.o.provides.build: CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.o

CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.o: CMakeFiles/hw_interface.dir/flags.make
CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.o: ../src/ImageLabel.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/justin/projector_tracker/release/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.o -c /home/justin/projector_tracker/src/ImageLabel.cpp

CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/justin/projector_tracker/src/ImageLabel.cpp > CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.i

CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/justin/projector_tracker/src/ImageLabel.cpp -o CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.s

CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.o.requires:
.PHONY : CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.o.requires

CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.o.provides: CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.o.requires
	$(MAKE) -f CMakeFiles/hw_interface.dir/build.make CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.o.provides.build
.PHONY : CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.o.provides

CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.o.provides.build: CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.o

# Object files for target hw_interface
hw_interface_OBJECTS = \
"CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.o" \
"CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.o" \
"CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.o" \
"CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.o"

# External object files for target hw_interface
hw_interface_EXTERNAL_OBJECTS =

libhw_interface.a: CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.o
libhw_interface.a: CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.o
libhw_interface.a: CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.o
libhw_interface.a: CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.o
libhw_interface.a: CMakeFiles/hw_interface.dir/build.make
libhw_interface.a: CMakeFiles/hw_interface.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libhw_interface.a"
	$(CMAKE_COMMAND) -P CMakeFiles/hw_interface.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hw_interface.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/hw_interface.dir/build: libhw_interface.a
.PHONY : CMakeFiles/hw_interface.dir/build

CMakeFiles/hw_interface.dir/requires: CMakeFiles/hw_interface.dir/src/CameraInterface.cpp.o.requires
CMakeFiles/hw_interface.dir/requires: CMakeFiles/hw_interface.dir/src/ProjectorInterface.cpp.o.requires
CMakeFiles/hw_interface.dir/requires: CMakeFiles/hw_interface.dir/src/CameraProjectorInterface.cpp.o.requires
CMakeFiles/hw_interface.dir/requires: CMakeFiles/hw_interface.dir/src/ImageLabel.cpp.o.requires
.PHONY : CMakeFiles/hw_interface.dir/requires

CMakeFiles/hw_interface.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/hw_interface.dir/cmake_clean.cmake
.PHONY : CMakeFiles/hw_interface.dir/clean

CMakeFiles/hw_interface.dir/depend:
	cd /home/justin/projector_tracker/release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/justin/projector_tracker /home/justin/projector_tracker /home/justin/projector_tracker/release /home/justin/projector_tracker/release /home/justin/projector_tracker/release/CMakeFiles/hw_interface.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/hw_interface.dir/depend

