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
CMAKE_SOURCE_DIR = /home/parallels/projector_tracker

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/parallels/projector_tracker/release

# Include any dependencies generated for this target.
include CMakeFiles/projector_tracker.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/projector_tracker.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/projector_tracker.dir/flags.make

CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.o: CMakeFiles/projector_tracker.dir/flags.make
CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.o: ../src/ProjectorTracker.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/parallels/projector_tracker/release/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.o -c /home/parallels/projector_tracker/src/ProjectorTracker.cpp

CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/parallels/projector_tracker/src/ProjectorTracker.cpp > CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.i

CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/parallels/projector_tracker/src/ProjectorTracker.cpp -o CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.s

CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.o.requires:
.PHONY : CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.o.requires

CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.o.provides: CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.o.requires
	$(MAKE) -f CMakeFiles/projector_tracker.dir/build.make CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.o.provides.build
.PHONY : CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.o.provides

CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.o.provides.build: CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.o

# Object files for target projector_tracker
projector_tracker_OBJECTS = \
"CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.o"

# External object files for target projector_tracker
projector_tracker_EXTERNAL_OBJECTS =

libprojector_tracker.a: CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.o
libprojector_tracker.a: CMakeFiles/projector_tracker.dir/build.make
libprojector_tracker.a: CMakeFiles/projector_tracker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libprojector_tracker.a"
	$(CMAKE_COMMAND) -P CMakeFiles/projector_tracker.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/projector_tracker.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/projector_tracker.dir/build: libprojector_tracker.a
.PHONY : CMakeFiles/projector_tracker.dir/build

CMakeFiles/projector_tracker.dir/requires: CMakeFiles/projector_tracker.dir/src/ProjectorTracker.cpp.o.requires
.PHONY : CMakeFiles/projector_tracker.dir/requires

CMakeFiles/projector_tracker.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/projector_tracker.dir/cmake_clean.cmake
.PHONY : CMakeFiles/projector_tracker.dir/clean

CMakeFiles/projector_tracker.dir/depend:
	cd /home/parallels/projector_tracker/release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/parallels/projector_tracker /home/parallels/projector_tracker /home/parallels/projector_tracker/release /home/parallels/projector_tracker/release /home/parallels/projector_tracker/release/CMakeFiles/projector_tracker.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/projector_tracker.dir/depend

