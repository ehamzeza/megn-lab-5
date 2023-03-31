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
CMAKE_SOURCE_DIR = /home/ehamzeza/megn-lab-5

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ehamzeza/megn-lab-5/build

# Utility rule file for erase_program.

# Include any custom commands dependencies for this target.
include CMakeFiles/erase_program.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/erase_program.dir/progress.make

CMakeFiles/erase_program:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ehamzeza/megn-lab-5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Erasing Program on atmega32u4 using avr109"
	avrdude -p atmega32u4 -c avr109 -P /dev/ttyZumoCarAVR -u -e

erase_program: CMakeFiles/erase_program
erase_program: CMakeFiles/erase_program.dir/build.make
.PHONY : erase_program

# Rule to build all files generated by this target.
CMakeFiles/erase_program.dir/build: erase_program
.PHONY : CMakeFiles/erase_program.dir/build

CMakeFiles/erase_program.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/erase_program.dir/cmake_clean.cmake
.PHONY : CMakeFiles/erase_program.dir/clean

CMakeFiles/erase_program.dir/depend:
	cd /home/ehamzeza/megn-lab-5/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ehamzeza/megn-lab-5 /home/ehamzeza/megn-lab-5 /home/ehamzeza/megn-lab-5/build /home/ehamzeza/megn-lab-5/build /home/ehamzeza/megn-lab-5/build/CMakeFiles/erase_program.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/erase_program.dir/depend

