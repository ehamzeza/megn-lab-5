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

# Utility rule file for upload_Lab1_eeprom.

# Include any custom commands dependencies for this target.
include Lab1-Serial/CMakeFiles/upload_Lab1_eeprom.dir/compiler_depend.make

# Include the progress variables for this target.
include Lab1-Serial/CMakeFiles/upload_Lab1_eeprom.dir/progress.make

Lab1-Serial/CMakeFiles/upload_Lab1_eeprom: Lab1-Serial/Lab1-atmega32u4-eeprom.hex
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ehamzeza/megn-lab-5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Uploading Lab1-atmega32u4-eeprom.hex to atmega32u4 using avr109"
	cd /home/ehamzeza/megn-lab-5/build/Lab1-Serial && avrdude -p atmega32u4 -c avr109 -U eeprom:w:Lab1-atmega32u4-eeprom.hex -P /dev/ttyZumoCarAVR

Lab1-Serial/Lab1-atmega32u4-eeprom.hex: Lab1-Serial/Lab1-atmega32u4.elf
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ehamzeza/megn-lab-5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating Lab1-atmega32u4-eeprom.hex"
	cd /home/ehamzeza/megn-lab-5/build/Lab1-Serial && /usr/bin/avr-objcopy -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O ihex Lab1-atmega32u4.elf Lab1-atmega32u4-eeprom.hex

upload_Lab1_eeprom: Lab1-Serial/CMakeFiles/upload_Lab1_eeprom
upload_Lab1_eeprom: Lab1-Serial/Lab1-atmega32u4-eeprom.hex
upload_Lab1_eeprom: Lab1-Serial/CMakeFiles/upload_Lab1_eeprom.dir/build.make
.PHONY : upload_Lab1_eeprom

# Rule to build all files generated by this target.
Lab1-Serial/CMakeFiles/upload_Lab1_eeprom.dir/build: upload_Lab1_eeprom
.PHONY : Lab1-Serial/CMakeFiles/upload_Lab1_eeprom.dir/build

Lab1-Serial/CMakeFiles/upload_Lab1_eeprom.dir/clean:
	cd /home/ehamzeza/megn-lab-5/build/Lab1-Serial && $(CMAKE_COMMAND) -P CMakeFiles/upload_Lab1_eeprom.dir/cmake_clean.cmake
.PHONY : Lab1-Serial/CMakeFiles/upload_Lab1_eeprom.dir/clean

Lab1-Serial/CMakeFiles/upload_Lab1_eeprom.dir/depend:
	cd /home/ehamzeza/megn-lab-5/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ehamzeza/megn-lab-5 /home/ehamzeza/megn-lab-5/Lab1-Serial /home/ehamzeza/megn-lab-5/build /home/ehamzeza/megn-lab-5/build/Lab1-Serial /home/ehamzeza/megn-lab-5/build/Lab1-Serial/CMakeFiles/upload_Lab1_eeprom.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Lab1-Serial/CMakeFiles/upload_Lab1_eeprom.dir/depend

