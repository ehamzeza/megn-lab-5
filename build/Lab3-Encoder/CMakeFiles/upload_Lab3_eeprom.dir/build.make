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
CMAKE_SOURCE_DIR = /u/st/by/ajgall/megn-lab-5

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /u/st/by/ajgall/megn-lab-5/build

# Utility rule file for upload_Lab3_eeprom.

# Include the progress variables for this target.
include Lab3-Encoder/CMakeFiles/upload_Lab3_eeprom.dir/progress.make

Lab3-Encoder/CMakeFiles/upload_Lab3_eeprom: Lab3-Encoder/Lab3-atmega32u4-eeprom.hex
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/u/st/by/ajgall/megn-lab-5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Uploading Lab3-atmega32u4-eeprom.hex to atmega32u4 using avr109"
	cd /u/st/by/ajgall/megn-lab-5/build/Lab3-Encoder && avrdude -p atmega32u4 -c avr109 -U eeprom:w:Lab3-atmega32u4-eeprom.hex -P /dev/ttyZumoCarAVR

Lab3-Encoder/Lab3-atmega32u4-eeprom.hex: Lab3-Encoder/Lab3-atmega32u4.elf
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/u/st/by/ajgall/megn-lab-5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating Lab3-atmega32u4-eeprom.hex"
	cd /u/st/by/ajgall/megn-lab-5/build/Lab3-Encoder && /usr/bin/avr-objcopy -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O ihex Lab3-atmega32u4.elf Lab3-atmega32u4-eeprom.hex

upload_Lab3_eeprom: Lab3-Encoder/CMakeFiles/upload_Lab3_eeprom
upload_Lab3_eeprom: Lab3-Encoder/Lab3-atmega32u4-eeprom.hex
upload_Lab3_eeprom: Lab3-Encoder/CMakeFiles/upload_Lab3_eeprom.dir/build.make

.PHONY : upload_Lab3_eeprom

# Rule to build all files generated by this target.
Lab3-Encoder/CMakeFiles/upload_Lab3_eeprom.dir/build: upload_Lab3_eeprom

.PHONY : Lab3-Encoder/CMakeFiles/upload_Lab3_eeprom.dir/build

Lab3-Encoder/CMakeFiles/upload_Lab3_eeprom.dir/clean:
	cd /u/st/by/ajgall/megn-lab-5/build/Lab3-Encoder && $(CMAKE_COMMAND) -P CMakeFiles/upload_Lab3_eeprom.dir/cmake_clean.cmake
.PHONY : Lab3-Encoder/CMakeFiles/upload_Lab3_eeprom.dir/clean

Lab3-Encoder/CMakeFiles/upload_Lab3_eeprom.dir/depend:
	cd /u/st/by/ajgall/megn-lab-5/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /u/st/by/ajgall/megn-lab-5 /u/st/by/ajgall/megn-lab-5/Lab3-Encoder /u/st/by/ajgall/megn-lab-5/build /u/st/by/ajgall/megn-lab-5/build/Lab3-Encoder /u/st/by/ajgall/megn-lab-5/build/Lab3-Encoder/CMakeFiles/upload_Lab3_eeprom.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Lab3-Encoder/CMakeFiles/upload_Lab3_eeprom.dir/depend

