# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.9

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
CMAKE_SOURCE_DIR = /root/code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/code/build

# Include any dependencies generated for this target.
include MyLib/CMakeFiles/Mylib.dir/depend.make

# Include the progress variables for this target.
include MyLib/CMakeFiles/Mylib.dir/progress.make

# Include the compile flags for this target's objects.
include MyLib/CMakeFiles/Mylib.dir/flags.make

MyLib/CMakeFiles/Mylib.dir/count.cpp.o: MyLib/CMakeFiles/Mylib.dir/flags.make
MyLib/CMakeFiles/Mylib.dir/count.cpp.o: ../MyLib/count.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object MyLib/CMakeFiles/Mylib.dir/count.cpp.o"
	cd /root/code/build/MyLib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Mylib.dir/count.cpp.o -c /root/code/MyLib/count.cpp

MyLib/CMakeFiles/Mylib.dir/count.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Mylib.dir/count.cpp.i"
	cd /root/code/build/MyLib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/code/MyLib/count.cpp > CMakeFiles/Mylib.dir/count.cpp.i

MyLib/CMakeFiles/Mylib.dir/count.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Mylib.dir/count.cpp.s"
	cd /root/code/build/MyLib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/code/MyLib/count.cpp -o CMakeFiles/Mylib.dir/count.cpp.s

MyLib/CMakeFiles/Mylib.dir/count.cpp.o.requires:

.PHONY : MyLib/CMakeFiles/Mylib.dir/count.cpp.o.requires

MyLib/CMakeFiles/Mylib.dir/count.cpp.o.provides: MyLib/CMakeFiles/Mylib.dir/count.cpp.o.requires
	$(MAKE) -f MyLib/CMakeFiles/Mylib.dir/build.make MyLib/CMakeFiles/Mylib.dir/count.cpp.o.provides.build
.PHONY : MyLib/CMakeFiles/Mylib.dir/count.cpp.o.provides

MyLib/CMakeFiles/Mylib.dir/count.cpp.o.provides.build: MyLib/CMakeFiles/Mylib.dir/count.cpp.o


MyLib/CMakeFiles/Mylib.dir/limit.cpp.o: MyLib/CMakeFiles/Mylib.dir/flags.make
MyLib/CMakeFiles/Mylib.dir/limit.cpp.o: ../MyLib/limit.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object MyLib/CMakeFiles/Mylib.dir/limit.cpp.o"
	cd /root/code/build/MyLib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Mylib.dir/limit.cpp.o -c /root/code/MyLib/limit.cpp

MyLib/CMakeFiles/Mylib.dir/limit.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Mylib.dir/limit.cpp.i"
	cd /root/code/build/MyLib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/code/MyLib/limit.cpp > CMakeFiles/Mylib.dir/limit.cpp.i

MyLib/CMakeFiles/Mylib.dir/limit.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Mylib.dir/limit.cpp.s"
	cd /root/code/build/MyLib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/code/MyLib/limit.cpp -o CMakeFiles/Mylib.dir/limit.cpp.s

MyLib/CMakeFiles/Mylib.dir/limit.cpp.o.requires:

.PHONY : MyLib/CMakeFiles/Mylib.dir/limit.cpp.o.requires

MyLib/CMakeFiles/Mylib.dir/limit.cpp.o.provides: MyLib/CMakeFiles/Mylib.dir/limit.cpp.o.requires
	$(MAKE) -f MyLib/CMakeFiles/Mylib.dir/build.make MyLib/CMakeFiles/Mylib.dir/limit.cpp.o.provides.build
.PHONY : MyLib/CMakeFiles/Mylib.dir/limit.cpp.o.provides

MyLib/CMakeFiles/Mylib.dir/limit.cpp.o.provides.build: MyLib/CMakeFiles/Mylib.dir/limit.cpp.o


# Object files for target Mylib
Mylib_OBJECTS = \
"CMakeFiles/Mylib.dir/count.cpp.o" \
"CMakeFiles/Mylib.dir/limit.cpp.o"

# External object files for target Mylib
Mylib_EXTERNAL_OBJECTS =

lib/libMylib.so: MyLib/CMakeFiles/Mylib.dir/count.cpp.o
lib/libMylib.so: MyLib/CMakeFiles/Mylib.dir/limit.cpp.o
lib/libMylib.so: MyLib/CMakeFiles/Mylib.dir/build.make
lib/libMylib.so: MyLib/CMakeFiles/Mylib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library ../lib/libMylib.so"
	cd /root/code/build/MyLib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Mylib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
MyLib/CMakeFiles/Mylib.dir/build: lib/libMylib.so

.PHONY : MyLib/CMakeFiles/Mylib.dir/build

MyLib/CMakeFiles/Mylib.dir/requires: MyLib/CMakeFiles/Mylib.dir/count.cpp.o.requires
MyLib/CMakeFiles/Mylib.dir/requires: MyLib/CMakeFiles/Mylib.dir/limit.cpp.o.requires

.PHONY : MyLib/CMakeFiles/Mylib.dir/requires

MyLib/CMakeFiles/Mylib.dir/clean:
	cd /root/code/build/MyLib && $(CMAKE_COMMAND) -P CMakeFiles/Mylib.dir/cmake_clean.cmake
.PHONY : MyLib/CMakeFiles/Mylib.dir/clean

MyLib/CMakeFiles/Mylib.dir/depend:
	cd /root/code/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/code /root/code/MyLib /root/code/build /root/code/build/MyLib /root/code/build/MyLib/CMakeFiles/Mylib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : MyLib/CMakeFiles/Mylib.dir/depend

