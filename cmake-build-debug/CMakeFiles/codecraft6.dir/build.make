# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.13

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\JetBrains\CLion 2018.3.4\bin\cmake\win\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\JetBrains\CLion 2018.3.4\bin\cmake\win\bin\cmake.exe" -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = E:\projects\codecraft6

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = E:\projects\codecraft6\cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/codecraft6.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/codecraft6.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/codecraft6.dir/flags.make

CMakeFiles/codecraft6.dir/main.cpp.obj: CMakeFiles/codecraft6.dir/flags.make
CMakeFiles/codecraft6.dir/main.cpp.obj: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=E:\projects\codecraft6\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/codecraft6.dir/main.cpp.obj"
	C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\G__~1.EXE  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\codecraft6.dir\main.cpp.obj -c E:\projects\codecraft6\main.cpp

CMakeFiles/codecraft6.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/codecraft6.dir/main.cpp.i"
	C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E E:\projects\codecraft6\main.cpp > CMakeFiles\codecraft6.dir\main.cpp.i

CMakeFiles/codecraft6.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/codecraft6.dir/main.cpp.s"
	C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\G__~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S E:\projects\codecraft6\main.cpp -o CMakeFiles\codecraft6.dir\main.cpp.s

# Object files for target codecraft6
codecraft6_OBJECTS = \
"CMakeFiles/codecraft6.dir/main.cpp.obj"

# External object files for target codecraft6
codecraft6_EXTERNAL_OBJECTS =

codecraft6.exe: CMakeFiles/codecraft6.dir/main.cpp.obj
codecraft6.exe: CMakeFiles/codecraft6.dir/build.make
codecraft6.exe: CMakeFiles/codecraft6.dir/linklibs.rsp
codecraft6.exe: CMakeFiles/codecraft6.dir/objects1.rsp
codecraft6.exe: CMakeFiles/codecraft6.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=E:\projects\codecraft6\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable codecraft6.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\codecraft6.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/codecraft6.dir/build: codecraft6.exe

.PHONY : CMakeFiles/codecraft6.dir/build

CMakeFiles/codecraft6.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\codecraft6.dir\cmake_clean.cmake
.PHONY : CMakeFiles/codecraft6.dir/clean

CMakeFiles/codecraft6.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" E:\projects\codecraft6 E:\projects\codecraft6 E:\projects\codecraft6\cmake-build-debug E:\projects\codecraft6\cmake-build-debug E:\projects\codecraft6\cmake-build-debug\CMakeFiles\codecraft6.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/codecraft6.dir/depend

