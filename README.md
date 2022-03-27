
To use this project please make sure to checkout the branch mmm2.2 on Simox, MMMCore and MMMTools. However, this branch will be merged to master soon!


___________________________________________

(Optional) Add motion handler in MMMViewer

1. Run mmmtools/build/bin/MMMViewer
2. Goto Extra -> Plugins... -> Motion Handler -> Add Plugin Directory
3. Open folder .../build/lib/motionHandlerPluginLibs of this project






Install drake library and build with gcc-8:

1. sudo mkdir /opt/drake8 && sudo chown COMPUTER_USER_NAME:COMPUTER_USER_NAME /opt/drake8)

2. git clone https://github.com/RobotLocomotion/drake.git

3. sudo ./drake/setup/install_prereqs.sh

4. Open drake/common/filesystem.h AND drake/common/filesystem.cpp

5. Remove the following lines for both files:
#if __has_include(<filesystem>) && !defined(__APPLE__)
...
#else

AND

#endif

6. Build and install drake:
(cd drake && mkdir build && cd build && CXX=g++-8 CC=gcc-8 cmake -DCMAKE_INSTALL_PREFIX=/opt/drake8 .. && make)

7. cd {this_project}

8. Rebuild this project. If already a drake library is installed use cmake-gui .. in your build folder to change to /opt/drake8

