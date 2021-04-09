cmake_minimum_required(VERSION 3.7)
set(COBOTSYS_DEPENDED_MODULES Cow.CobotStudio.FrameWork Puppy.Common Sensor.RobotDirver.Common Cow.CobotStudio.Common)
set(COBOTSYS_DEPENDED_3RD_MODULES rttr ompl pcl boost vtk opencv modbus Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Qml orocos_kdl)

include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/qt.template.cmake)
include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/test.template.cmake)