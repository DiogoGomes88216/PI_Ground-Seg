inherit ros_distro_melodic
DESCRIPTION = "ground_seg"
SECTION = "devel"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://package.xml;;beginline=16;endline=16;md5=2feaf30a620f46f06a4b016624acf46f"

ROS_CN = "ground-seg"
ROS_BPN = "ground-seg"

ROS_BUILDTOOL_DEPENDS = " \
    catkin-native \
"
ROS_BUILD_DEPENDS = " \
    roscpp \
    rospy \
    sensor-msgs \
    std-msgs \
    pcl-conversions \
    pcl-ros \
"
ROS_EXEC_DEPENDS = " \
    roscpp \
    rospy \
    sensor-msgs \
    std-msgs \
    pcl-conversions \
    pcl-ros \  
"
ROS_EXPORT_DEPENDS = " \
    roscpp \
    rospy \
    sensor-msgs \
    std-msgs \
    pcl-conversions \
    pcl-ros \
"

DEPENDS = "${ROS_BUILD_DEPENDS} ${ROS_BUILDTOOL_DEPENDS}"

RDEPENDS_${PN} += "${ROS_EXEC_DEPENDS}"
DEPENDS += "${ROS_EXPORT_DEPENDS}"

# SRC_URI = "git://github.com/author/basic_package.git"
SRC_URI = "file:///home/leonel/catkin_ws/src/ground_seg.zip"
SRCREV = "${AUTOREV}"

S = "${WORKDIR}/ground_seg"
ROS_BUILD_TYPE = "catkin"
inherit ros_${ROS_BUILD_TYPE}
