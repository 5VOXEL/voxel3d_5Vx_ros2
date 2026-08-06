#------------------------------------------------------------------------------
# Copyright (C) 2026 pmdtechnologies gmbh
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.24)
project(voxel3d_node)

# Default to a Release build if the caller didn't specify one. Without this,
# CMake's default is an *unoptimized* build (no -O flag at all) on Makefiles/
# Ninja, which is a real, measurable difference for the per-frame timing in
# this node (waitForFrames()/runSegment() etc.) -- not just a few percent.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  message(STATUS "CMAKE_BUILD_TYPE not specified, defaulting to Release")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # Be explicit instead of relying on CMake's default Release flags (-O3
  # -DNDEBUG on GCC/Clang), so this doesn't silently change if the default
  # ever does.
  add_compile_options($<$<CONFIG:Release>:-O3>)
  add_compile_options($<$<CONFIG:Release>:-DNDEBUG>)
endif()

find_package(rclcpp QUIET)  # ROS2 
if(rclcpp_FOUND)
    set(USE_ROS2 TRUE)
    message(STATUS "Using ROS2")
    add_definitions(-DROS2)
    find_package(ament_cmake REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(std_msgs REQUIRED)
else()
    find_package(catkin QUIET COMPONENTS roscpp sensor_msgs cv_bridge pluginlib) # ROS1
    if(catkin_FOUND)
        set(USE_ROS1 TRUE)
        message(STATUS "Using ROS1")
        add_definitions(-DROS1)
    else()
        message(WARNING "ROS not found. Cannot build the ROS node example.")
        return ()
    endif()
endif()

find_package(OpenCV REQUIRED)

if(USE_ROS1)
  catkin_package(
                INCLUDE_DIRS ""
                CATKIN_DEPENDS cv_bridge pluginlib rviz
  )
endif()



set(VOXEL3D_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
# The actual voxel3d.h / libvoxel3d.so live at the repo root (voxel3d/inc,
# voxel3d/lib), three directories up from here
# (voxel3d/node/ros2/voxel3d_node) -- not inside this package's own inc/.
set(VOXEL3D_ROOT_DIR "${VOXEL3D_DIR}/../../..")
set(VOXEL3D_INCLUDE_DIR "${VOXEL3D_ROOT_DIR}/inc")
set(VOXEL3D_LIB_DIR "${VOXEL3D_ROOT_DIR}/lib")
set(VOXEL3D_LIB "${VOXEL3D_LIB_DIR}/libvoxel3d.so")


set (SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/voxel3d_node.cpp")

# Models/ and Helpers/ are siblings of inc/ and lib/ at the repo root, not
# nested under inc/.
if(USE_ROS2)
  include_directories (
                    ${VOXEL3D_INCLUDE_DIR}
                    ${VOXEL3D_ROOT_DIR}/Models
                    ${VOXEL3D_ROOT_DIR}/Helpers
                    inc
                    ${OpenCV_INCLUDE_DIRS}
  )
else()
  include_directories(
                      ${VOXEL3D_INCLUDE_DIR}
                      ${VOXEL3D_ROOT_DIR}/Models
                      ${VOXEL3D_ROOT_DIR}/Helpers
                      ${OpenCV_INCLUDE_DIRS}
                      ${catkin_INCLUDE_DIRS}
  )
endif()

# NOTE: was link_directories(${VOXEL3D_DIR}) -- that's this package's own
# source directory, not where libvoxel3d.so actually lives.
link_directories(${VOXEL3D_LIB_DIR})

add_executable(voxel3d_node ${SOURCES})

if(USE_ROS2)
  target_link_libraries(voxel3d_node
                        ${VOXEL3D_LIB}
                        ${OBS_CLOUD_BUILDER_LIB}
                        ${OpenCV_LIBS})
else()
  target_link_libraries(voxel3d_node 
                        ${VOXEL3D_LIB}
                        ${OBS_CLOUD_BUILDER_LIB}
                        ${catkin_LIBRARIES}
                        ${OpenCV_LIBS})  
endif()

set_target_properties (voxel3d_node PROPERTIES FOLDER ../examples)

set_target_properties(voxel3d_node PROPERTIES
    BUILD_RPATH "${VOXEL3D_LIB_DIR}"
    INSTALL_RPATH "${VOXEL3D_LIB_DIR}"
    INSTALL_RPATH_USE_LINK_PATH TRUE)

if(USE_ROS2)
  ament_target_dependencies(voxel3d_node rclcpp sensor_msgs cv_bridge nav_msgs visualization_msgs geometry_msgs tf2_ros std_msgs)
  install(TARGETS voxel3d_node DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "Catkin Package: ${CATKIN_PACKAGE_BIN_DESTINATION}")
  install(TARGETS voxel3d_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()


if(USE_ROS2)
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
else()
  install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )
endif()


find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
if(USE_ROS2)
  find_package(rviz_common REQUIRED)
  find_package(pluginlib REQUIRED)
else()
  find_package(rviz REQUIRED)
endif()

set (CMAKE_INCLUDE_CURRENT_DIR ON)

set(MOC_HEADER 
  "${CMAKE_CURRENT_SOURCE_DIR}/inc/voxel3d_node/voxel3d_panel.hpp")

qt5_wrap_cpp(MOCED_HEADER ${MOC_HEADER})

add_library(voxel3d_panel SHARED src/voxel3d_panel.cpp ${MOCED_HEADER})
target_include_directories(voxel3d_panel PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
                            $<INSTALL_INTERFACE:include>
)  
if(USE_ROS2)        
  ament_target_dependencies(voxel3d_panel pluginlib rviz_common rclcpp Qt5Widgets cv_bridge)
else()
  target_link_libraries(voxel3d_panel ${catkin_LIBRARIES} Qt5::Widgets ${OpenCV_LIBRARIES})
endif()

target_include_directories(voxel3d_node PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/inc"
)
install(TARGETS voxel3d_panel
        EXPORT export_voxel3d_panel
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY inc/
        DESTINATION include
)

if(USE_ROS1)
  set(LIBRARY_PATH "lib/voxel3d_panel")
else()
  set(LIBRARY_PATH "voxel3d_panel")
endif()

if(USE_ROS2)
  set(RVIZ_BASE_CLASS "rviz_common::Panel")
else()
  set(RVIZ_BASE_CLASS "rviz::Panel")
endif()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/rviz_common_plugins.xml.in
  ${CMAKE_CURRENT_SOURCE_DIR}/rviz_common_plugins.xml 
  @ONLY 
)

if(USE_ROS1)
  install(FILES rviz_common_plugins.xml
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endif()

if(USE_ROS2)  
  install(FILES rviz_common_plugins.xml
          DESTINATION share/${PROJECT_NAME}
  )
  ament_export_include_directories(include)
  ament_export_targets(export_voxel3d_panel)
  pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)
endif()

if(BUILD_TESTING)
  if(USE_ROS2)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

if(USE_ROS2)
  ament_package()
endif()