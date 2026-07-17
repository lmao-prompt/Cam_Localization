# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/prhayogo/Downloads/Nexus_Gazebo/firmware/dev_ws/ament/ament_cmake/ament_cmake_vendor_package/test/depender"
  "/home/prhayogo/Downloads/Nexus_Gazebo/firmware/dev_ws/build/ament_cmake_vendor_package/test/depender-prefix/src/depender-build"
  "/home/prhayogo/Downloads/Nexus_Gazebo/firmware/dev_ws/build/ament_cmake_vendor_package/test/depender-prefix/install"
  "/home/prhayogo/Downloads/Nexus_Gazebo/firmware/dev_ws/build/ament_cmake_vendor_package/test/depender-prefix/tmp"
  "/home/prhayogo/Downloads/Nexus_Gazebo/firmware/dev_ws/build/ament_cmake_vendor_package/test/depender-prefix/src/depender-stamp"
  "/home/prhayogo/Downloads/Nexus_Gazebo/firmware/dev_ws/build/ament_cmake_vendor_package/test/depender-prefix/src"
  "/home/prhayogo/Downloads/Nexus_Gazebo/firmware/dev_ws/build/ament_cmake_vendor_package/test/depender-prefix/src/depender-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/prhayogo/Downloads/Nexus_Gazebo/firmware/dev_ws/build/ament_cmake_vendor_package/test/depender-prefix/src/depender-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/prhayogo/Downloads/Nexus_Gazebo/firmware/dev_ws/build/ament_cmake_vendor_package/test/depender-prefix/src/depender-stamp${cfgdir}") # cfgdir has leading slash
endif()
