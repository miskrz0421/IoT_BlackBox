# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/esp-idf/components/bootloader/subproject"
  "C:/IoT_BlackBox/client/build/bootloader"
  "C:/IoT_BlackBox/client/build/bootloader-prefix"
  "C:/IoT_BlackBox/client/build/bootloader-prefix/tmp"
  "C:/IoT_BlackBox/client/build/bootloader-prefix/src/bootloader-stamp"
  "C:/IoT_BlackBox/client/build/bootloader-prefix/src"
  "C:/IoT_BlackBox/client/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/IoT_BlackBox/client/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/IoT_BlackBox/client/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
