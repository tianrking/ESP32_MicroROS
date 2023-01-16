# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/w0x7ce/Desktop/AAA/esp_idf_4.3/components/bootloader/subproject"
  "/home/w0x7ce/Desktop/AAA/esp32_diy/diy_esp32c3/PWM_DIY/build/bootloader"
  "/home/w0x7ce/Desktop/AAA/esp32_diy/diy_esp32c3/PWM_DIY/build/bootloader-prefix"
  "/home/w0x7ce/Desktop/AAA/esp32_diy/diy_esp32c3/PWM_DIY/build/bootloader-prefix/tmp"
  "/home/w0x7ce/Desktop/AAA/esp32_diy/diy_esp32c3/PWM_DIY/build/bootloader-prefix/src/bootloader-stamp"
  "/home/w0x7ce/Desktop/AAA/esp32_diy/diy_esp32c3/PWM_DIY/build/bootloader-prefix/src"
  "/home/w0x7ce/Desktop/AAA/esp32_diy/diy_esp32c3/PWM_DIY/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/w0x7ce/Desktop/AAA/esp32_diy/diy_esp32c3/PWM_DIY/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/w0x7ce/Desktop/AAA/esp32_diy/diy_esp32c3/PWM_DIY/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
