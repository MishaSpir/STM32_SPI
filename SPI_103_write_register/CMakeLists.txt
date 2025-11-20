#================================================================================
cmake_minimum_required(VERSION 3.20)
set(CMAKE_TOOLCHAIN_FILE cmake/arm-none-eabi-gcc.cmake)


#================================================================================
project(SPI_103_send LANGUAGES CXX C ASM)
set(DEVICE stm32f103c8)

include(libopencm3)


#================================================================================
add_executable(${PROJECT_NAME}
     src/main.cpp
     # src/Pars.cpp
     src/setup.cpp
     src/NRF.cpp
     # src/LiquidCrystalSTM.cpp
)

target_include_directories(${PROJECT_NAME}
     PRIVATE
     ${CMAKE_CURRENT_LIST_DIR}
      ${CMAKE_CURRENT_LIST_DIR}/inc

)

target_link_libraries(${PROJECT_NAME}
     PRIVATE
     libopencm3
)

set_target_properties(${PROJECT_NAME}
        PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)

add_custom_command(TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND cp -u ${CMAKE_BINARY_DIR}/${PROJECT_NAME} ${CMAKE_BINARY_DIR}/firmware
)


#================================================================================
set(PROG stlink-v2)
string(CONCAT PROG_TARGET ${LOCM3_DEVICE_SERIE} x)

include(oocd_flash_script)

install(CODE ${OOCD_FLASH_SCRIPT})

