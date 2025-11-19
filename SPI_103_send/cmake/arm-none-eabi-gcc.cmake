MESSAGE(STATUS "Toolchain file loading")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})

set(ENV{CMAKE_EXPORT_COMPILE_COMMANDS} 1)


set(CMAKE_SYSTEM_NAME  Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)
set(CMAKE_CROSSCOMPILING 1)

set(TOOLCHAIN_PREFIX arm-none-eabi-)



if(MINGW OR CYGWIN OR WIN32)
    set(UTIL_SEARCH_CMD where)
elseif(UNIX OR APPLE)
    set(UTIL_SEARCH_CMD which)
endif()

if(CMAKE_HOST_WIN32)
    set(CMAKE_EXECUTABLE_SUFFIX .exe)
endif()

execute_process(
  COMMAND ${UTIL_SEARCH_CMD} arm-none-eabi-gcc
  OUTPUT_VARIABLE TOOLCHAIN_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

get_filename_component(TOOLCHAIN_DIR ${TOOLCHAIN_PATH} DIRECTORY)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER ${TOOLCHAIN_DIR}/arm-none-eabi-gcc${CMAKE_EXECUTABLE_SUFFIX})
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_DIR}/arm-none-eabi-gcc${CMAKE_EXECUTABLE_SUFFIX})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_DIR}/arm-none-eabi-g++${CMAKE_EXECUTABLE_SUFFIX})

set(SIZE_UTIL ${TOOLCHAIN_DIR}/arm-none-eabi-size${CMAKE_EXECUTABLE_SUFFIX})
set(OBJDUMP_UTIL ${TOOLCHAIN_DIR}/arm-none-eabi-objdump${CMAKE_EXECUTABLE_SUFFIX})
set(OBJCOPY_UTIL ${TOOLCHAIN_DIR}/arm-none-eabi-objcopy${CMAKE_EXECUTABLE_SUFFIX})


set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)


# Настройки компилятора
add_compile_options(
     -ggdb3
     -Os

     -fdata-sections
     -ffunction-sections

     -fno-common

     -fno-exceptions

     -Wall
     -Wextra
     -Wshadow
     -Wdouble-promotion
     -Wundef
     -Wconversion
)

# Настройки компоновщика
add_link_options(
     -nostartfiles

     -specs=nano.specs
     -specs=nosys.specs
     -lc
     -lm
     -lnosys

     -Wl,--gc-sections
)
