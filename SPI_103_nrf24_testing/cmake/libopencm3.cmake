message(CHECK_START "Include libopencm3")

if(NOT LIBOPENCM3_REPOSITORY)
  set(LIBOPENCM3_REPOSITORY https://github.com/libopencm3/libopencm3.git)
  message(STATUS "Select default libopencm3 repository.")
endif()

if(NOT DEVICE)
  set(DEVICE stm32f407vg)
  message(STATUS "Select default device.")
endif()

string(ASCII 27 ESC)
message(STATUS "${ESC}[33mUse repository: " ${LIBOPENCM3_REPOSITORY} "${ESC}[0m")
message(STATUS "${ESC}[33mDevice selected: " ${DEVICE} "${ESC}[0m")


include(FetchContent)

FetchContent_Declare(libopencm3_repo
  GIT_REPOSITORY ${LIBOPENCM3_REPOSITORY}
  GIT_TAG        master
)

FetchContent_GetProperties(libopencm3_repo)
if(NOT libopencm3_repo_POPULATED)
     FetchContent_Populate(libopencm3_repo)
endif()


set(LOCM3_DEVICE_SCRIPT ${libopencm3_repo_SOURCE_DIR}/scripts/genlink.py)
set(LOCM3_DEVICE_DATABASE ${libopencm3_repo_SOURCE_DIR}/ld/devices.data)
set(LOCM3_LINKER_TEMPLATE ${libopencm3_repo_SOURCE_DIR}/ld/linker.ld.S)
set(LOCM3_LINKER_GENERATED ${libopencm3_repo_SOURCE_DIR}/lib/generated.${DEVICE}.ld)
set(LOCM3_INCLUDE_DIR ${libopencm3_repo_SOURCE_DIR}/include)
set(LOCM3_STATIC_LIB_PREFIX ${libopencm3_repo_SOURCE_DIR}/lib/libopencm3_)


message(CHECK_START "Query device info and generate linker script")
if(CMAKE_HOST_WIN32)
     execute_process( COMMAND python ${LOCM3_DEVICE_SCRIPT} ${LOCM3_DEVICE_DATABASE} ${DEVICE} "CPPFLAGS" OUTPUT_VARIABLE LOCM3_DEVICE_DEFINES OUTPUT_STRIP_TRAILING_WHITESPACE)
     execute_process( COMMAND python ${LOCM3_DEVICE_SCRIPT} ${LOCM3_DEVICE_DATABASE} ${DEVICE} "CPU" OUTPUT_VARIABLE MCPU OUTPUT_STRIP_TRAILING_WHITESPACE)
     execute_process( COMMAND python ${LOCM3_DEVICE_SCRIPT} ${LOCM3_DEVICE_DATABASE} ${DEVICE} "FPU" OUTPUT_VARIABLE FPU_STR OUTPUT_STRIP_TRAILING_WHITESPACE)
     execute_process( COMMAND python ${LOCM3_DEVICE_SCRIPT} ${LOCM3_DEVICE_DATABASE} ${DEVICE} "DEFS" OUTPUT_VARIABLE DEVDEFS OUTPUT_STRIP_TRAILING_WHITESPACE)
     string(APPEND DEVDEFS " -P -E " ${LOCM3_LINKER_TEMPLATE} " " -o " " ${LOCM3_LINKER_GENERATED})
     separate_arguments(GEN_LD UNIX_COMMAND ${DEVDEFS})
     execute_process(COMMAND ${CMAKE_C_COMPILER} ${GEN_LD})
else()
     execute_process( COMMAND ${LOCM3_DEVICE_SCRIPT} ${LOCM3_DEVICE_DATABASE} ${DEVICE} "CPPFLAGS" OUTPUT_VARIABLE LOCM3_DEVICE_DEFINES OUTPUT_STRIP_TRAILING_WHITESPACE)
     execute_process( COMMAND ${LOCM3_DEVICE_SCRIPT} ${LOCM3_DEVICE_DATABASE} ${DEVICE} "CPU" OUTPUT_VARIABLE MCPU OUTPUT_STRIP_TRAILING_WHITESPACE)
     execute_process( COMMAND ${LOCM3_DEVICE_SCRIPT} ${LOCM3_DEVICE_DATABASE} ${DEVICE} "FPU" OUTPUT_VARIABLE FPU_STR OUTPUT_STRIP_TRAILING_WHITESPACE)
     execute_process( COMMAND ${LOCM3_DEVICE_SCRIPT} ${LOCM3_DEVICE_DATABASE} ${DEVICE} "DEFS" OUTPUT_VARIABLE DEVDEFS OUTPUT_STRIP_TRAILING_WHITESPACE)
     string(APPEND DEVDEFS " -P -E " ${LOCM3_LINKER_TEMPLATE} " " -o " " ${LOCM3_LINKER_GENERATED})
     separate_arguments(GEN_LD UNIX_COMMAND ${DEVDEFS})
     execute_process(COMMAND ${CMAKE_C_COMPILER} ${GEN_LD})
endif()
message(CHECK_PASS "Done")


string(TOLOWER ${LOCM3_DEVICE_DEFINES} LOCM3_DEVICE_DEFINES_LOWCASE)
string(REPLACE -d "" LOCM3_DEVICE_STRING ${LOCM3_DEVICE_DEFINES_LOWCASE})
string(REPLACE " " ";" LOCM3_DEVICE_LIST ${LOCM3_DEVICE_STRING})
list(GET LOCM3_DEVICE_LIST 0 LOCM3_DEVICE_SERIE)
string(REPLACE stm32 stm32/ LOCM3_TARGET ${LOCM3_DEVICE_SERIE})

message(STATUS "Libopencm3 build target: ${LOCM3_TARGET}")


set(THUMB_CORES cortex-m0 cortex-m0plus cortex-m3 cortex-m4 cortex-m7)
if(MCPU IN_LIST THUMB_CORES)
  set(MTHUMB "-mthumb")
endif()
string(PREPEND MCPU "-mcpu=")


if(FPU_STR STREQUAL "soft")
  set(MFLOAT_ABI "-msoft-float")
elseif(FPU_STR STREQUAL "hard-fpv4-sp-d16")
  set(MFLOAT_ABI "-mfloat-abi=hard")
  set(MFPU "-mfpu=fpv4-sp-d16")
elseif(FPU_STR STREQUAL "hard-fpv5-sp-d16")
  set(MFLOAT_ABI "-mfloat-abi=hard")
  set(MFPU "-mfpu=fpv5-sp-d16")
else()
  message(WARNING "Float ABI not defined!")
endif()


add_custom_target(libopencm3_lib ALL COMMAND make TARGETS=${LOCM3_TARGET} WORKING_DIRECTORY ${libopencm3_repo_SOURCE_DIR})

add_library(libopencm3 INTERFACE)
add_dependencies(libopencm3 libopencm3_lib)

target_compile_definitions(libopencm3 INTERFACE ${LOCM3_DEVICE_DEFINES})
target_include_directories(libopencm3 INTERFACE  ${LOCM3_INCLUDE_DIR})
target_link_libraries(libopencm3 INTERFACE ${LOCM3_STATIC_LIB_PREFIX}${LOCM3_DEVICE_SERIE}.a)
target_link_options(libopencm3 INTERFACE -T${LOCM3_LINKER_GENERATED})

add_compile_options(${MCPU} ${MTHUMB} ${MFPU} ${MFLOAT_ABI})
add_link_options(${MCPU} ${MTHUMB} ${MFPU} ${MFLOAT_ABI})

message(CHECK_PASS "done")

