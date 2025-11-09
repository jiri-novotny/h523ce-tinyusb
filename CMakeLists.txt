cmake_minimum_required(VERSION 3.12)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(h523ce-tinyusb C ASM)

message("-- Build type: " ${CMAKE_BUILD_TYPE})

# Project-specific
add_definitions(-DUSE_HAL_DRIVER)
add_definitions(-DUSE_FULL_LL_DRIVER)

include_directories(app)
include_directories(drivers)
include_directories(lib/cmsis-core/Include)

add_subdirectory(app)
add_subdirectory(lib)

if(stm32h5)
  set(linker_script_SRC ${CMAKE_CURRENT_SOURCE_DIR}/lib/cmsis-device-h5/Source/Templates/gcc/linker/STM32H523xx_FLASH.ld)
  set(CPU_CORE cortex-m33)

  add_definitions(-DCMSIS_device_header="stm32h5xx.h")
  add_definitions(-DSTM32H523xx)

  include_directories(drivers_h5)
  include_directories(lib/cmsis-device-h5/Include)
  include_directories(lib/stm32h5xx-hal-driver/Inc)

  add_subdirectory(drivers_h5)
else()
  message(FATAL_ERROR "Please select target platform!")
endif()

# Executable files
add_executable(${PROJECT_NAME} main.c syscalls.c ${SOURCE})

target_compile_options(${PROJECT_NAME} PRIVATE
    -mcpu=${CPU_CORE}
    $<$<CONFIG:Debug>:-Og -g3>
    $<$<CONFIG:MinSizeRel>:-Os>
    $<$<CONFIG:Release>:-O3>
    -ffunction-sections -fdata-sections
    -Wall
    -Wextra
    -Werror
    -pedantic
    -Wno-unused-parameter
    -fstack-usage -fcyclomatic-complexity
    --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb
)

# Linker options
target_link_options(${PROJECT_NAME} PRIVATE
    -mcpu=${CPU_CORE}
    -T${linker_script_SRC}
    --specs=nosys.specs
    -Wl,-Map=${PROJECT_NAME}.map
    -Wl,--gc-sections -static
    -Wl,--print-memory-usage
    --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb
    -Wl,--start-group -lc -lm -Wl,--end-group
)

# Execute post-build to print size, generate hex and bin
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND "echo"
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
)
