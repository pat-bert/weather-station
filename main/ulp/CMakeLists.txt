cmake_minimum_required(VERSION 3.16)

# Project/target name is passed from the main project to allow IDF to have a dependency on this target
# as well as embed the binary into the main app
project(${ULP_APP_NAME})
add_executable(${ULP_APP_NAME} main.cpp)

# Import the ULP project helper functions
include(IDFULPProject)

# Apply default compile options
ulp_apply_default_options(${ULP_APP_NAME})

# Apply default sources provided by the IDF ULP component
ulp_apply_default_sources(${ULP_APP_NAME})

# Add targets for building the binary, as well as the linkerscript which exports ULP shared variables to the main app
ulp_add_build_binary_targets(${ULP_APP_NAME})

# Create sensor library
set(lib_path "${CMAKE_CURRENT_LIST_DIR}/..")
add_library(sensor_lib 
    STATIC 
        "${lib_path}/bh1750/bh1750.cpp"
        "${lib_path}/bme280/bme280.c"
)
target_include_directories(sensor_lib 
    PUBLIC 
        "${lib_path}/bh1750"
        "${lib_path}/bme280"
)

# Link the library
target_link_libraries(${ULP_APP_NAME} PRIVATE sensor_lib)

add_compile_definitions(BME280_64BIT_ENABLE)