get_hwc_lib_name("${CMAKE_CURRENT_SOURCE_DIR}" lib_name)
project(${lib_name} C CXX ASM)

get_hwc_lib_dir("${CMAKE_CURRENT_SOURCE_DIR}" lib_dir)
set(LIBRARY_OUTPUT_PATH ${lib_dir})

set_property(GLOBAL PROPERTY ${lib_name}_libs)
set_property(GLOBAL PROPERTY ${lib_name}_incs)
set_property(GLOBAL PROPERTY ${lib_name}_srcs)

set(CMSISDSP "${CMAKE_CURRENT_SOURCE_DIR}/CMSIS-DSP-1.14.2")

search_incs_recurse(${CMAKE_CURRENT_SOURCE_DIR}/Eigen
                    ${lib_name}_public_includes)
remove_redundant_include_directories(${lib_name}_public_includes)

set(${lib_name}_private_includes ${CMAKE_CURRENT_SOURCE_DIR}/PrivateInclude)

file(GLOB_RECURSE ${lib_name}_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.*)

# add_library(${lib_name} STATIC ${${lib_name}_sources})

# target_include_directories( ${lib_name} PUBLIC ${${lib_name}_public_includes}
# PRIVATE ${${lib_name}_private_includes})

# set_property(GLOBAL APPEND PROPERTY ${lib_name}_libs ${lib_name})
set_property(GLOBAL APPEND PROPERTY ${lib_name}_incs
                                    ${${lib_name}_public_includes})
set_property(GLOBAL APPEND PROPERTY ${lib_name}_srcs ${${lib_name}_sources})
