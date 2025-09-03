cmake_minimum_required(VERSION 3.9)
project(BRepReCon)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set version
set(VERSION 1.0)

# OpenMP support (Windows MSVC)
option(BUILD_OPENMP "Enable support for OpenMP" ON)
if (BUILD_OPENMP)
    find_package(OpenMP REQUIRED)
endif()

# TBB support
option(BUILD_TBB "Enable support for TBB" ON)
if (BUILD_TBB)
    find_package(TBB REQUIRED)
endif()

# OpenCASCADE setup
find_package(OpenCASCADE REQUIRED)
include_directories(${OpenCASCADE_INCLUDE_DIR})
get_filename_component(PARENT_DIR ${OpenCASCADE_INSTALL_PREFIX} PATH)

# Set OpenCASCADE options
set(THIRD_PARTY_DIR ${PARENT_DIR}/products)  # User-defined folder for third-party libraries
set(OpenCASCADE_WITH_TCL ON)
set(OpenCASCADE_WITH_FREETYPE ON)
set(OpenCASCADE_WITH_FREEIMAGE OFF)
set(OpenCASCADE_WITH_TBB OFF)
set(OpenCASCADE_WITH_VTK OFF)
set(OpenCASCADE_WITH_FFMPEG OFF)
set(OpenCASCADE_WITH_GLES2 OFF)
set(OpenCASCADE_WITH_D3D OFF)

# Eigen3 setup
find_package(Eigen3 REQUIRED)


#nlohmann json
find_package(nlohmann_json CONFIG REQUIRED)


# LBFGSpp
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/lib/LBFGSpp/include")

# gco (Graph Cuts Optimization)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/lib/gco")

# Collect source and header files from the main src directory and subdirectories
file(GLOB SOURCES
    src/*.cpp
    src/*.h
    src/*.hpp
    src/*.cxx
)

#Zlib
find_package(ZLIB REQUIRED)
include_directories(${ZLIB_INCLUDE_DIRS})


# GCO sources
file(GLOB GCO_SOURCES lib/gco/*.cpp)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${NLOPT_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# Add definitions for PCL
add_definitions(${PCL_DEFINITIONS})

# **创建可执行文件**
add_executable(BRepReCon ${SOURCES} ${GCO_SOURCES} ${QRC_FILES} ${WRAP_FILES})

# OpenMP
if (BUILD_OPENMP AND OpenMP_CXX_FOUND)
    set(OpenMP_VERSION 201107)  # OpenMP 3.1 (201107)
    target_compile_definitions(BRepReCon PRIVATE -DWITH_OMP -DOMP_VER=${OpenMP_VERSION})
    target_compile_options(BRepReCon PRIVATE /openmp)
    target_link_libraries(BRepReCon PRIVATE OpenMP::OpenMP_CXX)
endif()

# Set output properties
set_target_properties(BRepReCon PROPERTIES
    LINK_FLAGS "/SUBSYSTEM:CONSOLE /ENTRY:mainCRTStartup /OUT:\"$(OutDir)$(TargetName)$(TargetExt)\""
)

# OpenCASCADE libraries
foreach(LIB ${OpenCASCADE_LIBRARIES})
    target_link_libraries(BRepReCon PRIVATE ${OpenCASCADE_LIBRARY_DIR}/${LIB}.lib)
endforeach()

# libigl
find_package(libigl REQUIRED QUIET)

# Link additional libraries
target_link_libraries(BRepReCon PRIVATE 
    ZLIB::ZLIB
    nlohmann_json::nlohmann_json
)

# GTE include directory
target_include_directories(BRepReCon PRIVATE lib)

# Adjust runtime environment for debugging
message(${OpenCASCADE_BINARY_DIR})
set_property(TARGET BRepReCon PROPERTY VS_DEBUGGER_ENVIRONMENT
    "PATH=$<$<CONFIG:DEBUG>:${OpenCASCADE_BINARY_DIR}d>$<$<NOT:$<CONFIG:DEBUG>>:${OpenCASCADE_BINARY_DIR}>;%PATH%"
)

# Define Unicode for the project
add_definitions(-DUNICODE)