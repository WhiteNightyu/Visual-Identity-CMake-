aux_source_directory(. DIR_LIB)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
add_library(Mylib SHARED ${DIR_LIB})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})