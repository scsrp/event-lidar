# In script mode, CMAKE_SOURCE_DIR and CMAKE_CURRENT_BINARY_DIR point to the working directory.
# The caller expands CMAKE_SOURCE_DIR and passes the project root via -DSRC_DIR.
set(CMAKE_MODULE_PATH "${SRC_DIR}/cmake")

message("info: inject_version arguments list:")
message("info:     SRC_DIR = ${SRC_DIR}")
message("info:     INPUT   = ${INPUT}")
message("info:     OUTPUT  = ${OUTPUT}")

# NOTE: https://cmake.org/cmake/help/latest/command/string.html#timestamp
string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S")
message("info: BUILD_TIMESTAMP = ${BUILD_TIMESTAMP}")

include(detect_project_version)

detect_prj_ver(VERBOSE)
configure_file("${INPUT}" "${OUTPUT}")
