##**************************************************************************************##
##                                    Option                                            ##
##**************************************************************************************##

# ==================================================
#                     Description
# ==================================================

# EL_ENABLE_STATIC_MSVC_CRT
#     - Statically link the CRT when building the Python binding with MSVC.
#     - default = 0 (use the same dynamic CRT as Python).
# EL_MSVC_MP
#     - Control MSVC parallel compilation with /MP[N]. A positive integer N enables /MP<N>
#       (up to N translation units per cl.exe process); zero or a negative integer enables /MP
#       (use the processor count); an empty string disables /MP.
#     - Applies to the Python binding's MSVC build through CMAKE_*_FLAGS.
#     - default = "" (disabled).
# ==================================================
#                     Default
# ==================================================

if (NOT DEFINED EL_ENABLE_STATIC_MSVC_CRT)
    set(EL_ENABLE_STATIC_MSVC_CRT 0)
endif()
if (NOT DEFINED EL_MSVC_MP)
    set(EL_MSVC_MP "")
endif()
# ==================================================
#                        Print
# ==================================================

message("info: Option values:")
message("info:     EL_ENABLE_STATIC_MSVC_CRT      = ${EL_ENABLE_STATIC_MSVC_CRT}")
message("info:     EL_MSVC_MP                     = ${EL_MSVC_MP}")
##**************************************************************************************##
##                                      Common                                          ##
##**************************************************************************************##

# ==================================================
#                      Variables
# ==================================================

# --- dir

# Directory for source files generated during the build.
set(GENERATED_SRC_DIR_REL "generated")
set(GENERATED_SRC_DIR_ABS "${CMAKE_CURRENT_BINARY_DIR}/${GENERATED_SRC_DIR_REL}")

# ==================================================
#                cmake_minimum_required
# ==================================================

# cmake version
cmake_minimum_required(VERSION 3.16...3.27)

# ==================================================
#               MSVCRT linkage (MSVC)
# ==================================================

# EL_ENABLE_STATIC_MSVC_CRT == 1:
#   Release: MultiThreaded      -> /MT
#   Debug:   MultiThreadedDebug -> /MTd
# EL_ENABLE_STATIC_MSVC_CRT == 0:
#   Release: MultiThreadedDLL      -> /MD
#   Debug:   MultiThreadedDebugDLL -> /MDd
set(CMAKE_MSVC_RUNTIME_LIBRARY
    "MultiThreaded$<$<CONFIG:Debug>:Debug>$<$<NOT:${EL_ENABLE_STATIC_MSVC_CRT}>:DLL>"
    CACHE STRING "MSVC runtime library" FORCE
)

message("info: CMAKE_MSVC_RUNTIME_LIBRARY = ${CMAKE_MSVC_RUNTIME_LIBRARY}")

# ==================================================
#                      Settings
# ==================================================

# project name
project(event_lidar CXX)

# Use C++20 by default.
if (NOT DEFINED CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

message("info: CMAKE_CXX_STANDARD = ${CMAKE_CXX_STANDARD}")

##**************************************************************************************##
##                                  PYTHON BUILD                                        ##
##**************************************************************************************##

if (SKBUILD)

    message("build = PYTHON")

    # ==================================================
    #                     packages
    # ==================================================

    # ------------------------------
    #             python
    # ------------------------------

    message("info: CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")
    message("info: CMAKE_CURRENT_BINARY_DIR = ${CMAKE_CURRENT_BINARY_DIR}")
    message("info: CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")

    # Virtual environment preferred
    set(Python_FIND_VIRTUALENV FIRST)

    message("info: Finding Python 3.8 or newer")

    find_package(Python 3.8
        REQUIRED COMPONENTS Interpreter Development.Module
        OPTIONAL_COMPONENTS Development.SABIModule)

    # ------------------------------
    #            nanobind
    # ------------------------------

    find_package(nanobind CONFIG REQUIRED)

    # ------------------------------
    #             fmt
    # ------------------------------

    # NOTE: https://fmt.dev/12.0/get-started/
    include(FetchContent)
    FetchContent_Declare(
        fmt
        GIT_REPOSITORY https://github.com/fmtlib/fmt
        GIT_TAG        12.1.0) # fmt 10.2.1 fails to build with Clang 20+ due to stricter consteval checks.
    FetchContent_MakeAvailable(fmt)

    # Enable PIC so a static fmt build can be linked into the Python extension module (a shared library) on Unix-like systems.
    # (BUILD_SHARED_LIBS can make fmt a shared library.)
    if (UNIX)
        set_target_properties(fmt PROPERTIES POSITION_INDEPENDENT_CODE ON)
    endif()

    # ==============================
    #              var
    # ==============================

    set(INSTALL_PATH "${CMAKE_PROJECT_NAME}")
    set(TARGET_NAME  "${CMAKE_PROJECT_NAME}_ext")

    # ==============================
    #            files
    # ==============================

    file(GLOB_RECURSE CXX_SOURCE RELATIVE "${CMAKE_SOURCE_DIR}" CONFIGURE_DEPENDS "src/*.cpp")
    file(GLOB_RECURSE CXX_HEADER RELATIVE "${CMAKE_SOURCE_DIR}" CONFIGURE_DEPENDS "src/*.hpp" "src/*.h")

    message("info: PROJECT_NAME         = ${PROJECT_NAME}")
    message("info: TARGET_NAME          = ${TARGET_NAME}")
    message("info: CXX_SOURCE           = ${CXX_SOURCE}")
    message("info: CXX_HEADER           = ${CXX_HEADER}")

    # ==============================
    #           option
    # ==============================

    if(MSVC)
        message("info: compiler = MSVC")

        set(CMAKE_CXX_FLAGS_RELEASE
            "${CMAKE_CXX_FLAGS_RELEASE} /Ox /Ob2 /Oi /Ot /Oy /GT /GL /GS- /Gy /QIntel-jcc-erratum /Qpar /favor:INTEL64 /GA"
        )
        set(CMAKE_CXX_FLAGS_DEBUG
            "${CMAKE_CXX_FLAGS_DEBUG} /W4"
        )

        # common
        set(CMAKE_CXX_FLAGS
            "${CMAKE_CXX_FLAGS} /arch:AVX2 /Zc:preprocessor"
        )

        # MSVC parallel compilation in Python builds.
        #   - Positive integer N: /MP<N> (up to N translation units per cl.exe process)
        #   - Zero or negative integer: /MP (automatically use the processor count)
        #   - Empty string (default): omit /MP (disabled)
        if (EL_MSVC_MP MATCHES "^[1-9][0-9]*$")
            set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} /MP${EL_MSVC_MP}")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP${EL_MSVC_MP}")
            message("info: MSVC parallel compile -> /MP${EL_MSVC_MP}")
        elseif (EL_MSVC_MP MATCHES "^-?[0-9]+$")
            set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} /MP")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
            message("info: MSVC parallel compile -> /MP (auto)")
        elseif (NOT EL_MSVC_MP STREQUAL "")
            message(WARNING "EL_MSVC_MP='${EL_MSVC_MP}' is not an integer; ignoring (no /MP).")
        endif()

        # NOTE: https://cmake.org/cmake/help/latest/variable/CMAKE_BUILD_TYPE.html
        set(CMAKE_BUILD_TYPE Release)
        message("info: CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

    elseif(MINGW OR UNIX)

        message("info: env = MINGW or UNIX")

        set(CMAKE_CXX_FLAGS
            "${CMAKE_CXX_FLAGS} -Wall -O3 -mtune=native -march=native -pipe -funroll-loops -ffast-math -flto=auto -fforce-addr -fdevirtualize-speculatively"
        )

        if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            message(STATUS "Compiler = GCC")

            # nanobind_add_module() builds the nanobind static library with the configured CMAKE_CXX_FLAGS,
            # producing an LTO-enabled static library.
            # LTO requires an intermediate representation (IR), so GCC must use gcc-ar instead of the default archiver
            # to handle object files containing the IR emitted by the compiler.
            # Without this configuration, linking may fail with "plugin needed to handle lto object".
            # Use the compiler-specific archiver path from CMAKE_CXX_COMPILER_AR when available; otherwise, fall back to
            # the system gcc-ar.
            if (NOT APPLE)
                if (CMAKE_CXX_COMPILER_AR)
                    set(CMAKE_AR "${CMAKE_CXX_COMPILER_AR}")
                else()
                    set(CMAKE_AR "gcc-ar")
                endif()
                set(CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> qcs <TARGET> <LINK_FLAGS> <OBJECTS>")
                set(CMAKE_CXX_ARCHIVE_FINISH true)
            endif()
        endif()

    else()
        message(
            WARNING
            "No project-specific optimization flags are configured for this platform."
        )
    endif()

    # ==================================================
    #                 Install
    # ==================================================

    include_directories(${CMAKE_CURRENT_BINARY_DIR})
    include_directories(${GENERATED_SRC_DIR_ABS})

    # Create a CMake MODULE library target for the Python extension.
    nanobind_add_module(
        ${TARGET_NAME} STABLE_ABI NB_STATIC ${CXX_SOURCE} ${CXX_HEADER})

    # version
    add_custom_target(
        VersionUpdater
        # Pass arguments to inject_version.cmake via -D variables.
        COMMAND   ${CMAKE_COMMAND}
                -DSRC_DIR=${CMAKE_SOURCE_DIR}
                -DINPUT=${CMAKE_SOURCE_DIR}/src/version.hpp.in
                -DOUTPUT=${GENERATED_SRC_DIR_ABS}/version.hpp
                -P "${CMAKE_SOURCE_DIR}/cmake/inject_version.cmake"
        DEPENDS "${CMAKE_SOURCE_DIR}/src/version.hpp.in"
                "${CMAKE_SOURCE_DIR}/cmake/inject_version.cmake"
                "${CMAKE_SOURCE_DIR}/cmake/detect_project_version.cmake"
        # Tell CMake that this target generates version.hpp as a byproduct.
        BYPRODUCTS "${GENERATED_SRC_DIR_ABS}/version.hpp"
        # Escape command arguments for the build tool so they reach the command unchanged.
        VERBATIM
    )
    add_dependencies(${TARGET_NAME} VersionUpdater)
    target_sources(${TARGET_NAME} PRIVATE "${GENERATED_SRC_DIR_ABS}/version.hpp")

    # Add include directories.
    target_include_directories(
        ${TARGET_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/src")
    target_include_directories(
        ${TARGET_NAME} SYSTEM PRIVATE "${CMAKE_SOURCE_DIR}/third_party/eigen-5.0.1")
    target_include_directories(
        ${TARGET_NAME} PRIVATE "${Python_INCLUDE_DIRS}")

    # Link required libraries.
    if(NOT MSVC)
        target_link_libraries(${TARGET_NAME} PRIVATE -pthread)
    endif()

    target_link_libraries(${TARGET_NAME} PRIVATE ${Python_LIBRARIES})

    target_link_libraries(${TARGET_NAME} PRIVATE Python::Module)
    target_link_libraries(${TARGET_NAME} PRIVATE fmt::fmt)

    # NOTE: https://cmake.org/cmake/help/latest/command/install.html#targets
    # Install directive for scikit-build-core
    install(TARGETS ${TARGET_NAME} LIBRARY DESTINATION ${INSTALL_PATH})

    # Generate typing metadata in the installed package.
    nanobind_add_stub(
        "${TARGET_NAME}_stub"
        INSTALL_TIME
        MODULE ${TARGET_NAME}
        OUTPUT "${INSTALL_PATH}/${TARGET_NAME}.pyi"
        PYTHON_PATH "${INSTALL_PATH}"
        MARKER_FILE "${INSTALL_PATH}/py.typed"
    )

endif()    # if (SKBUILD)
