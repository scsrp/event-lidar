function(detect_prj_ver)
    cmake_parse_arguments(DETECT_PRJ_VER "VERBOSE" "" "" ${ARGN})

    # ===== find git

    find_package(Git QUIET)

    # Return if Git is unavailable.
    if (NOT GIT_FOUND)
        if (DETECT_PRJ_VER_VERBOSE)
            message(WARNING "detect_prj_ver() failed.")
        endif()

        set(PROJECT_VERSION_FOUND "0" PARENT_SCOPE)
        return()
    endif()

    # ===== git describe

    # In script mode, CMAKE_SOURCE_DIR points to the command's working directory.
    # Use SRC_DIR to run git describe from the project root, even for out-of-tree builds.
    if(DEFINED SRC_DIR AND NOT "${SRC_DIR}" STREQUAL "")  # SRC_DIR was provided.
        set(DETECT_PRJ_VER_WORKING_DIRECTORY "${SRC_DIR}")
    else()
        set(DETECT_PRJ_VER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
    endif()

    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --tags --always --dirty --long
        WORKING_DIRECTORY "${DETECT_PRJ_VER_WORKING_DIRECTORY}"
        OUTPUT_VARIABLE GIT_DESCRIBE_OUTPUT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    # Return if git describe produces no output.
    if("${GIT_DESCRIBE_OUTPUT}" STREQUAL "")
        if (DETECT_PRJ_VER_VERBOSE)
            message(WARNING "detect_prj_ver() failed.")
        endif()

        set(PROJECT_VERSION_FOUND "0" PARENT_SCOPE)
        return()
    endif()

    # ===== parse

    # --- init

    set(INTERNAL_VER_MAJOR  "0")
    set(INTERNAL_VER_MINOR  "0")
    set(INTERNAL_VER_PATCH  "0")
    set(INTERNAL_VER_SUFFIX "")
    set(INTERNAL_VER_TWEAK  "0")
    set(INTERNAL_VER_SHA    "")
    set(INTERNAL_VER_DIRTY  0)

    # --- check the dirty flag

    if(GIT_DESCRIBE_OUTPUT MATCHES "-dirty$")
        set(INTERNAL_VER_DIRTY 1)
        string(REPLACE "-dirty" "" CLEAN_DESCRIBE "${GIT_DESCRIBE_OUTPUT}")
    else()
        set(CLEAN_DESCRIBE "${GIT_DESCRIBE_OUTPUT}")
    endif()

    # --- split into tag, tweak, and SHA

    # NOTE: <tag>-<tweak>-g<SHA>
    #       <tag> is the Git tag (e.g. v0.1.5-m8 or v0.1.5).
    #       <tweak> is the number of commits since that tag.
    if(CLEAN_DESCRIBE MATCHES "^(.+)-([0-9]+)-g([0-9a-f]+)$")
        set(TAG_PART           ${CMAKE_MATCH_1})
        set(INTERNAL_VER_TWEAK ${CMAKE_MATCH_2})
        set(INTERNAL_VER_SHA   ${CMAKE_MATCH_3})
    else()
        set(TAG_PART   ${CLEAN_DESCRIBE})
    endif()

    # --- parse tag

    # NOTE: Tag format: <major>.<minor>.<patch> with an optional suffix

    if(TAG_PART MATCHES "^v?([0-9]+)\\.([0-9]+)\\.([0-9]+)(.*)$")
        set(INTERNAL_VER_MAJOR  ${CMAKE_MATCH_1})
        set(INTERNAL_VER_MINOR  ${CMAKE_MATCH_2})
        set(INTERNAL_VER_PATCH  ${CMAKE_MATCH_3})
        set(INTERNAL_VER_SUFFIX ${CMAKE_MATCH_4})
    endif()

    # ===== parent scope

    # --- component

    set(PROJECT_VERSION_FOUND 1 PARENT_SCOPE)

    set(PRJ_VER_MAJOR  ${INTERNAL_VER_MAJOR}  PARENT_SCOPE)
    set(PRJ_VER_MINOR  ${INTERNAL_VER_MINOR}  PARENT_SCOPE)
    set(PRJ_VER_PATCH  ${INTERNAL_VER_PATCH}  PARENT_SCOPE)
    set(PRJ_VER_SUFFIX ${INTERNAL_VER_SUFFIX} PARENT_SCOPE)
    set(PRJ_VER_TWEAK  ${INTERNAL_VER_TWEAK}  PARENT_SCOPE)
    set(PRJ_VER_SHA    ${INTERNAL_VER_SHA}    PARENT_SCOPE)
    set(PRJ_VER_DIRTY  ${INTERNAL_VER_DIRTY}  PARENT_SCOPE)

    # --- sem ver

    # Default to an empty string.
    set(INTERNAL_VER_DIRTY_STR "")

    # Set the suffix when the working tree is dirty.
    if(INTERNAL_VER_DIRTY)
        set(INTERNAL_VER_DIRTY_STR ".dirty")
    endif()

    set(INTERNAL_VER_CORE   "${INTERNAL_VER_MAJOR}.${INTERNAL_VER_MINOR}.${INTERNAL_VER_PATCH}")
    set(INTERNAL_VER_PUBLIC "${INTERNAL_VER_CORE}${INTERNAL_VER_SUFFIX}")
    set(INTERNAL_VER_BUILD  "${INTERNAL_VER_CORE}+${INTERNAL_VER_SHA}${INTERNAL_VER_DIRTY_STR}")
    set(INTERNAL_VER_FULL   "${INTERNAL_VER_PUBLIC}+${INTERNAL_VER_TWEAK}.${INTERNAL_VER_SHA}${INTERNAL_VER_DIRTY_STR}")

    set(PRJ_VER_CORE   ${INTERNAL_VER_CORE}   PARENT_SCOPE)
    set(PRJ_VER_PUBLIC ${INTERNAL_VER_PUBLIC} PARENT_SCOPE)
    set(PRJ_VER_BUILD  ${INTERNAL_VER_BUILD}  PARENT_SCOPE)
    set(PRJ_VER_FULL   ${INTERNAL_VER_FULL}   PARENT_SCOPE)

    # ===== print

    if (DETECT_PRJ_VER_VERBOSE)
        message(STATUS "DETECT_PRJ_VER_VERBOSE = ${DETECT_PRJ_VER_VERBOSE}")
        message(STATUS "Version:")
        message(STATUS "    Major  : ${INTERNAL_VER_MAJOR}")
        message(STATUS "    Minor  : ${INTERNAL_VER_MINOR}")
        message(STATUS "    Patch  : ${INTERNAL_VER_PATCH}")
        message(STATUS "    Suffix : ${INTERNAL_VER_SUFFIX}")
        message(STATUS "    Tweak  : ${INTERNAL_VER_TWEAK}")
        message(STATUS "    SHA    : ${INTERNAL_VER_SHA}")
        message(STATUS "    Dirty  : ${INTERNAL_VER_DIRTY}")
        message(STATUS "    Core   : ${INTERNAL_VER_CORE}")
        message(STATUS "    Public : ${INTERNAL_VER_PUBLIC}")
        message(STATUS "    Build  : ${INTERNAL_VER_BUILD}")
        message(STATUS "    Full   : ${INTERNAL_VER_FULL}")
    endif()
endfunction()
