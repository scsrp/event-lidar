//! @file   version_utils.hpp
//! @brief  Macros for encoding and decoding version numbers.

#pragma once

// ==================================================
//                 Version Utilities
// ==================================================

#define EL_MAJOR_START 24
#define EL_MINOR_START 16
#define EL_PATCH_START 0

#define EL_MAJOR_MASK ((1 << (32 - EL_MAJOR_START)) - 1)
#define EL_MINOR_MASK ((1 << EL_MAJOR_START - EL_MINOR_START) - 1)
#define EL_PATCH_MASK ((1 << EL_MINOR_START - 0) - 1)

#define EL_MAKE_VER(major, minor, patch)                                                                                       \
    (((major) << EL_MAJOR_START) | ((minor) << EL_MINOR_START) | ((patch) << EL_PATCH_START))
#define EL_INT_TO_MAJOR(int_ver) (((int_ver) >> EL_MAJOR_START) & EL_MAJOR_MASK)
#define EL_INT_TO_MINOR(int_ver) (((int_ver) >> EL_MINOR_START) & EL_MINOR_MASK)
#define EL_INT_TO_PATCH(int_ver) (((int_ver) >> EL_PATCH_START) & EL_PATCH_MASK)
