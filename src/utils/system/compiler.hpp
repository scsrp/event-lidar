//! @file   compiler.hpp
//! @brief  Compiler detection

#pragma once

#include "version_utils.hpp"

// ==================================================
//                 Detect Compiler
// ==================================================

// clang-format off
#if !defined(EL_CMP_IS_INTEL)

    #if defined(__INTEL_COMPILER) || defined(__ICL) || defined(__ICC) || defined(__INTEL_LLVM_COMPILER)
        #define EL_CMP_IS_INTEL 1

        #if !defined(EL_CMP_NAME)
            #define EL_CMP_NAME "Intel C++ Compiler"
        #endif

        // HACK: Intel versioning is complex; use the raw version instead of major/minor/patch.
        #if !defined(EL_CMP_VER)
            #if defined(__INTEL_LLVM_COMPILER)
                #define EL_CMP_VER __INTEL_LLVM_COMPILER
            #elif defined(__INTEL_COMPILER)
                #define EL_CMP_VER __INTEL_COMPILER
            #elif defined(__ICL)
                #define EL_CMP_VER __ICL
            #elif defined(__ICC)
                #define EL_CMP_VER __ICC
            #endif
        #endif
    #else
        #define EL_CMP_IS_INTEL 0
    #endif
#endif    // ^^^ not defined(EL_CMP_IS_INTEL) ^^^
// clang-format on

#if !defined(EL_CMP_IS_ARM)
    #if defined(__CC_ARM) || defined(__ARMCC_VERSION)
        #define EL_CMP_IS_ARM 1
        #if !defined(EL_CMP_NAME)
            #define EL_CMP_NAME "Arm Compiler"
        #endif
    #else
        #define EL_CMP_IS_ARM 0
    #endif
#endif    // ^^^ not defined(EL_CMP_IS_ARM) ^^^

#if !defined(EL_CMP_IS_CLANG)
    #if defined(__clang__)
        #define EL_CMP_IS_CLANG 1
        #if defined(__apple_build_version__) && !defined(EL_CMP_NAME)
            #define EL_CMP_IS_APPLE_CLANG 1
            #define EL_CMP_IS_CLANG_CL    0
            #define EL_CMP_NAME           "Apple clang"
        #elif defined(_MSC_VER) && !defined(EL_CMP_NAME)
            // _MSC_VER is also defined by Clang targeting MSVC without clang-cl.
            #define EL_CMP_IS_APPLE_CLANG 0
            #define EL_CMP_IS_CLANG_CL    1
            #define EL_CMP_NAME           "Clang-cl"
        #else
            #define EL_CMP_IS_APPLE_CLANG 0
            #define EL_CMP_IS_CLANG_CL    0
            // Intel LLVM also reaches this branch, so preserve its compiler name.
            #if !defined(EL_CMP_NAME)
                #define EL_CMP_NAME "Clang"
            #endif
        #endif

        // ver
        #if !defined(EL_CMP_VER)
            #define EL_CMP_VER EL_MAKE_VER(__clang_major__, __clang_minor__, __clang_patchlevel__)
        #endif
    #else
        #define EL_CMP_IS_APPLE_CLANG 0
        #define EL_CMP_IS_CLANG_CL    0
        #define EL_CMP_IS_CLANG       0
    #endif
#endif    // ^^^ not defined(EL_CMP_IS_CLANG) ^^^

// NOTE: https://gcc.gnu.org/onlinedocs/cpp/Common-Predefined-Macros.html
//       __GNUG__ = (__GNUC__ && __cplusplus)
#if !defined(EL_CMP_IS_GCC)
    #if defined(__GNUC__) && !EL_CMP_IS_CLANG && !EL_CMP_IS_INTEL
        #define EL_CMP_IS_GCC 1
        #if !defined(EL_CMP_NAME)
            #define EL_CMP_NAME "GCC"
        #endif

        #if !defined(EL_CMP_VER)
            #if defined(__GNUC_PATCHLEVEL__)
                #define EL_CMP_VER EL_MAKE_VER(__GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__)
            #else
                #define EL_CMP_VER EL_MAKE_VER(__GNUC__, __GNUC_MINOR__, 0)
            #endif
        #endif
    #else
        #define EL_CMP_IS_GCC 0
    #endif
#endif    // ^^^ not defined(EL_CMP_IS_GCC) ^^^

// Are GCC extensions available?
#if !defined(EL_CMP_HAS_GCC)
    #if defined(__GNUC__)
        #define EL_CMP_HAS_GCC 1
    #else
        #define EL_CMP_HAS_GCC 0
    #endif
#endif    // ^^^ not defined(EL_CMP_HAS_GCC) ^^^

// NOTE: https://learn.microsoft.com/ja-jp/cpp/overview/compiler-versions?view=msvc-170
#if !defined(EL_CMP_IS_MSVC)
// NOTE: ICC and Clang may define _MSC_VER when targeting MSVC, so exclude them.
    #if defined(_MSC_VER) && !EL_CMP_IS_INTEL && !EL_CMP_IS_CLANG
        #define EL_CMP_IS_MSVC 1
        #if !defined(EL_CMP_NAME)
            #define EL_CMP_NAME "MSVC"
        #endif

        // HACK: MSVC versioning is complex; use the raw version instead of major/minor/patch.
        #if !defined(EL_CMP_VER)
            #define EL_CMP_VER _MSC_VER
        #endif

    #else
        #define EL_CMP_IS_MSVC 0
    #endif
#endif    // ^^^ not defined(EL_CMP_IS_MSVC) ^^^
