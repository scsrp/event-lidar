//! @file   system_impl.hpp
//! @brief  Detects the OS, compiler, C++ version, and language features.

#pragma once

#include "compiler.hpp"
#include "has_checker.hpp"
#include "os.hpp"

#if EL_HAS_INCLUDE(<version>)
    #include <version>    // Provides standard library feature-test macros.
#endif                    // ^^^ EL_HAS_INCLUDE(<version>) ^^^

// ==================================================
//                    FTM
// ==================================================

// ===== __cpp_*

// NOTE: https://github.com/llvm/llvm-project/issues/77262
//       Clang can define __cpp_static_call_operator before C++23 because it supports
//       static operator() as an extension in earlier language modes.
#if !defined(EL_HAS_CPP_STATIC_CALL_OP)
    #if defined(__cpp_static_call_operator)
        #if __cpp_static_call_operator
            #define EL_HAS_CPP_STATIC_CALL_OP 1
        #else
            #define EL_HAS_CPP_STATIC_CALL_OP 0
        #endif
    #else    // ^^^ defined(__cpp_static_call_operator) / not defined(__cpp_static_call_operator) vvv
        #define EL_HAS_CPP_STATIC_CALL_OP 0
    #endif
#endif       // ^^^ not defined(EL_HAS_CPP_STATIC_CALL_OP) ^^^

// ===== __cpp_lib_*

#if !defined(EL_HAS_CPP_LIB_FORMAT)
    #if defined(__cpp_lib_format)
        //! @def   EL_HAS_CPP_LIB_FORMAT
        //! @brief __cpp_lib_format
        #if __cpp_lib_format
            #define EL_HAS_CPP_LIB_FORMAT 1
        #else
            #define EL_HAS_CPP_LIB_FORMAT 0
        #endif
    #else    // ^^^ defined(__cpp_lib_format) / not defined(__cpp_lib_format) vvv
        #define EL_HAS_CPP_LIB_FORMAT 0
    #endif
#endif       // ^^^ not defined(EL_HAS_CPP_LIB_FORMAT) ^^^

// ==================================================
//                 C++ Version
// ==================================================

// ===== C++ version number

// NOTE: MSVC reports 199711L for __cplusplus by default; _MSVC_LANG tracks the selected standard.
//! @def   EL_CXX_VER
//! @brief The selected C++ language version.
#if !defined(EL_CXX_VER)
    #if EL_CMP_IS_MSVC
        #define EL_CXX_VER _MSVC_LANG
    #else
        #define EL_CXX_VER __cplusplus
    #endif
#endif

#define EL_CXX98_VER 199711L
#define EL_CXX11_VER 201103L
#define EL_CXX14_VER 201402L
#define EL_CXX17_VER 201703L
#define EL_CXX20_VER 202002L
#define EL_CXX23_VER 202302L

// ===== EL_HAS_CXX**

#if !defined(EL_HAS_CXX11)
    #if EL_CXX_VER >= EL_CXX11_VER
        #define EL_HAS_CXX11 1
    #else
        #define EL_HAS_CXX11 0
    #endif
#endif    // EL_HAS_CXX11

#if !defined(EL_HAS_CXX14)
    #if EL_HAS_CXX11 && EL_CXX_VER > EL_CXX11_VER
        #define EL_HAS_CXX14 1
    #else
        #define EL_HAS_CXX14 0
    #endif
#endif    // EL_HAS_CXX14

#if !defined(EL_HAS_CXX17)
    #if EL_HAS_CXX14 && EL_CXX_VER > EL_CXX14_VER
        #define EL_HAS_CXX17 1
    #else
        #define EL_HAS_CXX17 0
    #endif
#endif    // EL_HAS_CXX17

#if !defined(EL_HAS_CXX20)
    #if EL_HAS_CXX17 && EL_CXX_VER > EL_CXX17_VER
        #define EL_HAS_CXX20 1
    #else
        #define EL_HAS_CXX20 0
    #endif
#endif    // EL_HAS_CXX20

#if !defined(EL_HAS_CXX23)
    #if EL_HAS_CXX20 && EL_CXX_VER > EL_CXX20_VER
        #define EL_HAS_CXX23 1
    #else
        #define EL_HAS_CXX23 0
    #endif
#endif    // EL_HAS_CXX23

#if !defined(EL_HAS_CXX26)
    #if EL_HAS_CXX23 && EL_CXX_VER > EL_CXX23_VER
        #define EL_HAS_CXX26 1
    #else
        #define EL_HAS_CXX26 0
    #endif
#endif    // EL_HAS_CXX26

// ===== IS_CXX**

// --- C++ >= 26

// NOTE: The highest recognized version can only be identified as C++26 or later.
#if !defined(EL_CXX_NAME) && EL_HAS_CXX26
    #define EL_CXX_NAME "C++ >= 26"

    #if !defined(EL_IS_GE_CXX26)
        #define EL_IS_GE_CXX26
    #endif
#endif

// --- C++23

#if !defined(EL_CXX_NAME) && EL_HAS_CXX23
    #define EL_CXX_NAME "C++23"

    #if !defined(EL_IS_CXX23)
        #define EL_IS_CXX23
    #endif
#endif

// --- C++20

#if !defined(EL_CXX_NAME) && EL_HAS_CXX20
    #define EL_CXX_NAME "C++20"

    #if !defined(EL_IS_CXX20)
        #define EL_IS_CXX20
    #endif
#endif

// --- C++17

#if !defined(EL_CXX_NAME) && EL_HAS_CXX17
    #define EL_CXX_NAME "C++17"

    #if !defined(EL_IS_CXX17)
        #define EL_IS_CXX17
    #endif
#endif

// --- C++14

#if !defined(EL_CXX_NAME) && EL_HAS_CXX14
    #define EL_CXX_NAME "C++14"

    #if !defined(EL_IS_CXX14)
        #define EL_IS_CXX14
    #endif
#endif

// --- C++11

#if !defined(EL_CXX_NAME) && EL_HAS_CXX11
    #define EL_CXX_NAME "C++11"

    #if !defined(EL_IS_CXX11)
        #define EL_IS_CXX11
    #endif
#endif

// --- C++ < 11

// NOTE: Earlier versions can only be identified as older than C++11.
#if !defined(EL_CXX_NAME)
    // Both version macros are defined above.
    #if !defined(EL_CXX_VER) || !defined(EL_CXX11_VER)
static_assert(false);
    #endif

    // This branch is only reached for versions older than C++11.
    #if (EL_CXX_VER >= EL_CXX11_VER)
static_assert(false);
    #endif

    #define EL_CXX_NAME "C++ < 11"

    #if !defined(EL_IS_LT_CXX11)
        #define EL_IS_LT_CXX11
    #endif
#endif

// ==================================================
//               Compiler Extensions
// ==================================================

// ===== nodiscard

// NOTE: Place the attribute before inline in declarations.
#if EL_HAS_CPP_ATTR(nodiscard)
    #define EL_NODISCARD [[nodiscard]]
#elif EL_CMP_IS_MSVC && _MSC_VER >= 1700
    #define EL_NODISCARD _Check_return_
    #define EL_NODISCARD __attribute__((warn_unused_result))
#else
    #define EL_NODISCARD
#endif

// ===== likely / unlikely

#if EL_HAS_CPP_ATTR(likely)
    #define EL_IF_LIKELY(expr)   if (expr) [[likely]]
    #define EL_IF_UNLIKELY(expr) if (expr) [[unlikely]]
#elif EL_HAS_BLTN(__builtin_expect)
    #define EL_IF_LIKELY(expr)   if (__builtin_expect(!!(expr), 1))
    #define EL_IF_UNLIKELY(expr) if (__builtin_expect(!!(expr), 0))
#else
    #define EL_IF_LIKELY(expr)   if (expr)
    #define EL_IF_UNLIKELY(expr) if (expr)
#endif

// ===== inline

#if EL_CMP_IS_MSVC
    #if EL_OS_IS_WIN

        #ifndef NOMINMAX
            #define NOMINMAX
        #endif
        #include <windows.h>
        #undef NOMINMAX

        #define EL_INLINE FORCEINLINE
    #else
        #define EL_INLINE __forceinline
    #endif

#elif EL_HAS_ATTR(always_inline)
    #define EL_INLINE __attribute__((always_inline)) inline
#else
    #define EL_INLINE inline
#endif

// ==================================================
//                    MISC
// ==================================================

// ==============================
//            static
// ==============================

//! @def   EL_STATIC_CALL_OP
//! @brief Makes operator() static when supported, removing its implicit this parameter.
#if !defined(EL_STATIC_CALL_OP)
    #if EL_HAS_CPP_STATIC_CALL_OP
        #define EL_STATIC_CALL_OP static
    #else
        #define EL_STATIC_CALL_OP
    #endif
#endif    // ^^^ not defined(EL_STATIC_CALL_OP) ^^^

