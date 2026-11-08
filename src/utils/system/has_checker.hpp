//! @file   has_checker.hpp
//! @brief  Wrappers for __has_* checks that return 0 when unavailable.

#pragma once

// ==================================================
//   Compiler extensions (__has_attribute, __has_builtin)
// ==================================================

// ===== __has_attribute

//! @def   EL_HAS_ATTR
//! @brief Checks whether a GNU-style attribute is supported.
#if defined(__has_attribute)
    #if !defined(EL_HAS_ATTR)
        #define EL_HAS_ATTR(attr_name) __has_attribute(attr_name)
    #endif    // ^^^ not defined(EL_HAS_ATTR) ^^^

#else         // ^^^ defined(__has_attribute) / not defined(__has_attribute) vvv
    #if !defined(EL_HAS_ATTR)
        #define EL_HAS_ATTR(attr_name) 0
    #endif    // ^^^ not defined(EL_HAS_ATTR) ^^^

#endif        // ^^^  not defined(__has_attribute) ^^^

// ===== __has_builtin

//! @def   EL_HAS_BLTN
//! @brief Checks whether a compiler builtin is available.
#if defined(__has_builtin)
    #if !defined(EL_HAS_BLTN)
        #define EL_HAS_BLTN(bltn_name) __has_builtin(bltn_name)
    #endif    // ^^^ not defined(EL_HAS_BLTN) ^^^

#else         // ^^^ defined(__has_builtin) / not defined(__has_builtin) vvv
    #if !defined(EL_HAS_BLTN)
        #define EL_HAS_BLTN(bltn_name) 0
    #endif    // ^^^ not defined(EL_HAS_BLTN) ^^^

#endif        // ^^^ not defined(__has_builtin) ^^^

// ==================================================
//     Standardized feature checks (__has_cpp_attribute, __has_include)
// ==================================================

// ===== __has_cpp_attribute

//! @def   EL_HAS_CPP_ATTR
//! @brief Checks whether a C++ attribute is available.
//!        C++ attribute syntax was introduced in C++11.
#if defined(__has_cpp_attribute)
    #if !defined(EL_HAS_CPP_ATTR)
        #define EL_HAS_CPP_ATTR(attr_name) __has_cpp_attribute(attr_name)
    #endif    // ^^^ not defined(EL_HAS_CPP_ATTR) ^^^

#else         // ^^^ defined(__has_cpp_attribute) / not defined(__has_cpp_attribute) vvv
    #if !defined(EL_HAS_CPP_ATTR)
        #define EL_HAS_CPP_ATTR(attr_name) 0
    #endif    // ^^^ not defined(EL_HAS_CPP_ATTR) ^^^

#endif        // ^^^ not defined(__has_cpp_attribute) ^^^

// ===== __has_include

//! @def   EL_HAS_INCLUDE
//! @brief Checks whether a header can be found for inclusion.
//!        Standardized in C++17.
#if defined(__has_include)
    #if !defined(EL_HAS_INCLUDE)
        #define EL_HAS_INCLUDE(name) __has_include(name)
    #endif    // ^^^ not defined(EL_HAS_INCLUDE) ^^^

#else         // ^^^ defined(__has_include) / not defined(__has_include) vvv
    #if !defined(EL_HAS_INCLUDE)
        #define EL_HAS_INCLUDE(name) 0
    #endif    // ^^^ not defined(EL_HAS_INCLUDE) ^^^

#endif        // ^^^ not defined(__has_include) ^^^
