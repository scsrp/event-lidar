#pragma once

#include "has_checker.hpp"

// ==================================================
//                 Detect OS
// ==================================================

// ==============================
//              OS
// ==============================

// ===== Windows

// --- 64-bit Windows target

#if !defined(EL_OS_IS_WIN64)
    #if defined(_WIN64)
        #define EL_OS_IS_WIN64 1
    #else
        #define EL_OS_IS_WIN64 0
    #endif
#endif    // ^^^ not defined(EL_OS_IS_WIN64) ^^^

// --- _WIN32 is defined for both 32-bit and 64-bit Windows targets.

#if !defined(EL_OS_IS_WIN32)
    #if defined(_WIN32)
        #define EL_OS_IS_WIN32 1
    #else
        #define EL_OS_IS_WIN32 0
    #endif
#endif    // ^^^ not defined(EL_OS_IS_WIN32) ^^^

// --- Windows target

// clang-format off
#if !defined(EL_OS_IS_WIN)
// A 64-bit Windows target must also set EL_OS_IS_WIN32.
    #if EL_OS_IS_WIN64
        static_assert(EL_OS_IS_WIN32);
    #endif

    #if EL_OS_IS_WIN64        || EL_OS_IS_WIN32          || \
        defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__)

        #define EL_OS_IS_WIN 1

        #if !defined(EL_OS_NAME)
            // NOTE: https://learn.microsoft.com/en-us/cpp/porting/modifying-winver-and-win32-winnt?view=msvc-170
            // The SDK header provides constants for minimum Windows target versions.
            #if EL_HAS_INCLUDE(<sdkddkver.h>)

                #include <sdkddkver.h>

                #if !defined(EL_OS_NAME) && (_WIN32_WINNT == _WIN32_WINNT_WIN10)
                    #define EL_OS_NAME "Windows 10"
                #endif
            
                #if !defined(EL_OS_NAME) && (_WIN32_WINNT == _WIN32_WINNT_WINBLUE)
                    #define EL_OS_NAME "Windows 8.1"
                #endif
            
                #if !defined(EL_OS_NAME) && (_WIN32_WINNT == _WIN32_WINNT_WIN8)
                    #define EL_OS_NAME "Windows 8"
                #endif
            
                #if !defined(EL_OS_NAME) && (_WIN32_WINNT == _WIN32_WINNT_WIN7)
                    #define EL_OS_NAME "Windows 7"
                #endif
            
                #if !defined(EL_OS_NAME) && ((_WIN32_WINNT == _WIN32_WINNT_LONGHORN) ||    \
                                          (_WIN32_WINNT == _WIN32_WINNT_VISTA)    ||    \
                                          (_WIN32_WINNT == _WIN32_WINNT_WIN6))
                    #define EL_OS_NAME "Windows Vista"
                #endif

                #if !defined(EL_OS_NAME) && (_WIN32_WINNT == _WIN32_WINNT_WS08)
                    #define EL_OS_NAME "Windows Server 2008"
                #endif

                #if !defined(EL_OS_NAME) && (_WIN32_WINNT == _WIN32_WINNT_WS03)
                    #define EL_OS_NAME "Windows Server 2003"
                #endif

                #if !defined(EL_OS_NAME) && (_WIN32_WINNT == _WIN32_WINNT_WINXP)
                    #define EL_OS_NAME "Windows XP"
                #endif

                #if !defined(EL_OS_NAME) && (_WIN32_WINNT == _WIN32_WINNT_WIN2K)
                    #define EL_OS_NAME "Windows 2000"
                #endif
            
                #if !defined(EL_OS_NAME) && (_WIN32_WINNT == _WIN32_WINNT_NT4)
                    #define EL_OS_NAME "Windows NT 4.0"
                #endif
            
                #if !defined(EL_OS_NAME)
                    #define EL_OS_NAME "Windows"    // Unknown
                #endif
            #else    // ^^^  EL_HAS_INCLUDE(<sdkddkver.h>) / not EL_HAS_INCLUDE(<sdkddkver.h>) vvv
                #define EL_OS_NAME "Windows"
            #endif    // ^^^ not EL_HAS_INCLUDE(<sdkddkver.h>) ^^^
        #endif    // ^^^ not defined(EL_OS_NAME) ^^^

        // EL_OS_NAME should be defined by this point.
        #if !defined(EL_OS_NAME)
            static_assert(false);
        #endif
    #else
        #define EL_OS_IS_WIN 0
    #endif
#endif    // ^^^ not defined(EL_OS_IS_WIN) ^^^

// clang-format on

// ===== Unix-like targets

#if !defined(EL_OS_IS_LINUX)
    #if defined(__linux__) || defined(__linux) || defined(linux)
        #define EL_OS_IS_LINUX 1
        #if !defined(EL_OS_NAME)
            #define EL_OS_NAME "Linux"
        #endif
    #else
        #define EL_OS_IS_LINUX 0
    #endif
#endif    // ^^^ not defined(EL_OS_IS_LINUX) ^^^

// Check generic Unix last so more specific EL_OS_NAME values take precedence.
#if !defined(EL_OS_IS_UNIX)
    #if defined(__unix__) || defined(__unix)
        #define EL_OS_IS_UNIX 1
        #if !defined(EL_OS_NAME)
            #define EL_OS_NAME "Unix family"
        #endif
    #else
        #define EL_OS_IS_UNIX 0
    #endif
#endif    // ^^^ not defined(EL_OS_IS_UNIX) ^^^

// ===== Apple targets

//! @def   EL_OS_IS_MAC
//! @brief Detects Apple Mach-based or classic Macintosh targets.
#if !defined(EL_OS_IS_MAC)
    #if defined(__APPLE__) && defined(__MACH__)
        #define EL_OS_IS_MAC 1
        #if !defined(EL_OS_NAME)
            #define EL_OS_NAME "Mac OS X"
        #endif
    #elif defined(macintosh) || defined(Macintosh)
        #define EL_OS_IS_MAC 1
        #if !defined(EL_OS_NAME)
            #define EL_OS_NAME "Mac OS 9"
        #endif
    #else
        #define EL_OS_IS_MAC 0
    #endif
#endif    // ^^^ not defined(EL_OS_IS_MAC) ^^^

// ==============================
//          Other environments
// ==============================

// ===== MinGW

// --- 64-bit MinGW target

#if !defined(EL_ENV_IS_MINGW64)
    #if defined(__MINGW64__)
        #define EL_ENV_IS_MINGW64 1
        #if !defined(EL_ENV_NAME)
            #define EL_ENV_NAME "MinGW 64"
        #endif
    #else
        #define EL_ENV_IS_MINGW64 0
    #endif
#endif    // ^^^ not defined(EL_ENV_IS_MINGW64) ^^^

          // --- __MINGW32__ is defined for both 32-bit and 64-bit MinGW targets.

#if !defined(EL_ENV_IS_MINGW32)
    #if defined(__MINGW32__)
        #define EL_ENV_IS_MINGW32 1
        #if !defined(EL_ENV_NAME)
            #define EL_ENV_NAME "MinGW 32"
        #endif
    #else
        #define EL_ENV_IS_MINGW32 0
    #endif
#endif    // ^^^ not defined(EL_ENV_IS_MINGW32) ^^^

          // --- Any MinGW target

// clang-format off
#if !defined(EL_ENV_IS_MINGW)
    #if !(defined(EL_ENV_IS_MINGW64) && defined(EL_ENV_IS_MINGW32))
        static_assert(false);
    #endif

    #if EL_ENV_IS_MINGW64 || EL_ENV_IS_MINGW32
        #define EL_ENV_IS_MINGW 1
    #else
        #define EL_ENV_IS_MINGW 0
    #endif
#endif    // ^^^ not defined(EL_ENV_IS_MINGW) ^^^
// clang-format on

// ===== Cygwin

#if !defined(EL_ENV_IS_CYGWIN)
    #if defined(__CYGWIN__)
        #define EL_ENV_IS_CYGWIN 1
        #if !defined(EL_ENV_NAME)
            #define EL_ENV_NAME "Cygwin"
        #endif
    #else
        #define EL_ENV_IS_CYGWIN 0
    #endif
#endif    // ^^^ not defined(EL_ENV_IS_CYGWIN) ^^^
