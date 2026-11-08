//! @file   config.hpp
//! @brief Configuration macros.

#include "utils/system.hpp"

// --- assert

//! @def EL_ENABLE_ASSERT
//! @brief Enable EL_ASSERT by default; define it as 0 to disable assertions.
#if !defined(EL_ENABLE_ASSERT)
    #define EL_ENABLE_ASSERT 1
#endif

// --- fmt

// clang-format off

//! @def   EL_HAS_FMT
//! @brief Checks whether fmt is available.
#if !defined(EL_HAS_FMT)
    #if EL_HAS_INCLUDE(<fmt/format.h>)
        #define EL_HAS_FMT 1
    #else
        #define EL_HAS_FMT 0
    #endif
#endif

// clang-format on
