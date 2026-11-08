#pragma once

#include <cstdlib>
#include <iostream>

#include "../config.hpp"

//! @def EL_ASSERT_IMPL
//! @brief Print an error and abort if the condition is false.
#define EL_ASSERT_IMPL(COND)                                                                                                   \
    do {                                                                                                                       \
        if (!(COND)) {                                                                                                         \
            std::cout << "error: assertion failed: " << #COND << std::endl;                                                    \
            std::abort();                                                                                                      \
        }                                                                                                                      \
    } while (false)

//! @def EL_ASSERT
//! @brief When enabled, print an error and abort if the condition is false.
#if EL_ENABLE_ASSERT
    #define EL_ASSERT(COND) EL_ASSERT_IMPL(COND)
#else
    #define EL_ASSERT(COND)
#endif
