#pragma once

#include "../config.hpp"

#if EL_HAS_FMT
    #include <fmt/format.h>
namespace el_fmt = ::fmt;

#elif EL_HAS_CXX20 && EL_HAS_CPP_LIB_FORMAT
    #include <format>
    // std::print requires C++23.
    #if EL_HAS_CXX23 && EL_HAS_INCLUDE(<print>)
        #include <print>
    #endif
namespace el_fmt = ::std;

#else
    #error "Neither fmt::format nor std::format is available."
#endif
