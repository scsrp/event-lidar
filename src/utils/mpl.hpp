#pragma once

#include <type_traits>

//! @def EL_ENABLE_WHEN
//! @brief Enable a template only when the condition is true.
#define EL_ENABLE_WHEN(...) typename ::std::enable_if_t<(__VA_ARGS__), ::std::nullptr_t> = nullptr

//! @brief Small metafunctions used by event-lidar.
namespace el::mpl {

    //! @brief Wrap a type to prevent template argument deduction.
    template<typename Ty>
    struct type_identity {
        using type = Ty;
    };

    //! @brief Return the type unchanged and prevent template argument deduction.
    template<typename Ty>
    using type_identity_t = typename type_identity<Ty>::type;

    //! @brief Check whether Fn accepts Args and returns a value convertible to bool.
    template<typename Fn, typename... Args>
    inline constexpr bool is_predicate_v = std::is_invocable_r_v<bool, Fn, Args...>;

}    // namespace el::mpl
