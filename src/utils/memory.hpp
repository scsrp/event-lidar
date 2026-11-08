#pragma once

#include <cstddef>
#include <cstdlib>
#include <limits>
#include <memory>
#include <type_traits>

#include "mpl.hpp"
#include "system.hpp"

// Cygwin uses posix_memalign and free.
#if (EL_OS_IS_WIN || EL_ENV_IS_MINGW) && !EL_ENV_IS_CYGWIN
    #include <malloc.h>
#endif

//! @namespace el::mem
//! @brief    Memory utilities.
namespace el::mem {
#define EL_MEM_NS_ ::el::mem::

    // ===== aligned

    // --- alloc / free

    //! @brief  Allocates uninitialized storage with the requested alignment.
    //! @param  alignment Alignment in bytes; must be a nonzero power of two.
    //! @param  size Number of bytes to allocate; zero returns nullptr.
    //! @return Aligned storage, or nullptr if allocation fails or the arguments are invalid.
    inline void* aligned_alloc(const std::size_t alignment, const size_t size) {
        if (size == 0 || alignment == 0 || (alignment & (alignment - 1)) != 0) {
            return nullptr;
        }
#if (EL_OS_IS_WIN || EL_ENV_IS_MINGW) && !EL_ENV_IS_CYGWIN
        return _aligned_malloc(size, alignment);
#else
        void*      ptr             = nullptr;
        // POSIX requires alignment to be a multiple of sizeof(void*).
        const auto posix_alignment = alignment < sizeof(void*) ? sizeof(void*) : alignment;
        // On failure, POSIX leaves ptr unchanged or sets it to nullptr.
        static_cast<void>(posix_memalign(&ptr, posix_alignment, size));
        return ptr;
#endif
    }

    //! @brief  Frees storage returned by aligned_alloc.
    //! @param  ptr Storage to free; nullptr is allowed.
    inline void aligned_free(void* const ptr) {
#if (EL_OS_IS_WIN || EL_ENV_IS_MINGW) && !EL_ENV_IS_CYGWIN
        _aligned_free(ptr);
#else
        std::free(ptr);
#endif
    }

    // --- deleter

    template<typename Ty>
    struct AlignedDeleter {
        void operator()(Ty* const ptr) const { EL_MEM_NS_ aligned_free(ptr); }
    };

    template<typename Ty>
    struct AlignedDeleter<Ty[]> : AlignedDeleter<Ty> {};

    // --- unique_ptr

    template<typename Ty>
    using aligned_unique_ptr = std::unique_ptr<Ty, AlignedDeleter<Ty>>;

    // --- make unique_ptr

    //! @brief  Creates an aligned_unique_ptr with default initialization.
    //! @tparam Ty For arrays, only types with an unspecified bound, such as int[], are accepted; int[5] is not.
    template<typename Ty, EL_ENABLE_WHEN(!std::is_array_v<Ty>)>
    EL_NODISCARD EL_MEM_NS_ aligned_unique_ptr<Ty> make_aligned_unique_for_overwrite(const std::size_t alignment) {
        return EL_MEM_NS_ aligned_unique_ptr<Ty>(reinterpret_cast<Ty*>(EL_MEM_NS_ aligned_alloc(alignment, sizeof(Ty))));
    }

    template<typename Ty, EL_ENABLE_WHEN(std::is_array_v<Ty>&& std::extent_v<Ty> == 0)>
    EL_NODISCARD EL_MEM_NS_ aligned_unique_ptr<Ty> make_aligned_unique_for_overwrite(
        const std::size_t alignment, const std::size_t n_elem) {
        using ElemTy = std::remove_extent_t<Ty>;
        // Do not allocate if the element count would overflow the byte-size calculation.
        if (n_elem > (std::numeric_limits<std::size_t>::max)() / sizeof(ElemTy)) {
            return {};
        }
        return EL_MEM_NS_ aligned_unique_ptr<Ty>(
            reinterpret_cast<ElemTy*>(EL_MEM_NS_ aligned_alloc(alignment, sizeof(ElemTy) * n_elem)));
    }
}    // namespace el::mem
