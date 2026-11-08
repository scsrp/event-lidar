#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "utils/assert.hpp"
#include "utils/container.hpp"
#include "utils/fmt_dispatcher.hpp"
#include "utils/int_types.hpp"
#include "utils/mpl.hpp"

namespace el {

    template<typename Ty>
    using Vector3 = Eigen::Matrix<Ty, 3, 1>;

    template<typename Ty>
    using Matrix3 = Eigen::Matrix<Ty, 3, 3, Eigen::RowMajor>;

    // Row-major matrix for a 4x4 homogeneous transform.
    template<typename Ty>
    using Matrix4 = Eigen::Matrix<Ty, 4, 4, Eigen::RowMajor>;

    // ==================================================
    //                  Transform
    // ==================================================

    //! @brief Common constants for transformation matrices.
    struct TransformCommon {
        static constexpr std::array<std::size_t, 2> kHTransformShape   = {4, 4};
        static constexpr std::size_t                kHTransformSize    = kHTransformShape[0] * kHTransformShape[1];

        static constexpr std::array<std::size_t, 2> kRotationShape     = {3, 3};
        static constexpr std::size_t                kRotationSize      = kRotationShape[0] * kRotationShape[1];

        static constexpr std::size_t                kTranslationOffset = 3;
        static constexpr std::size_t                kTranslationSize   = 3;
    };

    //! @brief Computes the inverse of a rigid 4x4 homogeneous transform.
    //! @param src The source transform.
    //! @return The inverse transform.
    template<typename Ty>
    Matrix4<Ty> compute_ht_inv(const Matrix4<Ty>& src) {
        Matrix4<Ty> dst;
        // The rotation matrix is orthogonal, so R^-1 = R^T.
        dst.template block<3, 3>(0, 0)  = src.template block<3, 3>(0, 0).transpose();

        dst.template block<3, 1>(0, 3)  = dst.template block<3, 3>(0, 0) * src.template block<3, 1>(0, 3);    // t' = R^-1 * t

        dst.template block<3, 1>(0, 3) *= -1;                                                                 // t'' = -t'
        dst.template block<1, 3>(3, 0).setZero();    // Zero the first three entries of the last row.

        dst(3, 3) = 1;                               // Set the bottom-right entry to 1.
        return dst;
    }

    //! @brief Computes the rotation and translation from one frame to another.
    //!        f2f means frame to frame.
    //! @param dst_rot Destination for the rotation matrix.
    //! @param dst_transl Destination for the translation vector.
    //! @param from_f2w Homogeneous transform from the source frame to the world frame.
    //! @param to_f2w Homogeneous transform from the target frame to the world frame.
    //! @param zero_clear Whether to zero the destinations before accumulating the result.
    template<typename Ty>
    void compute_f2f(
        Matrix3<Ty>& dst_rot, Vector3<Ty>& dst_transl, const Matrix4<Ty>& from_f2w, const Matrix4<Ty>& to_f2w,
        const bool zero_clear = true) {
        const Matrix4<Ty> to_inv = compute_ht_inv(to_f2w);

        if (zero_clear) {
            dst_rot.setZero();
            dst_transl.setZero();
        }

        dst_rot    += to_inv.template block<3, 3>(0, 0) * from_f2w.template block<3, 3>(0, 0);

        dst_transl += to_inv.template block<3, 3>(0, 0) * from_f2w.template block<3, 1>(0, 3);
        dst_transl += to_inv.template block<3, 1>(0, TransformCommon::kTranslationOffset);
    }

    // ==================================================
    //                  math
    // ==================================================

    //! @brief Provides constants for pi.
    namespace numbers {
        template<typename Ty, class Enable = std::nullptr_t>
        constexpr Ty kPi_v;

        template<typename Ty>
        constexpr Ty kPi_v<Ty, std::enable_if_t<std::is_floating_point_v<Ty>, std::nullptr_t>>
            = 3.1415926535'8979323846'2643383279'5028841971;

        //! @brief Pi as a double.
        constexpr double kPi = kPi_v<double>;
    }    // namespace numbers

    //! @brief Returns the square of a value.
    template<typename Ty>
    constexpr auto pow2(const Ty v) {
        return v * v;
    }

    //! @brief Returns a nonnegative remainder.
    //! @param val The dividend.
    //! @param div The positive divisor.
    template<typename T1, typename T2, EL_ENABLE_WHEN(std::is_arithmetic_v<T1>&& std::is_arithmetic_v<T2>)>
    auto positive_fmod(const T1 val, const T2 div) noexcept(noexcept(std::fmod(std::declval<T1>(), std::declval<T2>()))) {
        const auto& ret = std::fmod(val, div);
        return (ret < 0 ? ret + div : ret);
    }

    //! @brief Converts an angle from degrees to radians.
    //! @param deg The angle in degrees.
    template<typename Ty>
    constexpr std::conditional_t<std::is_integral_v<Ty>, double, Ty> deg2rad(const Ty deg) noexcept {
        return (deg / 180.) * numbers::kPi;
    }

    template<typename Ty>
    cnt::Triplet<Ty, Ty, Ty> polar2xyz(const Ty lon, const Ty lat, const Ty r) noexcept {
        const Ty& r_cos_lat = r * std::cos(lat);
        return {r_cos_lat * std::cos(lon), -r_cos_lat * std::sin(lon), r * std::sin(lat)};
    }

    template<typename Ty>
    cnt::Triplet<Ty, Ty, Ty> polar2xyz(const Ty* const polar_3d) noexcept {
        return polar2xyz(polar_3d[0], polar_3d[1], polar_3d[2]);
    }

    //! @brief Converts Cartesian coordinates to (longitude, latitude, range).
    //! @details Longitude is in [-pi, pi], latitude in [-pi/2, pi/2], and range is nonnegative.
    //!          Longitude uses atan2(-y, x), so positive angles point toward negative y.
    template<typename Ty>
    cnt::Triplet<Ty, Ty, Ty> xyz2polar(const Ty x, const Ty y, const Ty z) noexcept {
        const Ty& r = std::sqrt(x * x + y * y + z * z);
        return {std::atan2(-y, x), std::asin(z / r), r};
    }

    template<typename Ty>
    cnt::Triplet<Ty, Ty, Ty> xyz2polar(const Ty* const xyz) noexcept {
        return xyz2polar(xyz[0], xyz[1], xyz[2]);
    }

    //! @brief Overload for Eigen vectors.
    template<typename Ty>
    Vector3<Ty> xyz2polar(const Vector3<Ty>& v3) {
        const auto& polar = xyz2polar(v3[0], v3[1], v3[2]);
        return Vector3<Ty>{polar.first, polar.second, polar.third};
    }

    template<typename Ty>
    auto norm_of(Ty x, Ty y, Ty z) {
        return std::sqrt(pow2(x) + pow2(y) + pow2(z));
    };

    template<typename Ty>
    auto norm_of(const Vector3<Ty>& v) {
        return std::sqrt(pow2(v[0]) + pow2(v[1]) + pow2(v[2]));
    };

    // ==================================================
    //                  Local Geometry Projection
    // ==================================================

    //! @brief Projects a point onto the line through two points, or onto that point if they coincide.
    //! @param curr The point to project.
    //! @param prev The first point defining the line.
    //! @param prev2 The second point defining the line.
    //! @return The projected point.
    template<typename Ty>
    auto project_onto_line(const Vector3<Ty>& curr, const Vector3<Ty>& prev, const Vector3<Ty>& prev2) {
        const Vector3<Ty> line_vec  = prev2 - prev;
        const Vector3<Ty> point_vec = curr - prev;

        Ty                line_ssq  = line_vec.squaredNorm();
        if (line_ssq == 0) {    // Avoid division by zero.
            line_ssq = 1;
        }

        const Ty          prj_norm = point_vec.dot(line_vec) / line_ssq;
        const Vector3<Ty> prj_prev = prev + (prj_norm * line_vec);

        return prj_prev;
    };

    //! @brief Projects a point onto a plane defined by three points.
    //! @param curr The point to project.
    //! @param prev The first point defining the plane.
    //! @param prev2 The second point defining the plane.
    //! @param prev3 The third point defining the plane.
    //! @return The projected point.
    template<typename Ty>
    auto project_onto_plane(
        const Vector3<Ty>& curr, const Vector3<Ty>& prev, const Vector3<Ty>& prev2, const Vector3<Ty>& prev3) {
        const Vector3<Ty> line1_vec   = prev2 - prev;
        const Vector3<Ty> line2_vec   = prev3 - prev;
        const Vector3<Ty> point_vec   = curr - prev;

        Vector3<Ty>       normal_vec  = line1_vec.cross(line2_vec);

        const Ty          normal_ssq  = normal_vec.squaredNorm();
        normal_vec                   /= std::sqrt(normal_ssq);

        const Ty          distance    = point_vec.dot(normal_vec);
        const Vector3<Ty> prj_prev    = curr - (distance * normal_vec);

        return prj_prev;
    };

    // ==================================================
    //                  Depth Map Entry
    // ==================================================

    //! @brief A simple depth map entry.
    template<typename Ty>
    struct BasicDepthMapEntry {
        static constexpr Ty kNone = std::numeric_limits<Ty>::max();

        Vector3<Ty>         xyz;    // An invalid x marks an invalid entry.

        BasicDepthMapEntry() : xyz{kNone, kNone, kNone} {}

        BasicDepthMapEntry(const BasicDepthMapEntry& rhs) : xyz{{rhs.xyz[0]}, {rhs.xyz[1]}, {rhs.xyz[2]}} {}

        void operator=(const BasicDepthMapEntry& rhs) {
            xyz[0] = rhs.xyz[0];
            xyz[1] = rhs.xyz[1];
            xyz[2] = rhs.xyz[2];
        }

        // An entry is invalid when x has the sentinel value.
        void clear() noexcept { xyz[0] = kNone; }

        bool empty() const noexcept { return xyz[0] == kNone; }

        void set_xyz(const Ty ax, const Ty ay, const Ty az) noexcept {
            xyz[0] = ax;
            xyz[1] = ay;
            xyz[2] = az;
        }

        struct IsValid {
            EL_STATIC_CALL_OP bool operator()(const BasicDepthMapEntry& entry) { return !entry.empty(); }
        };
    };

    // ==================================================
    //                  Index Access
    // ==================================================

    //! @brief Provides 2D indexing for data stored in a flat array.
    struct Indexer2D {
    public:
        using value_type = std::size_t;

    private:
        //! @brief Size of the second dimension.
        value_type shape_1_;

    public:
        explicit Indexer2D(const value_type shape_1) noexcept : shape_1_(shape_1) {}

        value_type shape1() const noexcept { return shape_1_; }

        value_type as1d(const value_type i0, const value_type i1) const noexcept { return i0 * shape_1_ + i1; }
        std::pair<value_type, value_type> as2d(const value_type idx) const noexcept { return {idx / shape_1_, idx % shape_1_}; }

        template<typename Container1D>
        decltype(auto) at(Container1D& cnt1d, const value_type i0, const value_type i1) const {
            return std::forward<Container1D>(cnt1d)[as1d(i0, i1)];
        }

        template<typename Container1D>
        auto at(Container1D&& cnt1d, const value_type i0, const value_type i1) const {
            return std::move(std::forward<Container1D>(cnt1d)[as1d(i0, i1)]);
        }
    };

    // ==================================================
    //                  Depth Map Interpolation
    // ==================================================

    // The following diagram illustrates grid-based nearest-neighbor search.
    // Each vertical column of the depth map is processed independently.
    // A-E are valid observations; x is a pixel without an observation.
    //          <- bottom                       top ->
    // m   := x x x A x x x B x x x C x x x D x x x E x x x  : Original depth map from the previous frame.
    // mi  := A A A A A A A B B B B C C C C D D D D D D D D  : Index table from InterpolateToRight (interpolate_idx).
    // m'  := x x x B x x x C x x x D x x x E x x x E x x x  : Conceptual shift of valid observations (not stored).
    // mi' := B B B B B B B C C C C D D D D E E E E E E E E  : Index table from ShiftInterpolateToLeft (interpolate_idx2).
    // dif := [B - A      ][C - B ][D - C ][E - D         ]  : Local line geometry, extrapolated at the ends.
    //

    //! @brief Shift valid indices one valid position left where possible, then fill gaps.
    struct ShiftInterpolateToLeft {
    protected:
        Indexer2D   indexer_;
        std::size_t resolution_H_;

    public:
        explicit ShiftInterpolateToLeft(const std::size_t resolution_H) noexcept
            : indexer_(resolution_H), resolution_H_(resolution_H) {}

        //! @brief Return the sentinel used for an invalid index.
        template<typename IdxTy>
        static constexpr IdxTy IdxInf() noexcept {
            return std::numeric_limits<IdxTy>::max() - 1;
        }

        //! @brief Fill gaps while shifting valid indices left; repeat indices at the edges.
        //! @param dst Destination index array.
        //! @param src Source depth map.
        //! @param start_column First column to process.
        //! @param end_column One past the last column to process.
        //! @param is_valid Predicate returning true for a nonempty entry.
        //! @details
        //! src and dst must refer to distinct arrays.
        //! src is unchanged; dst stores indices into src.
        //! m   := x x x A x x x B x x x C x x x D x x x E x x x  : Original depth map.
        //! m'  := x x x B x x x C x x x D x x x E x x x E x x x  : Conceptual shift of valid observations (not stored).
        //! mi' := B B B B B B B C C C C D D D D E E E E E E E E  : Filled index table (the output).
        //!
        //! Both arrays have shape (W, H): W columns with H entries each.
        //!
        template<
            typename IdxTy, typename Ty = BasicDepthMapEntry<float>, typename Pred = BasicDepthMapEntry<float>::IsValid,
            EL_ENABLE_WHEN(el::mpl::is_predicate_v<Pred, Ty>)>
        EL_INLINE void operator()(
            IdxTy* const dst, Ty const* const src, const std::size_t start_column, const std::size_t end_column,
            Pred is_valid = {}) {
            for (std::size_t i = start_column; i < end_column; ++i) {
                constexpr IdxTy kIdxInf = IdxInf<IdxTy>();

                IdxTy           valid_idx;
                IdxTy           first_valid_idx = kIdxInf;
                IdxTy           last_valid_idx  = kIdxInf;

                for (std::size_t j = 0; j < resolution_H_; ++j) {
                    auto&& entry = indexer_.at(src, i, j);
                    if (is_valid(entry)) {
                        last_valid_idx = j;
                        break;
                    }
                }

                for (IdxTy j = resolution_H_ - 1; j > last_valid_idx; --j) {
                    const auto& src_entry = indexer_.at(src, i, j);
                    IdxTy&      dst_entry = indexer_.at(dst, i, j);

                    if (is_valid(src_entry)) {    // Valid entry.
                        // Use the previously found valid index to shift this entry.
                        if (first_valid_idx != kIdxInf) {    // A previous valid entry exists.
                            dst_entry = valid_idx;
                        }

                        // Use this index to fill subsequent entries.
                        valid_idx = indexer_.as1d(i, j);

                        if (first_valid_idx == kIdxInf) {
                            // The rightmost valid entry has no further valid entry to shift from.
                            // Keep its own index.
                            dst_entry       = valid_idx;

                            // Save the first valid index for edge extrapolation.
                            first_valid_idx = j;
                        }
                    }
                    else if (first_valid_idx != kIdxInf) {    // Fill after finding a valid entry.
                        dst_entry = valid_idx;
                    }
                }

                // The loop above stops before the leftmost valid entry.
                EL_IF_LIKELY(last_valid_idx != kIdxInf) {
                    EL_IF_UNLIKELY(first_valid_idx == kIdxInf) {    // This column has only one valid entry.
                        valid_idx = indexer_.as1d(i, last_valid_idx);
                    }

                    static_assert(std::is_signed_v<IdxTy>);
                    for (IdxTy j = last_valid_idx; j >= 0; --j) {
                        indexer_.at(dst, i, j) = valid_idx;
                    }
                }
                else {    // All entries are invalid.
                    for (std::size_t j = 0; j < resolution_H_; ++j) {
                        indexer_.at(dst, i, j) = kIdxInf;
                    }
                }

                EL_IF_UNLIKELY(first_valid_idx == kIdxInf) {
                    first_valid_idx = last_valid_idx;
                }

                EL_IF_LIKELY(first_valid_idx != kIdxInf) {
                    valid_idx = indexer_.as1d(i, first_valid_idx);
                    for (std::size_t j = first_valid_idx; j < resolution_H_; ++j) {
                        indexer_.at(dst, i, j) = valid_idx;
                    }
                }
            }
        }
    };

    //! @brief Fill gaps to the right using valid-point indices.
    struct InterpolateToRight {
    public:

    protected:
        Indexer2D   indexer_;
        std::size_t resolution_H_;

    public:
        explicit InterpolateToRight(const std::size_t resolution_H) noexcept
            : indexer_(resolution_H), resolution_H_(resolution_H) {}

        //! @brief Return the sentinel used for an invalid index.
        template<typename IdxTy>
        static constexpr IdxTy IdxInf() noexcept {
            return std::numeric_limits<IdxTy>::max() - 1;
        }

        //! @brief Build a separate index table by filling gaps to the right.
        //!        The source depth map is unchanged; dst stores indices into src.
        //!        The rightmost valid entry uses the preceding valid index when available.
        //! @param dst Destination index array.
        //! @param src Source depth map.
        //! @param start_column First column to process.
        //! @param end_column One past the last column to process.
        //! @param is_valid Predicate returning true for a nonempty entry.
        //! @details
        //!
        //!          <- bottom                       top ->
        //! m   := x x x A x x x B x x x C x x x D x x x E x x x  : Original depth map.
        //! mi  := A A A A A A A B B B B C C C C D D D D D D D D  : Filled index table (the output).
        //!
        //! Both arrays have shape (W, H); each source entry stores 3D coordinates.
        //!
        template<
            typename IdxTy, typename Ty = BasicDepthMapEntry<float>, typename Pred = BasicDepthMapEntry<float>::IsValid,
            EL_ENABLE_WHEN(el::mpl::is_predicate_v<Pred, Ty>)>
        EL_INLINE void operator()(
            IdxTy* const dst, Ty const* const src, const std::size_t start_column, const std::size_t end_column,
            Pred is_valid = {}) {
            for (std::size_t i = start_column; i < end_column; ++i) {
                constexpr IdxTy kIdxInf = IdxInf<IdxTy>();

                IdxTy           valid_idx;
                IdxTy           first_valid_idx = kIdxInf;
                IdxTy           last_valid_idx  = kIdxInf;

                static_assert(std::is_signed_v<IdxTy>);    // An unsigned index would make the descending loop infinite.
                for (IdxTy j = resolution_H_ - 1; j >= 0; --j) {
                    auto&& entry = indexer_.at(src, i, j);
                    if (is_valid(entry)) {
                        last_valid_idx = j;
                        break;
                    }
                }

                EL_IF_LIKELY(last_valid_idx != kIdxInf) {
                    // ===== Before the rightmost valid entry

                    // With only one valid entry, this loop writes nothing.
                    for (IdxTy j = 0; j < last_valid_idx; ++j) {
                        const auto& src_entry = indexer_.at(src, i, j);
                        IdxTy&      dst_entry = indexer_.at(dst, i, j);

                        if (is_valid(src_entry)) {
                            valid_idx = indexer_.as1d(i, j);
                            dst_entry = valid_idx;

                            // Save the first valid index for edge extrapolation.
                            if (first_valid_idx == kIdxInf) {
                                first_valid_idx = j;
                            }
                        }
                        else if (first_valid_idx != kIdxInf) {    // Fill after finding a valid entry.
                            // Fill the gap.
                            dst_entry = valid_idx;
                        }
                    }

                    // ===== The rightmost valid entry and entries beyond it

                    EL_IF_UNLIKELY(first_valid_idx == kIdxInf) {    // This column has only one valid entry.
                        valid_idx       = indexer_.as1d(i, last_valid_idx);
                        first_valid_idx = last_valid_idx;
                    }

                    EL_ASSERT(first_valid_idx >= 0 && static_cast<std::size_t>(first_valid_idx) < resolution_H_);
                    EL_ASSERT(last_valid_idx >= 0 && static_cast<std::size_t>(last_valid_idx) < resolution_H_);

                    // The loop above stops before the rightmost valid entry.
                    // valid_idx holds the preceding valid index, or the only valid index if there is just one.
                    // Fill the rightmost entry and the remaining edge with valid_idx.
                    for (IdxTy j = last_valid_idx; j < static_cast<IdxTy>(resolution_H_); ++j) {
                        indexer_.at(dst, i, j) = valid_idx;
                    }

                    // ===== Entries before the leftmost valid entry

                    valid_idx = indexer_.as1d(i, first_valid_idx);
                    for (IdxTy j = 0; j < first_valid_idx; ++j) {
                        indexer_.at(dst, i, j) = valid_idx;
                    }
                }
                else {    // All entries are invalid.
                    for (std::size_t j = 0; j < resolution_H_; ++j) {
                        indexer_.at(dst, i, j) = kIdxInf;
                    }
                }
            }
        }
    };

    // ==================================================
    //                  Out
    // ==================================================

    // ===== out

    // --- runtime info

    struct RuntimeInfo {
        // LiDAR shape
        int beam_H;
        int beam_W;
        // Depth map shape (corresponds to resolution_H_ and resolution_W_).
        int dm_H;
        int dm_W;
    };

    // --- out type

    enum class OutType {
        // event flag
        kEventFlag,
        // Temporal difference features
        kDeltaScalar,
        kDeltaXyz,
        // depth map
        kPrevDepthMap,
        kPrevIpIdx,
        kPrevIpIdx2,
        // num
        kNum
    };

    inline constexpr std::array kOutTypesDiffFeature = {OutType::kDeltaScalar, OutType::kDeltaXyz};

    //! @brief Converts an output name to an OutType value.
    //! @param str The output name to convert.
    inline OutType              str_to_out_type(const std::string& str) {
        if (str == "event")
            return OutType::kEventFlag;
        if (str == "delta_scalar")
            return OutType::kDeltaScalar;
        if (str == "delta_xyz")
            return OutType::kDeltaXyz;
        if (str == "prev_dm")
            return OutType::kPrevDepthMap;
        if (str == "prev_ip_idx")
            return OutType::kPrevIpIdx;
        if (str == "prev_ip_idx2")
            return OutType::kPrevIpIdx2;
        throw std::runtime_error(el_fmt::format("unexpected str={}", str));
    }

    //! @brief Returns the size of the output feature axis.
    //! @param ot The output type.
    inline std::size_t get_out_feature_dim(const OutType ot) {
        switch (ot) {
        case OutType::kEventFlag:
        case OutType::kDeltaScalar:
        case OutType::kPrevIpIdx:
        case OutType::kPrevIpIdx2  : return 1;
        case OutType::kDeltaXyz    :
        case OutType::kPrevDepthMap: return 3;
        default                    : throw std::runtime_error("invalid OutType");
        }
    }

    //! @brief Returns the output shape (width, feature dimension, height).
    //! @param ot The output type.
    //! @param ri The frame and depth map dimensions.
    inline std::array<std::size_t, 3> get_out_shape(const OutType ot, const RuntimeInfo& ri) {
        const auto dim = get_out_feature_dim(ot);
        switch (ot) {
        case OutType::kEventFlag:
        case OutType::kDeltaScalar:
        case OutType::kDeltaXyz    : return {static_cast<std::size_t>(ri.beam_W), dim, static_cast<std::size_t>(ri.beam_H)};
        case OutType::kPrevDepthMap:
        case OutType::kPrevIpIdx   :
        case OutType::kPrevIpIdx2  : return {static_cast<std::size_t>(ri.dm_W), dim, static_cast<std::size_t>(ri.dm_H)};
        default                    : throw std::runtime_error("invalid OutType");
        }
    }

    //! @brief Returns the number of elements required for the output buffer.
    //! @param ot The output type.
    //! @param ri The frame and depth map dimensions.
    inline std::size_t get_out_element_count(const OutType ot, const RuntimeInfo& ri) {
        const auto shape = get_out_shape(ot, ri);
        return shape[0] * shape[1] * shape[2];
    }

    //! @brief Non-owning output buffers filled by the eventifier.
    //! @details Outputs are skipped when their pointers are nullptr.
    //!          Each buffer must contain at least get_out_element_count elements and remain valid until computation ends.
    //!          Buffers use row-major layout (width, feature dimension, height).
    template<typename Ty>
    struct Out {
        s32*         event_flags    = nullptr;
        Ty*          delta_scalar   = nullptr;
        Ty*          delta_xyz      = nullptr;
        Ty*          prev_depth_map = nullptr;
        std::size_t* prev_ip_idx    = nullptr;
        std::size_t* prev_ip_idx2   = nullptr;
    };

#define EL_DEFAULT_DELTA 0

    namespace event_fn {

        //! @brief Tests whether the norm of the difference vector is at least the threshold.
        struct DiffVec {
            float threshold_;

            template<typename Ty1, typename Ty2>
            void get(
                Ty1* const diff, Ty1* const thre, const Ty2 x1, const Ty2 y1, const Ty2 z1, const Ty2 x2, const Ty2 y2,
                const Ty2 z2) const {
                *diff = std::sqrt(pow2(x1 - x2) + pow2(y1 - y2) + pow2(z1 - z2));
                *thre = threshold_;
            }

            template<typename Ty1, typename Ty2>
            EL_INLINE bool operator()(
                Ty1* const diff, Ty1* const thre, const Ty2 x1, const Ty2 y1, const Ty2 z1, const Ty2 x2, const Ty2 y2,
                const Ty2 z2) const {
                get(diff, thre, x1, y1, z1, x2, y2, z2);
                return *diff >= *thre;
            }

            // NOTE: Use >= so a zero difference passes when the threshold is zero.
            template<typename Ty1>
            EL_INLINE bool operator()(const Ty1 diff, const Ty1 thre) const {
                return diff >= thre;
            }
        };

        //! @brief Tests whether the absolute difference between vector norms is at least the threshold.
        struct DiffNorm {
            float threshold_;

            template<typename Ty1, typename Ty2>
            void get(
                Ty1* const diff, Ty1* const thre, const Ty2 x1, const Ty2 y1, const Ty2 z1, const Ty2 x2, const Ty2 y2,
                const Ty2 z2) const {
                const auto ssq1 = pow2(x1) + pow2(y1) + pow2(z1);
                const auto ssq2 = pow2(x2) + pow2(y2) + pow2(z2);

                *diff           = std::abs(std::sqrt(ssq1) - std::sqrt(ssq2));
                *thre           = threshold_;
            }

            template<typename Ty1, typename Ty2>
            EL_INLINE bool operator()(
                Ty1* const diff, Ty1* const thre, const Ty2 x1, const Ty2 y1, const Ty2 z1, const Ty2 x2, const Ty2 y2,
                const Ty2 z2) const {
                get(diff, thre, x1, y1, z1, x2, y2, z2);
                return *diff >= *thre;
            }

            template<typename Ty1>
            EL_INLINE bool operator()(const Ty1 diff, const Ty1 thre) const {
                return diff >= thre;
            }
        };
    }    // namespace event_fn
}    // namespace el
