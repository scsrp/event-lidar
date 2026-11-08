#pragma once

#include <algorithm>
#include <array>
#include <iostream>
#include <memory>
#include <optional>
#include <unordered_set>

#include "eventifier/eventifier_common.hpp"
#include "utils/assert.hpp"
#include "utils/int_types.hpp"
#include "utils/memory.hpp"
#include "utils/mpl.hpp"

namespace el {

    struct ComputeEventsConfig {
        bool                  return_all    = false;

        double                delta_no_prev = EL_DEFAULT_DELTA;
        double                delta_init    = EL_DEFAULT_DELTA;

        ComputeEventsConfig set_return_all(const bool val) && {
            return_all = val;
            return std::move(*this);
        }

        ComputeEventsConfig set_delta_no_prev(const double val) && {
            delta_no_prev = val;
            return std::move(*this);
        }
        ComputeEventsConfig set_delta_init(const double val) && {
            delta_init = val;
            return std::move(*this);
        }
    };

    template<typename Ty, int kTmDims>
    class BasicTemporalEventifier {
        static_assert(kTmDims == 1 || kTmDims == 2);

    public:
        // The input has three dimensions: (lon, lat, range).
        static constexpr std::size_t kInputDims    = 3;
        static constexpr std::size_t kLonIndex     = 0;
        static constexpr std::size_t kLatIndex     = 1;
        static constexpr std::size_t kRangeIndex   = 2;

        // The depth map has shape (W, H).
        // dm_index2_type stores an index along either axis.
        // dm_index1_type stores a flattened index in an interpolation table.
        using dm_index1_type                       = s32;
        using dm_index2_type                       = s16;

        // Leave a margin of one to avoid overflow and underflow.
        static constexpr dm_index1_type kIdx1Inf   = std::numeric_limits<dm_index1_type>::max() - 1;
        static constexpr dm_index1_type kIdx1NInf  = std::numeric_limits<dm_index1_type>::min() + 1;

        static constexpr dm_index2_type kIdx2Inf   = std::numeric_limits<dm_index2_type>::max() - 1;
        static constexpr dm_index2_type kIdx2NInf  = std::numeric_limits<dm_index2_type>::min() + 1;

        static constexpr std::size_t    kAlignment = 32;

        template<typename Ty2>
        using ptr_type   = mem::aligned_unique_ptr<Ty2>;

        using entry_type = BasicDepthMapEntry<Ty>;

    private:
        // Range image height; common values are 32, 64, and 128.
        // For example, Waymo uses 64.
        const int                     beam_H_;
        // Range image width; for example, Waymo uses 2650.
        const int                     beam_W_;

        // Padding on both the top and bottom increases the height by 2 * n_padding_.
        const int                     n_padding_;

        // Depth map height and width, including padding (height only; no width padding).
        const int                     resolution_H_;
        const int                     resolution_W_;

        const Ty                      vfov_min_rad_;
        const Ty                      vfov_max_rad_;
        const Ty                      vfov_rad_;

        // Vertical angle per depth map pixel.
        const Ty                      vfov_step_rad_;
        // Horizontal angle per depth map pixel.
        // Equivalent to 2 * pi / beam_W.
        const Ty                      hfov_step_rad_;

        // Lower VFOV bound of the padded depth map [rad].
        const Ty                      ext_vfov_min_rad_;
        Indexer2D                     indexer_;

        Matrix3<Ty>                   curr2prev_rot_;
        Vector3<Ty>                   curr2prev_transl_;

        // Pose of the latest target, retained for the next reference update.
        std::optional<Matrix4<Ty>>    prev_s2w_opt_;
        // Pose corresponding to prev_depth_map_.
        std::optional<Matrix4<Ty>>    reference_s2w_opt_;

        // Both are flattened depth maps with shape (resolution_W_, resolution_H_).
        std::unique_ptr<entry_type[]> prev_depth_map_;
        std::unique_ptr<entry_type[]> current_depth_map_;

        // Buffers for the interpolation results.
        // Interpolation computes indices without copying the entries themselves.
        ptr_type<dm_index1_type[]>    interpolate_idx_;
        ptr_type<dm_index1_type[]>    interpolate_idx2_;

        // Functors that compute interpolated depth map index tables.
        ShiftInterpolateToLeft        shift_interpolate_to_left_;
        InterpolateToRight            interpolate_to_right_;

    public:
        BasicTemporalEventifier(
            const int beam_H, const int beam_W, const el::mpl::type_identity_t<Ty> vfov_min_deg,
            const el::mpl::type_identity_t<Ty> vfov_max_deg, const int n_padding = 2)
            : beam_H_(beam_H), beam_W_(beam_W), n_padding_(n_padding), resolution_H_(beam_H + 2 * n_padding),
              resolution_W_(beam_W), vfov_min_rad_(deg2rad(vfov_min_deg)), vfov_max_rad_(deg2rad(vfov_max_deg)),
              vfov_rad_(vfov_max_rad_ - vfov_min_rad_), vfov_step_rad_(vfov_rad_ / beam_H),

              hfov_step_rad_(2 * numbers::kPi / resolution_W_), ext_vfov_min_rad_(vfov_min_rad_ - n_padding * vfov_step_rad_),
              indexer_(resolution_H_), shift_interpolate_to_left_(resolution_H_), interpolate_to_right_(resolution_H_)

        {
            reset();

            EL_IF_UNLIKELY(vfov_max_deg - vfov_min_deg <= 1) {
                std::cout
                    << "warn: vfov must be specified in degrees. " "vfov = (vfov_max - vfov_min) <= 1 seems too small for the units to be degrees."
                    << std::endl;
            }

            EL_ASSERT((resolution_H_ - 1 <= std::numeric_limits<dm_index2_type>::max()) && "resolution_H is too large");
            EL_ASSERT((resolution_W_ - 1 <= std::numeric_limits<dm_index2_type>::max()) && "resolution_W is too large");
            EL_ASSERT((resolution_H_ * resolution_W_ <= kIdx1Inf) && "depth map is too large");
        }

        //! @brief Reset the internal history to its initial state.
        void reset() {
            // std::make_unique_for_overwrite requires C++20.
            prev_depth_map_.reset(new entry_type[resolution_W_ * resolution_H_]);
            current_depth_map_.reset(new entry_type[resolution_W_ * resolution_H_]);

            interpolate_idx_
                = mem::make_aligned_unique_for_overwrite<dm_index1_type[]>(kAlignment, resolution_W_ * resolution_H_);
            interpolate_idx2_
                = mem::make_aligned_unique_for_overwrite<dm_index1_type[]>(kAlignment, resolution_W_ * resolution_H_);

            std::fill_n(interpolate_idx_.get(), resolution_W_ * resolution_H_, kIdx1Inf);
            std::fill_n(interpolate_idx2_.get(), resolution_W_ * resolution_H_, kIdx1Inf);

            curr2prev_rot_.setIdentity();
            curr2prev_transl_.setZero();

            prev_s2w_opt_.reset();
            reference_s2w_opt_.reset();

            static_assert(std::is_signed_v<dm_index1_type>);
            static_assert(std::is_signed_v<dm_index2_type>);
        }

        int         beam_H() const noexcept { return beam_H_; }
        int         beam_W() const noexcept { return beam_W_; }

        int         resolution_H() const noexcept { return resolution_H_; }
        int         resolution_W() const noexcept { return resolution_W_; }

        //! @brief Return the runtime shapes of the input range image and depth map.
        RuntimeInfo get_runtime_info() const noexcept {
            RuntimeInfo ri;
            ri.beam_H = beam_H_;
            ri.beam_W = beam_W_;
            ri.dm_H   = resolution_H_;
            ri.dm_W   = resolution_W_;
            return ri;
        }

        Ty vfov_step_rad() const noexcept { return vfov_step_rad_; }
        Ty hfov_step_rad() const noexcept { return hfov_step_rad_; }

        // clang-format off

        Matrix3<Ty> const& curr2prev_rotation() const& noexcept { return curr2prev_rot_; }
        Vector3<Ty> const& curr2prev_translation() const& noexcept { return curr2prev_transl_; }
        std::optional<Matrix4<Ty>> const& prev_s2w_ht() const& noexcept { return prev_s2w_opt_; }

        // clang-format on

        //! @brief Use the last processed target frame as the reference frame.
        //! @details Call after compute_events_from_polar() to retain the target depth map and rebuild the index tables.
        //!          Before the first target pose is set, the reference remains unset.
        void update_reference_frame() {
            reference_s2w_opt_              = prev_s2w_opt_;

            entry_type const* const curr_dm = current_depth_map_.get();
            entry_type* const       prev_dm = prev_depth_map_.get();

            for (int i = 0; i < resolution_W_ * resolution_H_; ++i) {
                const entry_type& entry = curr_dm[i];
                if (!entry.empty()) {
                    prev_dm[i] = entry;
                }
                else {
                    prev_dm[i].clear();
                }
            }

            interpolate_to_right_(interpolate_idx_.get(), prev_dm, 0, resolution_W_, typename entry_type::IsValid{});
            shift_interpolate_to_left_(interpolate_idx2_.get(), prev_dm, 0, resolution_W_, typename entry_type::IsValid{});
        }

        //! @brief Prepare the eventifier to process a target frame while retaining the reference frame.
        //! @param[in] curr_s2w_ht 4x4 homogeneous transform from the target sensor frame to the world frame.
        //! @details Updates the target-to-reference transform and clears the target depth map for compute_events_from_polar().
        template<typename Ty2, EL_ENABLE_WHEN(std::is_floating_point_v<Ty2>)>
        void prepare_target_state(const Ty2* curr_s2w_ht) {
            const Matrix4<Ty> curr_s2w_ht_matrix = Eigen::Map<const Matrix4<Ty2>>(curr_s2w_ht).template cast<Ty>();

            if (reference_s2w_opt_) {
                compute_f2f(curr2prev_rot_, curr2prev_transl_, curr_s2w_ht_matrix, *reference_s2w_opt_);
            }
            prev_s2w_opt_ = curr_s2w_ht_matrix;

            for (int i = 0; i < resolution_W_ * resolution_H_; ++i) {
                current_depth_map_[i].clear();
            }
        }

        //! @brief Prepare the eventifier to process the next target frame, optionally updating the reference frame.
        //! @param[in] curr_s2w_ht 4x4 homogeneous transform from the target sensor frame to the world frame.
        //! @param[in] update_reference Use the last processed target as the reference before preparing the eventifier.
        template<typename Ty2, EL_ENABLE_WHEN(std::is_floating_point_v<Ty2>)>
        void prepare_for_next_frame(const Ty2* curr_s2w_ht, const bool update_reference = true) {
            if (update_reference) {
                update_reference_frame();
            }
            prepare_target_state(curr_s2w_ht);
        }

        //! @return Depth map indices for polar coordinates (lon, lat).
        //!         The depth map has shape (resolution_W_, resolution_H_).
        template<typename Ty2>
        std::pair<dm_index2_type, dm_index2_type> dm_index_from_polar(const Ty2 lon, const Ty2 lat) const {
            return {
                static_cast<dm_index2_type>(
                    positive_fmod(static_cast<dm_index2_type>((lon + numbers::kPi) / hfov_step_rad_), resolution_W_)),
                std::clamp<dm_index2_type>(
                    static_cast<dm_index2_type>((lat - ext_vfov_min_rad_) / vfov_step_rad_), 0, resolution_H_ - 1)};
        }

        std::optional<entry_type> try_get_entry(entry_type const* const ref_dm, const dm_index1_type idx) {
            if (idx == kIdx1Inf) {
                return std::nullopt;
            }
            EL_ASSERT(idx < (resolution_H_ * resolution_W_));

            const entry_type& entry = ref_dm[idx];
            if (entry.empty()) {
                return std::nullopt;
            }
            return entry;
        }

        template<int kDims>
        auto project_onto_local_geometry(
            const Vector3<Ty> curr_xyz, entry_type const* const ref_dm,
            const std::array<dm_index1_type, kDims + 1>& local_geo_indices) -> std::optional<Vector3<Ty>> {
            static_assert(kDims == 1 || kDims == 2);

            if constexpr (kDims == 1) {
                const dm_index1_type ip_idx  = local_geo_indices[0];
                const dm_index1_type ip_idx2 = local_geo_indices[1];

                // Line approximation requires two indices.
                // If only one point exists, the interpolation functors provide its index twice.
                if ((ip_idx == kIdx1Inf) || (ip_idx2 == kIdx1Inf)) {
                    return std::nullopt;
                }

                const std::optional<entry_type> entry_opt  = try_get_entry(ref_dm, ip_idx);
                const std::optional<entry_type> entry2_opt = try_get_entry(ref_dm, ip_idx2);
                if (!entry_opt || !entry2_opt) {
                    return std::nullopt;
                }

                // With only one valid point, passing it twice makes project_onto_line return that point.
                return project_onto_line(curr_xyz, entry_opt->xyz, entry2_opt->xyz);
            }
            else {
                const auto ip_idx  = local_geo_indices[0];
                const auto ip_idx2 = local_geo_indices[1];
                const auto ip_idx3 = local_geo_indices[2];

                // Without both indices for line approximation, report no local geometry; the caller treats the point as an event.
                if ((ip_idx == kIdx1Inf) || (ip_idx2 == kIdx1Inf)) {
                    return std::nullopt;
                }

                const std::optional<entry_type> entry_opt  = try_get_entry(ref_dm, ip_idx);
                const std::optional<entry_type> entry2_opt = try_get_entry(ref_dm, ip_idx2);
                const std::optional<entry_type> entry3_opt = try_get_entry(ref_dm, ip_idx3);

                // clang-format off
                std::unordered_set<dm_index1_type> idx_set;
                EL_IF_LIKELY(entry_opt)
                    idx_set.emplace(ip_idx);
                EL_IF_LIKELY(entry2_opt)
                    idx_set.emplace(ip_idx2);
                EL_IF_LIKELY(entry3_opt)
                    idx_set.emplace(ip_idx3);
                const int n_unique_prev = idx_set.size();
                // clang-format on

                // Use plane approximation when all three points are distinct.
                EL_IF_LIKELY(n_unique_prev == 3) {
                    return project_onto_plane(curr_xyz, entry_opt->xyz, entry2_opt->xyz, entry3_opt->xyz);
                }

                // Fall back to line approximation.
                return project_onto_line(curr_xyz, entry_opt->xyz, entry2_opt->xyz);
            }
        };

    public:
        //! @brief Compute events for the supplied target point cloud.
        //! @param[out] out             Output buffers; every non-null buffer is filled.
        //! @param[in]  curr_polar      Polar coordinates (lon, lat, range) in the target sensor frame;
        //!                           shape (beam_W, 3, beam_H).
        //! @param[in]  tm_event_fn     Event function used to evaluate temporal differences.
        template<
            typename TmEventFn = event_fn::DiffVec,
            EL_ENABLE_WHEN(std::is_invocable_r_v<bool, TmEventFn, Ty*, Ty*, Ty, Ty, Ty, Ty, Ty, Ty>)>
        void compute_events_from_polar(
            const Out<Ty>& out, const Ty* curr_polar, TmEventFn tm_event_fn = {}, const ComputeEventsConfig cfg = {}) {
            // ======================================================================
            // 0)                        Set up variables
            // ======================================================================

            // ===== Get buffer pointers

            entry_type* const     prev_dm          = reinterpret_cast<entry_type* const>(prev_depth_map_.get());
            entry_type* const     curr_dm          = reinterpret_cast<entry_type* const>(current_depth_map_.get());

            dm_index1_type* const interpolate_idx  = reinterpret_cast<dm_index1_type* const>(interpolate_idx_.get());
            dm_index1_type* const interpolate_idx2 = reinterpret_cast<dm_index1_type* const>(interpolate_idx2_.get());

            if (out.prev_depth_map || out.prev_ip_idx || out.prev_ip_idx2) {
                for (int i = 0; i < resolution_W_; ++i) {
                    for (int j = 0; j < resolution_H_; ++j) {
                        const auto idx = indexer_.as1d(i, j);
                        if (out.prev_depth_map) {
                            const entry_type& entry       = prev_dm[idx];
                            const bool        entry_valid = !entry.empty();
                            for (int k = 0; k < 3; ++k) {
                                if (entry_valid) {
                                    out.prev_depth_map[(i * 3 + k) * resolution_H_ + j] = entry.xyz[k];
                                }
                                else {
                                    out.prev_depth_map[(i * 3 + k) * resolution_H_ + j] = 0;
                                }
                            }
                        }

                        if (out.prev_ip_idx) {
                            out.prev_ip_idx[idx] = interpolate_idx_[idx];
                        }
                        if (out.prev_ip_idx2) {
                            out.prev_ip_idx2[idx] = interpolate_idx2_[idx];
                        }
                    }
                }
            }

            // ===== Temporal Eventification

            for (int i = 0; i < beam_W_; ++i) {
                const Ty* const curr_lon   = curr_polar + i * kInputDims * beam_H_ + (kLonIndex * beam_H_);
                const Ty* const curr_lat   = curr_polar + i * kInputDims * beam_H_ + (kLatIndex * beam_H_);
                const Ty* const curr_range = curr_polar + i * kInputDims * beam_H_ + (kRangeIndex * beam_H_);

                // Treat all valid points as events without a reference frame or when requested.
                EL_IF_UNLIKELY(cfg.return_all || !reference_s2w_opt_) {
                    // ======================================================================
                    // 1) Convert the current frame's input points from polar coordinates
                    //    to Cartesian coordinates in the current sensor frame.
                    // 2) Build the current depth map (current-frame coordinates).
                    // ======================================================================

                    for (int j = 0; j < beam_H_; ++j) {
                        const Ty& lon_ = curr_lon[j];
                        const Ty& lat_ = curr_lat[j];
                        const Ty& r_   = curr_range[j];

                        if (r_ > 0) {
                            const auto& [x, y, z]  = polar2xyz(lon_, lat_, r_);
                            const auto& [i0, i1]   = dm_index_from_polar(lon_, lat_);

                            entry_type& curr_entry = indexer_.at(curr_dm, i0, i1);
                            curr_entry.set_xyz(x, y, z);
                        }
                    }

                    // ======================================================================
                    // 3) Mark valid points as events.
                    // ======================================================================

                    for (int j = 0; j < beam_H_; ++j) {
                        const bool is_valid = (curr_range[j] > 0);

                        if (out.event_flags) {
                            out.event_flags[i * beam_H_ + j] = is_valid;
                        }

                        const auto delta_result = (is_valid ? cfg.delta_init : 0);

                        if (out.delta_scalar) {
                            out.delta_scalar[i * beam_H_ + j] = delta_result;
                        }
                        if (out.delta_xyz) {
                            for (int k = 0; k < 3; ++k) {
                                out.delta_xyz[(i * 3 + k) * beam_H_ + j] = delta_result;
                            }
                        }
                    }
                }
                else {
                    // ======================================================================
                    // 1) Convert the current frame's input points from polar coordinates
                    //    to Cartesian coordinates in the current sensor frame.
                    // 2) Build the current depth map (current-frame coordinates).
                    // 3) Transform the current frame's Cartesian coordinates into the reference sensor frame.
                    // 4) For each current point, find neighboring points in the reference frame's depth map
                    //    and use them to construct local geometry.
                    // ======================================================================

                    for (int j = 0; j < beam_H_; ++j) {
                        const Ty& lon = curr_lon[j];
                        const Ty& lat = curr_lat[j];
                        const Ty& r   = curr_range[j];

                        if (r <= 0) {
                            if (out.event_flags) {
                                out.event_flags[i * beam_H_ + j] = false;
                            }

                            if (out.delta_scalar) {
                                out.delta_scalar[i * beam_H_ + j] = 0;
                            }
                            if (out.delta_xyz) {
                                for (int k = 0; k < 3; ++k) {
                                    out.delta_xyz[(i * 3 + k) * beam_H_ + j] = 0;
                                }
                            }
                            continue;
                        }

                        // Add the current point to the depth map in current-frame coordinates.
                        const auto& [i0_, i1_]   = dm_index_from_polar(lon, lat);
                        const auto& [x_, y_, z_] = polar2xyz(lon, lat, r);

                        entry_type& curr_entry   = indexer_.at(curr_dm, i0_, i1_);
                        curr_entry.set_xyz(x_, y_, z_);

                        // Transform the current point to reference-frame coordinates.
                        Vector3<Ty> curr_xyz{x_, y_, z_};
                        curr_xyz                     = curr2prev_rot_ * curr_xyz + curr2prev_transl_;

                        const auto& [lon_, lat_, r_] = xyz2polar(curr_xyz[0], curr_xyz[1], curr_xyz[2]);
                        const auto& [i0, i1]         = dm_index_from_polar(lon_, lat_);

                        using local_geo_t            = std::array<dm_index1_type, kTmDims + 1>;
                        local_geo_t local_geo;

                        if constexpr (kTmDims == 1) {
                            const dm_index1_type ip_idx  = indexer_.at(interpolate_idx, i0, i1);
                            const dm_index1_type ip_idx2 = indexer_.at(interpolate_idx2, i0, i1);

                            local_geo                    = {ip_idx, ip_idx2};
                        }
                        else {
                            const dm_index1_type ip_idx  = indexer_.at(interpolate_idx, i0, i1);
                            const dm_index1_type ip_idx2 = indexer_.at(interpolate_idx2, i0, i1);
                            const dm_index1_type ip_idx3
                                = (i0 == 0 ? indexer_.at(interpolate_idx, i0 + 1, i1)
                                           : indexer_.at(interpolate_idx, i0 - 1, i1));

                            local_geo = {ip_idx, ip_idx2, ip_idx3};
                        }

                        struct EventResult {
                            std::optional<Vector3<Ty>> prj_prev_opt;

                            Ty                         diff;
                            Ty                         thre;
                            bool                       event_flag;
                        } event_result;
                        event_result.prj_prev_opt = project_onto_local_geometry<kTmDims>(curr_xyz, prev_dm, local_geo);

                        if (event_result.prj_prev_opt) {
                            const auto& prj_prev    = *event_result.prj_prev_opt;

                            event_result.event_flag = tm_event_fn(
                                &event_result.diff,
                                &event_result.thre,
                                curr_xyz[0],
                                curr_xyz[1],
                                curr_xyz[2],
                                prj_prev[0],
                                prj_prev[1],
                                prj_prev[2]);

                            if (out.event_flags) {
                                out.event_flags[i * beam_H_ + j] = event_result.event_flag;
                            }

                            if (out.delta_scalar) {
                                const bool to_front               = norm_of(curr_xyz) < norm_of(prj_prev);
                                const auto delta_result           = (to_front ? -event_result.diff : event_result.diff);

                                out.delta_scalar[i * beam_H_ + j] = delta_result;
                            }
                            if (out.delta_xyz) {
                                const Vector3<Ty> delta_vec = curr_xyz - prj_prev;

                                for (int k = 0; k < 3; ++k) {
                                    out.delta_xyz[(i * 3 + k) * beam_H_ + j] = delta_vec[k];
                                }
                            }
                        }
                        else {    // No usable local geometry was found in the reference depth map.
                            if (out.event_flags) {
                                out.event_flags[i * beam_H_ + j] = true;
                            }

                            const auto delta_result = cfg.delta_no_prev;
                            if (out.delta_scalar) {
                                out.delta_scalar[i * beam_H_ + j] = delta_result;
                            }
                            if (out.delta_xyz) {
                                for (int k = 0; k < 3; ++k) {
                                    out.delta_xyz[(i * 3 + k) * beam_H_ + j] = delta_result;
                                }
                            }
                        }
                    }
                }
            }
        }
    };

    //! @brief Temporal eventification using a local plane approximation.
    template<typename Ty>
    using PlaneTemporalEventifier = BasicTemporalEventifier<Ty, 2>;

    //! @brief Temporal eventification using a local line approximation.
    template<typename Ty>
    using LineTemporalEventifier = BasicTemporalEventifier<Ty, 1>;
}    // namespace el
