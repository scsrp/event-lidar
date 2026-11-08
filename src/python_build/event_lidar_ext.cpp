#include <array>
#include <utility>
#include <variant>

#include "../config.hpp"

#if !defined(EL_CXX_BUILD)
    #include <nanobind/nanobind.h>
    #include <nanobind/nb_defs.h>
    #include <nanobind/ndarray.h>
    #include <nanobind/stl/optional.h>    // Required to accept None as an optional value.
    #include <nanobind/stl/string.h>
    #include <nanobind/stl/variant.h>
    #include <nanobind/stl/vector.h>

    #include "../eventifier/temporal_diff.hpp"
    #include "../utils/fmt_dispatcher.hpp"
    #include "version.hpp"

namespace nb = nanobind;
using namespace nb::literals;

namespace nb_util {
    //! @brief Equivalent to np.empty(shape).
    template<typename Ty>
    nb::ndarray<nb::numpy, nb::c_contig, Ty> create_empty_ndarray(const std::array<std::size_t, 3>& shape) {
        // alloc
        const std::size_t size = shape[0] * shape[1] * shape[2];
        Ty* const         data = new Ty[size];

        // deleter
        nb::capsule       owner(data, [](void* ptr) noexcept { delete[] static_cast<Ty*>(ptr); });

        return nb::ndarray<nb::numpy, nb::c_contig, Ty>(data, shape.size(), shape.data(), owner);
    }

    namespace detail {
        template<class T, std::size_t N, std::size_t... I>
        nb::tuple to_py_tuple_impl(const std::array<T, N>& values, std::index_sequence<I...>) {
            return nb::make_tuple(values[I]...);
        }
    }

    template<class T, std::size_t N>
    nb::tuple make_tuple_from_array(const std::array<T, N>& values) {
        return detail::to_py_tuple_impl(values, std::make_index_sequence<N>{});
    }

}    // namespace nb_util

//! @brief Internal helpers for the Python bindings.
namespace detail {
    //! @brief Event function variants accepted by temporal eventifiers.
    using EventFnVariant = std::variant<el::event_fn::DiffVec, el::event_fn::DiffNorm>;

    //! @brief Allocate the requested ndarrays and register them in Out and the Python return dictionary.
    //! @param[out] out          Out passed to the C++ eventifier.
    //! @param[in]  out_types    Output types requested by Python.
    //! @param[in]  runtime_info Runtime information used to calculate output shapes.
    template<typename DType, typename EventType>
    nb::dict set_out_buffers(
        el::Out<DType>& out, const std::vector<el::OutType>& out_types, const el::RuntimeInfo& runtime_info) {
        nb::dict ret_dict;

        for (const auto ot : out_types) {
            const auto shape = el::get_out_shape(ot, runtime_info);
            switch (ot) {
            case el::OutType::kEventFlag:
            {
                auto buffer            = nb_util::create_empty_ndarray<EventType>(shape);
                out.event_flags        = buffer.data();
                ret_dict[nb::cast(ot)] = buffer;
                break;
            }
            case el::OutType::kDeltaScalar:
            case el::OutType::kDeltaXyz:
            case el::OutType::kPrevDepthMap:
            {
                auto buffer = nb_util::create_empty_ndarray<DType>(shape);
                if (ot == el::OutType::kDeltaScalar) {
                    out.delta_scalar = buffer.data();
                }
                else if (ot == el::OutType::kDeltaXyz) {
                    out.delta_xyz = buffer.data();
                }
                else {
                    out.prev_depth_map = buffer.data();
                }
                ret_dict[nb::cast(ot)] = buffer;
                break;
            }
            case el::OutType::kPrevIpIdx:
            case el::OutType::kPrevIpIdx2:
            {
                auto buffer = nb_util::create_empty_ndarray<std::size_t>(shape);
                if (ot == el::OutType::kPrevIpIdx) {
                    out.prev_ip_idx = buffer.data();
                }
                else {
                    out.prev_ip_idx2 = buffer.data();
                }
                ret_dict[nb::cast(ot)] = buffer;
                break;
            }
            default: throw std::runtime_error("invalid OutType");
            }
        }

        return ret_dict;
    }
    // clang-format off

//! @brief Define the Python binding for a BasicTemporalEventifier instantiation.
//! @param[in,out] m        Python module.
//! @param[in]     cls_name Python class name.
template<typename BindTy, typename DType, typename EventType>
void bind_temporal_eventifier(nb::module_& m, const char* const cls_name) {
    nb::class_<BindTy>(m, cls_name)
        // Constructor.
        .def(nb::init<const int, const int, const DType, const DType, const int>(), 
            "beam_H"_a, "beam_W"_a, "vfov_min_deg"_a, "vfov_max_deg"_a, "n_padding"_a = 2,
            R"(
            A class that converts LiDAR point cloud data into events.

            Args:
                beam_W    (int)  : The number of columns in the input range image.
                                   For Waymo, this is 2650.
                beam_H    (int)  : The number of rows in the input range image.
                                   For Waymo, this is 64.
            )"
        )
        // getter
        .def("beam_H", [](const BindTy& self) {return self.beam_H();}, 
            R"(Returns the range image height.
            
            Returns:
                int: beam_H
            )"
        )
        .def("beam_W", [](const BindTy& self) {return self.beam_W();},
            R"(Returns the range image width.
            
            Returns:
                int: beam_W
            )"
        )
        .def("resolution_H", [](const BindTy& self) {return self.resolution_H();}, 
            R"(Returns the depth map height.
            
            Returns:
                int: resolution_H
            )"
        )
        .def("resolution_W", [](const BindTy& self) {return self.resolution_W();},
            R"(Returns the depth map width.
            
            Returns:
                int: resolution_W
            )"
        )
        .def("get_runtime_info", [](const BindTy& self) {return self.get_runtime_info();},
            R"(Returns the range image and depth map dimensions.
            
            Returns:
                RuntimeInfo: runtime shape info
            )"
        )
        // member fn
        .def("reset", &BindTy::reset,
            R"(Resets the internal state to its initial state.)"
        )
        .def("update_reference_frame", &BindTy::update_reference_frame,
            R"(Uses the last processed target frame as the reference frame.

            Call after compute_events_from_polar(). The target depth map and pose become the reference,
            and the reference index tables are rebuilt. The map includes non-event points.
            )"
        )
        .def(
            "prepare_target_state",
            [](
                BindTy                                     & self,
                nb::ndarray<nb::numpy, nb::c_contig, DType>& curr_s2w_ht
            ) {
                if (curr_s2w_ht.size() != el::TransformCommon::kHTransformSize) {
                    throw std::runtime_error(el_fmt::format(
                        "unexpected curr_s2w_ht size, got = {}, expected = {}",
                        curr_s2w_ht.size(), el::TransformCommon::kHTransformSize));
                }
                self.prepare_target_state(curr_s2w_ht.data());
            },
            "curr_s2w_ht"_a,
            R"(Prepares the eventifier to process a target frame while retaining the reference frame.

            Updates the target-to-reference transform and clears the target depth map.
            Follow with compute_events_from_polar() to supply and process the target points.

            Args:
                curr_s2w_ht (np.ndarray): The target 4x4 sensor-to-world transformation matrix.

            Returns:
                None
            )"
        )
        .def(
            "prepare_for_next_frame",
            [](
                BindTy                                     & self, 
                nb::ndarray<nb::numpy, nb::c_contig, DType>& curr_s2w_ht,
                bool                                  const update_reference
            ) {
                if (curr_s2w_ht.size() != (el::TransformCommon::kHTransformSize)) {
                    throw std::runtime_error(el_fmt::format(
                        "unexpected curr_s2w_ht size, got = {}, expected = {}",
                        curr_s2w_ht.size(), el::TransformCommon::kHTransformSize));
                }
                
                self.prepare_for_next_frame(curr_s2w_ht.data(), update_reference);
            },
            "curr_s2w_ht"_a, "update_reference"_a = true,
            R"(Prepares the eventifier to process the next target frame, optionally updating the reference frame.

            Follow with compute_events_from_polar() to supply and process the target points.

            Args:
                curr_s2w_ht (np.ndarray): The target 4x4 sensor-to-world transformation matrix.
                update_reference (bool): If true, use the last processed target as the reference before preparing
                    the eventifier. If false, retain the existing reference. Defaults to true.
            
            Returns:
                None
            )"
        )
        .def(
            "compute_events_from_polar",
            [](
                BindTy                             & self, 
                std::vector<el::OutType>      const& out_types,
                nb::ndarray<nb::numpy,
                    nb::c_contig, DType>      const& curr_polar,
                EventFnVariant                const  event_fn_vrnt,
                std::optional<DType>                 delta_no_prev,
                std::optional<DType>                 delta_init,
                bool                          const  return_all     = false
            ) {
                // check input
                EL_IF_UNLIKELY(curr_polar.size() != (self.beam_W() * 3 * self.beam_H())) {
                    throw std::runtime_error("unexpected curr_polar size");
                }
                // validate option
                if (!delta_no_prev) {
                    delta_no_prev = EL_DEFAULT_DELTA;
                }
                if (!delta_init) {
                    delta_init = EL_DEFAULT_DELTA;
                }
                // set out
                el::Out<DType> out;
                const auto runtime_info = self.get_runtime_info();
                nb::dict ret_dict = set_out_buffers<DType, EventType>(out, out_types, runtime_info);

                // config
                const auto cfg = el::ComputeEventsConfig{}
                    .set_return_all(return_all)
                    .set_delta_init(*delta_init)
                    .set_delta_no_prev(*delta_no_prev);

                // exec
                std::visit([&](auto event_fn) {
                    self.compute_events_from_polar(out, curr_polar.data(), event_fn, cfg);
                }, event_fn_vrnt);

                return ret_dict;
            },
            "out_types"_a, "curr_polar"_a, "event_fn"_a,
            "delta_no_prev"_a = EL_DEFAULT_DELTA, "delta_init"_a = EL_DEFAULT_DELTA,
            "return_all"_a = false,
            R"(
            Computes event flags and the requested outputs for the supplied target point cloud.

            Args:
                out_types (List[OutType]):
                    list of desired output types
                curr_polar (np.ndarray) : Polar coordinates (lon, lat, range) in the target sensor frame.
                    Shape: (beam_W, 3, beam_H).
                event_fn                : DiffVec or DiffNorm
                delta_no_prev (float)   : Delta for points without usable reference geometry. Defaults to 0.
                delta_init (float)      : Delta for valid points before a reference is available or when
                                          return_all is true. Defaults to 0.
                return_all (bool)       : If true, treat all valid points as events. Without a reference frame, all valid points
                                          are treated as events even when return_all is false.
                                          Defaults to false.

            Returns:
                dict[OutType, np.ndarray]: kEventFlag, kDeltaScalar, and kDeltaXyz have shape
                    (beam_W, feature_dim, beam_H). kPrevDepthMap, kPrevIpIdx, and kPrevIpIdx2 have shape
                    (resolution_W, feature_dim, resolution_H). feature_dim is 3 for kDeltaXyz and kPrevDepthMap,
                    and 1 otherwise.
            )"
        )
        ;
}

}    // namespace detail

// Bindings for the event_lidar_ext module.
NB_MODULE(event_lidar_ext, m) {

    // NOTE: https://nanobind.readthedocs.io/en/latest/classes.html#binding-classes-with-template-parameters
    //       Bind each class template instantiation separately.
    using DType      = float;

    using EventType  = el::s32;    // Type representing 0 and 1.
    static_assert(std::is_arithmetic_v<DType>);

    using PlaneTemporalEventifierType = el::PlaneTemporalEventifier<DType>;
    using LineTemporalEventifierType  = el::LineTemporalEventifier<DType>;

    m.doc() = "Converts LiDAR point clouds into events.";

    m.def(
        "internal_version",
        []() -> std::string {
                return std::string(EVENT_LIDAR_VERSION_FULL);
        },
        R"(
            Returns:
                version string
        )"
    );

    // ===== RuntimeInfo

    nb::class_<el::RuntimeInfo>(m, "RuntimeInfo")
        .def(
            nb::init<int, int, int, int>(),
            "beam_H"_a,
            "beam_W"_a,
            "dm_H"_a,
            "dm_W"_a
        )
        .def_rw("beam_H", &el::RuntimeInfo::beam_H)
        .def_rw("beam_W", &el::RuntimeInfo::beam_W)
        .def_rw("dm_H", &el::RuntimeInfo::dm_H)
        .def_rw("dm_W", &el::RuntimeInfo::dm_W)
        .def("__repr__", [](const el::RuntimeInfo& self) {
            return "RuntimeInfo(beam_H=" + std::to_string(self.beam_H)
                + ", beam_W=" + std::to_string(self.beam_W)
                + ", dm_H=" + std::to_string(self.dm_H)
                + ", dm_W=" + std::to_string(self.dm_W)
                + ")";
        });

    // ===== OutType

    m.def(
        "get_out_shape",
        [](const el::OutType ot, const el::RuntimeInfo& ri) {
            const auto shape = el::get_out_shape(ot, ri);
            return nb_util::make_tuple_from_array(shape);
        },
        "out_type"_a, "runtime_info"_a,
        R"(
            Args:
                out_type: OutType
                runtime_info: RuntimeInfo

            Returns:
                Output shape (width, feature dimension, height)
        )"
    );
    m.def(
        "get_out_element_count",
        el::get_out_element_count, "out_type"_a, "runtime_info"_a,
        R"(
            Args:
                out_type: OutType
                runtime_info: RuntimeInfo

            Returns:
                Number of elements required for the output buffer
        )"
    );
    m.def(
        "get_out_feature_dim",
        el::get_out_feature_dim, "out_type"_a,
        R"(
            Args:
                out_type: OutType

            Returns:
                Size of the feature axis (e.g. kEventFlag -> 1, kDeltaXyz -> 3)
        )"
    );

    m.def(
        "str_to_out_type",
        el::str_to_out_type, "str"_a,
        R"(
            Args:
                str : str

            Returns:
                OutType
        )"
    );

    nb::enum_<el::OutType>(m, "OutType")
        .value("kEventFlag", el::OutType::kEventFlag, "Event flag (1 for an event, 0 otherwise).")
        .value("kDeltaScalar", el::OutType::kDeltaScalar, "Signed scalar difference from the projected reference point.")
        .value("kDeltaXyz", el::OutType::kDeltaXyz, "Difference vector from the projected reference-frame point to the current point.")
        .value("kPrevDepthMap", el::OutType::kPrevDepthMap, "Reference-frame depth map of XYZ points.")
        .value("kPrevIpIdx", el::OutType::kPrevIpIdx, "Rightward-interpolated reference depth-map indices.")
        .value("kPrevIpIdx2", el::OutType::kPrevIpIdx2, "Leftward-interpolated reference depth-map indices.");

    m.attr("OUT_TYPES_DIFF_FEATURE") = nb_util::make_tuple_from_array(el::kOutTypesDiffFeature);

    m.def(
        "get_out_types_diff_feature",
        []() {
            return nb_util::make_tuple_from_array(el::kOutTypesDiffFeature);
        },
        "Return output types corresponding to temporal difference features."
    );

            // ===== event fn
            

    nb::class_<el::event_fn::DiffVec>(m, "DiffVec")
        .def(nb::init<float>(), "threshold"_a,
            R"(
            Tests whether a point is an event.
            Event condition: |p - c| >= threshold, where p is the projection onto the reference frame's local geometry and c is the current point in reference-frame coordinates.
            )"
        );

    nb::class_<el::event_fn::DiffNorm>(m, "DiffNorm")
        .def(nb::init<float>(), "threshold"_a,
            R"(
            Tests whether a point is an event.
            Event condition: ||p| - |c|| >= threshold, where p is the projection onto the reference frame's local geometry and c is the current point in reference-frame coordinates.
            )"
        );

    // ===== LineTemporalEventifier, PlaneTemporalEventifier

    detail::bind_temporal_eventifier<LineTemporalEventifierType, DType, EventType>(m, "LineTemporalEventifier");
    detail::bind_temporal_eventifier<PlaneTemporalEventifierType, DType, EventType>(m, "PlaneTemporalEventifier");
}

// clang-format on

#endif    // #if not defined(EL_CXX_BUILD)
