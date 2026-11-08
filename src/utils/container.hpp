#pragma once

#include <ostream>
#include <string>

//! @brief Containers used by event-lidar.
namespace el::cnt {

    //! @brief Three-element counterpart of std::pair.
    template<typename Ty1, typename Ty2, typename Ty3>
    struct Triplet {
        Ty1  first;
        Ty2  second;
        Ty3  third;

        void dump(std::ostream& os, const std::string& sep = ", ", const std::string& end = "\n") const {
            os << first << sep << second << sep << third << end;
        }
    };

    template<typename Ty1, typename Ty2, typename Ty3>
    std::ostream& operator<<(std::ostream& os, const Triplet<Ty1, Ty2, Ty3>& value) {
        value.dump(os, ", ", "");
        return os;
    }

}    // namespace el::cnt
