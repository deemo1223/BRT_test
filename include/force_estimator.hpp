#ifndef FORCE_ESTIMATOR_HPP
#define FORCE_ESTIMATOR_HPP

#include "data_types.hpp"

class ForceEstimator {
public:
    // Convert the four lengths into a wrench estimate.
    ForceResult estimate(const LengthArray& lengths_mm) const;
};

#endif
