#include "force_estimator.hpp"

// Return a zero wrench until the real force model is implemented.
ForceResult ForceEstimator::estimate(const LengthArray& lengths_mm) const {
    (void)lengths_mm;

    // TODO: Replace this placeholder with the elastic mechanism model.
    // TODO: Map the four displacement values into estimated force/torque.
    return ForceResult{};
}
