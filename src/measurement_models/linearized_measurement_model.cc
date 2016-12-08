#include "refill/measurement_models/linearized_measurement_model.h"

using std::size_t;

namespace refill {

LinearizedMeasurementModel::LinearizedMeasurementModel(
    const size_t& state_dim, const size_t& measurement_dim,
    const DistributionInterface<Eigen::VectorXd>& measurement_noise)
    : MeasurementModelBase(state_dim, measurement_dim, measurement_noise) {}

}  // namespace refill
