#include "refill/measurement_models/measurement_model_base.h"

using std::size_t;

namespace refill {
template<typename MeasurementType>
MeasurementModelBase<MeasurementType>::MeasurementModelBase(
    const size_t& state_dim, const size_t& measurement_dim,
    const DistributionInterface<MeasurementType>& measurement_noise)
    : state_dim_(state_dim),
      measurement_dim_(measurement_dim),
      measurement_noise_(measurement_noise.clone()) {
}

template<typename MeasurementType>
void MeasurementModelBase<MeasurementType>::setMeasurementModelBaseParameters(
    const size_t& state_dim, const size_t& measurement_dim,
    const DistributionInterface<MeasurementType>& measurement_noise) {
  state_dim_ = state_dim;
  measurement_dim_ = measurement_dim;
  measurement_noise_.reset(measurement_noise.clone());
}

template<typename MeasurementType>
size_t MeasurementModelBase<MeasurementType>::getStateDim() const {
  return state_dim_;
}

template<typename MeasurementType>
size_t MeasurementModelBase<MeasurementType>::getMeasurementDim() const {
  return measurement_dim_;
}

template<typename MeasurementType>
size_t MeasurementModelBase<MeasurementType>::getMeasurementNoiseDim() const {
  CHECK(measurement_noise_) << "Measurement noise has not been set.";
  return measurement_noise_->mean().size();
}

template<typename MeasurementType>
DistributionInterface<MeasurementType>*
MeasurementModelBase<MeasurementType>::getMeasurementNoise() const {
  CHECK(measurement_noise_) << "Measurement noise has not been set.";
  return measurement_noise_.get();
}

}  // namespace refill
