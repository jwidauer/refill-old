#include "refill/system_models/system_model_base.h"

using std::size_t;

namespace refill {

template<typename StateType>
SystemModelBase<StateType>::SystemModelBase(
    const size_t& state_dim,
    const DistributionInterface<StateType>& system_noise)
    : SystemModelBase(state_dim, system_noise, 0) {
}

template<typename StateType>
SystemModelBase<StateType>::SystemModelBase(
    const size_t& state_dim,
    const DistributionInterface<StateType>& system_noise,
    const size_t& input_dim)
    : state_dim_(state_dim),
      input_dim_(input_dim),
      system_noise_(system_noise.clone()) {
}

template<typename StateType>
void SystemModelBase<StateType>::setSystemModelBaseParameters(
    const std::size_t& state_dim,
    const DistributionInterface<StateType>& system_noise) {
  this->setSystemModelBaseParameters(state_dim, system_noise, 0);
}

template<typename StateType>
void SystemModelBase<StateType>::setSystemModelBaseParameters(
    const std::size_t& state_dim,
    const DistributionInterface<StateType>& system_noise,
    const std::size_t& input_dim) {
  state_dim_ = state_dim;
  system_noise_.reset(system_noise.clone());
  input_dim_ = input_dim;
}

template<typename StateType>
size_t SystemModelBase<StateType>::getStateDim() const {
  return state_dim_;
}

template<typename StateType>
size_t SystemModelBase<StateType>::getInputDim() const {
  return input_dim_;
}

template<typename StateType>
size_t SystemModelBase<StateType>::getSystemNoiseDim() const {
  CHECK(system_noise_) << "System noise has not been set.";
  return system_noise_->mean().size();
}

template<typename StateType>
DistributionInterface<StateType>*
SystemModelBase<StateType>::getSystemNoise() const {
  CHECK(system_noise_) << "System noise has not been set.";
  return system_noise_.get();
}

}  // namespace refill
