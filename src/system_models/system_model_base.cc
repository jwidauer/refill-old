#include "refill/system_models/system_model_base.h"

using std::size_t;

namespace refill {


/**
 * Use this constructor if your system does not have an input.
 * The constructor clones the system noise, so it can be used again.
 *
 * @param state_dim The systems state dimension.
 * @param system_noise The system noise.
 */
SystemModelBase::SystemModelBase(const size_t& state_dim,
                                 const DistributionInterface& system_noise)
    : SystemModelBase(state_dim, system_noise, 0u) {}


/**
 * Use this constructor if your system model does have an input.
 * The constructor clones the system noise, so it can be used again.
 *
 * @param state_dim The systems state dimension.
 * @param system_noise The system noise.
 * @param input_dim The systems input dimension.
 */
SystemModelBase::SystemModelBase(const size_t& state_dim,
                                 const DistributionInterface& system_noise,
                                 const size_t& input_dim)
    : state_dim_(state_dim),
      input_dim_(input_dim),
      system_noise_(system_noise.clone()) {}

/**
 * @f$ N_s @f$ / @f$ N_n @f$ denotes state and noise dimensions respectively.
 *
 * @f$ M_s @f$ / @f$ M_n @f$ user defined number of state/noise samples.
 *
 * @param sampled_state is a @f$ N_s \times M_s @f$ Matrix.
 * @param input is the input to the system function.
 * @param sampled_noise is a @f$ N_n \times M_n @f$ Matrix.
 * @return an @f$ N_s \times (M_s \cdot M_n) @f$ Matrix.
 */
Eigen::MatrixXd SystemModelBase::propagateVectorized(
    const Eigen::MatrixXd& sampled_state, const Eigen::VectorXd& input,
    const Eigen::MatrixXd& sampled_noise) const {
  const size_t kStateDim = getStateDim();
  const size_t kInputDim = getInputDim();
  const size_t kNoiseDim = getNoiseDim();
  const size_t kStateSampleCount = sampled_state.cols();
  const size_t kNoiseSampleCount = sampled_noise.cols();

  CHECK_EQ(kStateDim, sampled_state.rows());
  CHECK_EQ(kNoiseDim, sampled_noise.rows());

  if (kInputDim != 0) {
    CHECK_EQ(kInputDim, input.rows());
  }

  Eigen::MatrixXd result(kStateDim, kStateSampleCount * kNoiseSampleCount);

  // Evaluate the propagate function for each combination of state / noise
  // samples.
  for (size_t i = 0u; i < kStateSampleCount; ++i) {
    for (size_t j = 0u; j < kNoiseSampleCount; ++j) {
      result.col(i * kNoiseSampleCount + j) = propagate(sampled_state.col(i),
                                                        input,
                                                        sampled_noise.col(j));
    }
  }

  return result;
}

/**
 * Use this function if your system does not have an input.
 * The function clones the system noise, so it can be used again.
 *
 * @param state_dim The systems state dimension.
 * @param system_noise The system noise.
 */
void SystemModelBase::setSystemModelBaseParameters(
    const std::size_t& state_dim, const DistributionInterface& system_noise) {
  this->setSystemModelBaseParameters(state_dim, system_noise, 0);
}

/**
 * Use this function if your system model does have an input.
 * The function clones the system noise, so it can be used again.
 *
 * @param state_dim The systems state dimension.
 * @param system_noise The system noise.
 * @param input_dim The systems input dimension.
 */
void SystemModelBase::setSystemModelBaseParameters(
    const std::size_t& state_dim, const DistributionInterface& system_noise,
    const std::size_t& input_dim) {
  state_dim_ = state_dim;
  system_noise_.reset(system_noise.clone());
  input_dim_ = input_dim;
}

/** @return the state dimension. */
size_t SystemModelBase::getStateDim() const {
  return state_dim_;
}

/** @return the input dimension. */
size_t SystemModelBase::getInputDim() const {
  return input_dim_;
}

/** @return the noise dimension. */
size_t SystemModelBase::getNoiseDim() const {
  CHECK(system_noise_) << "System noise has not been set.";
  return system_noise_->mean().size();
}

/** @return a pointer to the system noise distribution. */
DistributionInterface* SystemModelBase::getNoise() const {
  CHECK(system_noise_) << "System noise has not been set.";
  return system_noise_.get();
}

}  // namespace refill
