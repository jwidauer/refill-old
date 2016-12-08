#ifndef REFILL_SYSTEM_MODELS_SYSTEM_MODEL_BASE_H_
#define REFILL_SYSTEM_MODELS_SYSTEM_MODEL_BASE_H_

#include <Eigen/Dense>
#include <glog/logging.h>

#include <cstdlib>
#include <memory>

#include "refill/distributions/distribution_base.h"

using std::size_t;

namespace refill {

template<typename StateType>
class SystemModelBase {
 public:
  virtual StateType propagate(const StateType& state,
                              const StateType& input) const = 0;

  size_t getStateDim() const;
  size_t getInputDim() const;
  size_t getSystemNoiseDim() const;
  DistributionInterface<StateType>* getSystemNoise() const;

 protected:
  SystemModelBase() = delete;
  SystemModelBase(const size_t& state_dim,
                  const DistributionInterface<StateType>& system_noise);
  SystemModelBase(const size_t& state_dim,
                  const DistributionInterface<StateType>& system_noise,
                  const size_t& input_dim);

  void setSystemModelBaseParameters(
      const size_t& state_dim,
      const DistributionInterface<StateType>& system_noise);
  void setSystemModelBaseParameters(
      const size_t& state_dim,
      const DistributionInterface<StateType>& system_noise,
      const size_t& input_dim);

 private:
  size_t state_dim_;
  size_t input_dim_;
  std::unique_ptr<DistributionInterface<StateType>> system_noise_;
};

}  // namespace refill

#endif  // REFILL_SYSTEM_MODELS_SYSTEM_MODEL_BASE_H_
