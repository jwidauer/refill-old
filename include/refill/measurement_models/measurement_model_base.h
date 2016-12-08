#ifndef REFILL_MEASUREMENT_MODELS_MEASUREMENT_MODEL_BASE_H_
#define REFILL_MEASUREMENT_MODELS_MEASUREMENT_MODEL_BASE_H_

#include <Eigen/Dense>
#include <glog/logging.h>

#include <cstdlib>
#include <memory>

#include "refill/distributions/distribution_base.h"

using std::size_t;

namespace refill {

template<typename MeasurementType>
class MeasurementModelBase {
 public:
  virtual Eigen::VectorXd observe(const MeasurementType& state) const = 0;

  size_t getStateDim() const;
  size_t getMeasurementDim() const;
  size_t getMeasurementNoiseDim() const;
  DistributionInterface<MeasurementType>* getMeasurementNoise() const;

 protected:
  MeasurementModelBase() = delete;
  MeasurementModelBase(
      const size_t& state_dim, const size_t& measurement_dim,
      const DistributionInterface<MeasurementType>& measurement_noise);

  void setMeasurementModelBaseParameters(
      const size_t& state_dim, const size_t& measurement_dim,
      const DistributionInterface<MeasurementType>& measurement_noise);

 private:
  size_t state_dim_;
  size_t measurement_dim_;
  std::unique_ptr<DistributionInterface<MeasurementType>> measurement_noise_;
};

}  // namespace refill

#endif  // REFILL_MEASUREMENT_MODELS_MEASUREMENT_MODEL_BASE_H_
