#ifndef REFILL_FILTERS_PARTICLE_FILTER_H_
#define REFILL_FILTERS_PARTICLE_FILTER_H_

#include <glog/logging.h>
#include <Eigen/Dense>

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "refill/distributions/distribution_base.h"
#include "refill/filters/filter_base.h"
#include "refill/measurement_models/measurement_model_base.h"
#include "refill/system_models/system_model_base.h"

using std::size_t;

namespace refill {

template<typename StateType, typename MeasurementType>
class ParticleFilter : public FilterBase<MeasurementType> {
 public:
  using ParticleType = std::pair<StateType, double>;

  ParticleFilter();
  ParticleFilter(
      const size_t& n_particles,
      const DistributionInterface<StateType>& initial_state_dist,
      const std::function<void(std::vector<ParticleType>*)>& resample_method);
  ParticleFilter(
      const size_t& n_particles,
      const DistributionInterface<StateType>& initial_state_dist,
      const std::function<void(std::vector<ParticleType>*)>& resample_method,
      std::unique_ptr<SystemModelBase<StateType>> system_model,
      std::unique_ptr<MeasurementModelBase<MeasurementType>> measurement_model);

  void setFilterParameters(
      const size_t& n_particles,
      const DistributionInterface<StateType>& initial_state_dist,
      const std::function<void(std::vector<ParticleType>*)>& resample_method);
  void setFilterParameters(
      const size_t& n_particles,
      const DistributionInterface<StateType>& initial_state_dist,
      const std::function<void(std::vector<ParticleType>*)>& resample_method,
      std::unique_ptr<SystemModelBase<StateType>> system_model,
      std::unique_ptr<MeasurementModelBase<MeasurementType>> measurement_model);

  void predict() override;
  void predict(const StateType& input);
  void predict(const SystemModelBase<StateType>& system_model);
  void predict(const SystemModelBase<StateType>& system_model,
               const StateType& input);

  void update(const MeasurementType& measurement) override;
  void update(const MeasurementModelBase<MeasurementType>& measurement_model,
              const MeasurementType& measurement);

 private:
  void initializeParticles(
      const DistributionInterface<StateType>& initial_state);
  void resample();

  std::vector<ParticleType> particles_;
  std::unique_ptr<SystemModelBase<StateType>> system_model_;
  std::unique_ptr<MeasurementModelBase<MeasurementType>> measurement_model_;
  std::function<void(std::vector<ParticleType>*)> resample_method_;
};

}  // namespace refill

#endif  // REFILL_FILTERS_PARTICLE_FILTER_H_
