#include "refill/filters/particle_filter.h"

using std::size_t;

namespace refill {

template<typename StateType, typename MeasurementType>
ParticleFilter<StateType, MeasurementType>::ParticleFilter()
    : particles_(0),
      system_model_(nullptr),
      measurement_model_(nullptr),
      resample_method_(nullptr) {
}

template<typename StateType, typename MeasurementType>
ParticleFilter<StateType, MeasurementType>::ParticleFilter(
    const size_t& n_particles,
    const DistributionInterface<StateType>& initial_state_dist,
    const std::function<void(std::vector<ParticleType>*)>& resample_method)
    : particles_(n_particles),
      system_model_(nullptr),
      measurement_model_(nullptr),
      resample_method_(resample_method) {
  initializeParticles(initial_state_dist);
}

template<typename StateType, typename MeasurementType>
ParticleFilter<StateType, MeasurementType>::ParticleFilter(
    const size_t& n_particles,
    const DistributionInterface<StateType>& initial_state_dist,
    const std::function<void(std::vector<ParticleType>*)>& resample_method,
    std::unique_ptr<SystemModelBase<StateType>> system_model,
    std::unique_ptr<MeasurementModelBase<MeasurementType>> measurement_model)
    : particles_(n_particles),
      system_model_(std::move(system_model)),
      measurement_model_(std::move(measurement_model)),
      resample_method_(resample_method) {
  initializeParticles(initial_state_dist);
}

template<typename StateType, typename MeasurementType>
void ParticleFilter<StateType, MeasurementType>::setFilterParameters(
    const size_t& n_particles,
    const DistributionInterface<StateType>& initial_state_dist,
    const std::function<void(std::vector<ParticleType>*)>& resample_method) {
  particles_.resize(n_particles);
  initializeParticles(initial_state_dist);

  resample_method_ = resample_method;
}

template<typename StateType, typename MeasurementType>
void ParticleFilter<StateType, MeasurementType>::setFilterParameters(
    const size_t& n_particles,
    const DistributionInterface<StateType>& initial_state_dist,
    const std::function<void(std::vector<ParticleType>*)>& resample_method,
    std::unique_ptr<SystemModelBase<StateType>> system_model,
    std::unique_ptr<MeasurementModelBase<MeasurementType>> measurement_model) {
  particles_.resize(n_particles);
  initializeParticles(initial_state_dist);

  system_model_ = std::move(system_model);
  measurement_model_ = std::move(measurement_model);

  resample_method_ = resample_method;
}

template<typename StateType, typename MeasurementType>
void ParticleFilter<StateType, MeasurementType>::initializeParticles(
    const DistributionInterface<StateType>& initial_state_dist) {
  for (ParticleType& particle : particles_) {
    particle = initial_state_dist.drawSample();
  }
}

template<typename StateType, typename MeasurementType>
void ParticleFilter<StateType, MeasurementType>::resample() {
  resample_method_(&particles_);
}

}  // namespace refill
