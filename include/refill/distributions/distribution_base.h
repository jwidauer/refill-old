#ifndef REFILL_DISTRIBUTIONS_DISTRIBUTION_BASE_H_
#define REFILL_DISTRIBUTIONS_DISTRIBUTION_BASE_H_

#include <Eigen/Dense>
#include <glog/logging.h>
#include <typeinfo>

namespace refill {

// Interface class for distributions
template<typename ObjectType>
class DistributionInterface {
 public:
  virtual ObjectType mean() const = 0;
  virtual Eigen::MatrixXd cov() const = 0;
  virtual ObjectType drawSample() const = 0;
  virtual DistributionInterface* clone() const = 0;
};

//  Class that implements the Curiously Recurring Templating Pattern
//  so the clone function doesn't have to be implemented in every
//  derived distribution.
//  For new distributions, inherit from this class like this:
//  class NewDistribution : public DistributionBase<NewDistribution>

template<typename DERIVED, typename ObjectType>
class DistributionBase : public DistributionInterface<ObjectType> {
  virtual DistributionInterface* clone() const {
    DERIVED casted_derived_obj;
    try {
      casted_derived_obj = dynamic_cast<DERIVED const&>(*this);
    } catch (const std::bad_cast& e) {
      LOG(FATAL) << "Tried cloning, but encountered: " << e.what();
    }
    return new DERIVED(casted_derived_obj);
  }
};

}  // namespace refill

#endif  // REFILL_DISTRIBUTIONS_DISTRIBUTION_BASE_H_
