/** @brief Experiment Interface for Discrete Continuous SAM
 *
 * @author Dan McGann
 * @date Feb 2022
 */
#pragma once
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "dcsam/DCFactor.h"
#include "dcsam/DCSAM.h"
#include "exp_runner/Runner.h"

/**
 * DCBetweenFactor
 * 
 * Based heavily on 
 * dcsam:SemanticBearingRangeFactor
 */
template <typename VALUE_TYPE>
class DCBetweenFactor : public dcsam::DCFactor {
 private:
  typedef DCBetweenFactor<VALUE_TYPE> This;

  gtsam::BetweenFactor<VALUE_TYPE> factor_;
  std::vector<double> probs_;

 public:
  using Base = DCFactor;

  DCBetweenFactor() = default;

  DCBetweenFactor(const gtsam::Key& key1, const gtsam::Key& key2, const gtsam::DiscreteKey& discreteKey,
                  const std::vector<double> measuredProbs, const VALUE_TYPE& measured,
                  const gtsam::SharedNoiseModel& model)
      : probs_(measuredProbs), factor_(key1, key2, measured, model) {
    gtsam::KeyVector keys{key1, key2};
    gtsam::DiscreteKeys dks(discreteKey);
    keys_ = keys;
    discreteKeys_ = dks;
  }

  virtual ~DCBetweenFactor() = default;

  DCBetweenFactor& operator=(const DCBetweenFactor& rhs) {
    Base::operator=(rhs);
    this->factor_ = rhs.factor_;
    this->probs_ = rhs.probs_;
    this->keys_ = rhs.keys_;
    this->discreteKeys_ = rhs.discreteKeys_;
    return *this;
  }

  // Error is the sum of the continuous and discrete negative
  // log-likelihoods
  double error(const gtsam::Values& continuousVals, const DiscreteValues& discreteVals) const override {
    size_t assignment = discreteVals.at(discreteKeys_[0].first);
    double discrete_error = log(probs_[assignment]);

    // Subtraction because -log(p(A,B)) = -log p(A)p(B) = -log p(A) - log p(B)
    return factor_.error(continuousVals) - discrete_error;
  }

  // dim is the dimension of the underlying between factor
  size_t dim() const override { return factor_.dim(); }

  boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& continuousVals,
                                                     const DiscreteValues& discreteVals) const override {
    return factor_.linearize(continuousVals);
  }

  bool equals(const DCFactor& other, double tol = 1e-9) const override {
    // We attempt a dynamic cast from DCFactor to DCBetweenFactor.
    // If it fails, return false.
    if (!dynamic_cast<const DCBetweenFactor*>(&other)) return false;

    // If the cast is successful, we'll properly construct a
    // DCBetweenFactor object from `other`
    const DCBetweenFactor& f(static_cast<const DCBetweenFactor&>(other));

    // compare the between factors
    if (!(factor_.equals(f.factor_, tol))) return false;

    // If everything above passes, and the keys_, discreteKeys_ and probs_
    // variables are identical, return true.
    return (std::equal(keys_.begin(), keys_.end(), f.keys().begin()) && (discreteKeys_ == f.discreteKeys_) &&
            (probs_ == f.probs_));
  }

  double logNormalizingConstant(const gtsam::Values& values) const override {
    return nonlinearFactorLogNormalizingConstant(this->factor_, values);
  }
};

namespace exp_runner {

template <class POSE_TYPE>
class DCRunner : public Method<POSE_TYPE> {
  /** FIELDS **/
 private:
  /// @brief the actual solver
  boost::shared_ptr<dcsam::DCSAM> solver_;
  /// @brief internal tracker for current estimate
  dcsam::DCValues current_estimate_;
  /// @brief the current assignment of modes
  ModeSequence current_mode_sequence_;
  /// @brief information about mh measurements
  std::vector<std::pair<int, gtsam::DiscreteKey>> mh_measures_infos_;

  /** PARAMETERS **/
 private:
  int discrete_var_counter_{0};

  /** HELPERS **/
 protected:
  /** INTERFACE IMPL **/
 public:
  void init() override;
  void handlePrior(irl::Prior<POSE_TYPE>& prior) override;
  void handleOdometry(irl::Odometry<POSE_TYPE>& odom) override;
  void handleLoop(irl::Loop<POSE_TYPE>& loop) override;
  MethodEstimateResults getEstimate() override;
  MethodModeSeqResults getModeSequence() override;
  std::string getName() override;
};
}  // namespace exp_runner

#include "exp_runner/DCRunner-inl.h"