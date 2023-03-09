/** @brief Experiment Interface for MaxMixture Hypothesis Tracking
 *
 * @author Dan McGann
 * @date January 2022
 */
#pragma once
#include "exp_runner/MaxMixRunner.h"
namespace exp_runner {

/*************************************************************************************************/
MaxMixtureFactor::MaxMixtureFactor(const gtsam::KeyVector& keys, const MixtureModel& mixture_components,
                                   const gtsam::FastVector<double>& weights)
    : NonlinearFactor(keys), mixture_model_(mixture_components) {
  // GUARD
  if (mixture_model_.size() != weights.size()) {
    throw std::runtime_error("MaxMixtureFactor: Number of components should match number of weights");
  }
  // Initializations
  std::transform(weights.begin(), weights.end(), std::back_inserter(log_weights_), (double (*)(double))std::log);
  std::transform(mixture_model_.begin(), mixture_model_.end(), std::back_inserter(negative_log_normalizers_),
                 (double (*)(const gtsam::NonlinearFactor::shared_ptr&))computeNegLogNormalizationConstant);
}

/*************************************************************************************************/

double MaxMixtureFactor::computeNegLogNormalizationConstant(const gtsam::NonlinearFactor::shared_ptr& component) {
  // Information matrix (inverse covariance matrix) for the factor.
  gtsam::NoiseModelFactor::shared_ptr component_as_noisemodel_factor =
      boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(component);
  gtsam::Matrix info_matrix;
  if (component_as_noisemodel_factor) {
    // If dynamic cast to NoiseModelFactor succeeded, see if the noise model is Gaussian
    gtsam::noiseModel::Base::shared_ptr component_noise_model = component_as_noisemodel_factor->noiseModel();
    gtsam::noiseModel::Gaussian::shared_ptr noise_model_as_gaussian =
        boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(component_noise_model);
    if (noise_model_as_gaussian) {
      // If the noise model is Gaussian, retrieve the information matrix
      info_matrix = noise_model_as_gaussian->information();
    } else {
      throw std::runtime_error("MaxMixtureFactor: attempting to use component factor with a non-gaussian noise model");
    }
  } else {
    throw std::runtime_error("MaxMixtureFactor: attempting to use component factor without noise model");
  }

  // Compute the (negative) log of the normalizing constant
  return (component->dim() * log(2.0 * M_PI) / 2.0) - (log(info_matrix.determinant()) / 2.0);
}

/*************************************************************************************************/
std::pair<size_t, double> MaxMixtureFactor::computeMaxIndexAndNormalizedError(
    const gtsam::Values& current_estimate) const {
  double min_error = std::numeric_limits<double>::infinity();
  size_t min_error_idx;
  for (int i = 0; i < mixture_model_.size(); i++) {
    double error = -log_weights_[i] + negative_log_normalizers_[i] + mixture_model_[i]->error(current_estimate);
    if (error < min_error) {
      min_error = error;
      min_error_idx = i;
    }
  }
  return std::make_pair(min_error_idx, min_error);
}
/*************************************************************************************************/
size_t MaxMixtureFactor::nrModes() const { return mixture_model_.size(); }

/*************************************************************************************************/
size_t MaxMixtureFactor::dim() const { return mixture_model_.front()->dim(); }

/*************************************************************************************************/
double MaxMixtureFactor::error(const gtsam::Values& current_estimate) const {
  // Unlike our internal use of error this method requires we return unnormalized error
  if (keys_.front() == gtsam::symbol_shorthand::X(915))
    std::cout << gtsam::DefaultKeyFormatter(keys_.front()) << " " << gtsam::DefaultKeyFormatter(keys_.back());

  // Find the component with minimum normalized error and return its unnormalized error
  double min_normalized_error = std::numeric_limits<double>::infinity();
  double best_error = 0;
  for (int i = 0; i < mixture_model_.size(); i++) {
    double error = mixture_model_[i]->error(current_estimate);
    double normalized_error = -log_weights_[i] + negative_log_normalizers_[i] + error;
    if (normalized_error < min_normalized_error) {
      min_normalized_error = normalized_error;
      best_error = error;
    }
  }
  if (keys_.front() == gtsam::symbol_shorthand::X(915)) std::cout << "unconstrained error " << best_error << std::endl;
  return best_error;
}

/*************************************************************************************************/
gtsam::GaussianFactor::shared_ptr MaxMixtureFactor::linearize(const gtsam::Values& current_estimate) const {
  auto idx_error_pair = computeMaxIndexAndNormalizedError(current_estimate);

  // Get the Linearization of the maximal component
  gtsam::Matrix maxA;
  gtsam::Vector maxb;
  auto linearized_maximal_component = mixture_model_[idx_error_pair.first]->linearize(current_estimate);
  std::tie(maxA, maxb) = linearized_maximal_component->jacobian();
  size_t output_dim = maxb.size();

  // Determine dimensions for each of the variables
  gtsam::FastMap<gtsam::Key, size_t> key_dim_map;
  for (const auto& k : keys_) {
    key_dim_map[k] = current_estimate.at(k).dim();
  }

  // Construct the container for the linearized blocks
  gtsam::FastMap<gtsam::Key, gtsam::Matrix> linearized_blocks;

  // Fill necessary blocks with info from the linearized factors
  gtsam::FastSet<gtsam::Key> filled_set;
  size_t max_component_start = 0;
  for (const auto& max_key : linearized_maximal_component->keys()) {
    size_t d = key_dim_map[max_key];
    gtsam::Matrix vblock = gtsam::Matrix::Zero(output_dim, d);
    vblock.block(0, 0, output_dim, d) = maxA.block(0, max_component_start, output_dim, d);
    max_component_start += d;
    filled_set.insert(max_key);
    linearized_blocks[max_key] = vblock;
  }

  // Fill remaining blocks with zeros
  for (const auto& k : keys_) {
    if (filled_set.count(k) == 0) {
      linearized_blocks[k] = gtsam::Matrix::Zero(output_dim, key_dim_map[k]);
    }
  }
  return boost::make_shared<gtsam::JacobianFactor>(linearized_blocks, maxb);
}

/*************************************************************************************************/
gtsam::NonlinearFactor::shared_ptr MaxMixtureFactor::clone() const {
  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new MaxMixtureFactor(*this)));
}

/*************************************************************************************************/
void MaxMixtureFactor::print(const std::string& s, const gtsam::KeyFormatter& kf) const {
  // Basic Info
  std::cout << s << "MaxMixture Factor";
  Base::print(s, kf);
  // Configuration
  std::cout << "\t"
            << "Log Weights: [";
  for (auto w : log_weights_) std::cout << w << ", ";
  std::cout << "]" << std::endl;
  // Components
  size_t idx = 0;
  for (auto& comp : mixture_model_) {
    comp->print("Component " + std::to_string(idx++), kf);
  }
}

/*************************************************************************************************/
template <class POSE_TYPE>
void MaxMixRunner<POSE_TYPE>::init() {
  solver_ = boost::make_shared<gtsam::ISAM2>(gtsam::ISAM2Params(gtsam::ISAM2DoglegParams()));
}

/*************************************************************************************************/
template <class POSE_TYPE>
void MaxMixRunner<POSE_TYPE>::handlePrior(irl::Prior<POSE_TYPE>& prior) {
  // In experiments priors are never multi-hypothesis
  gtsam::NonlinearFactorGraph new_factors;
  new_factors.push_back(prior.asFactor(0));
  gtsam::Values new_values = prior.getGenerator(0)(current_estimate_);
  solver_->update(new_factors, new_values);
  current_mode_sequence_.push_back(0);
  get_mode_from_factor_.push_back(false);
  current_estimate_ = solver_->calculateEstimate();
}

/*************************************************************************************************/
template <class POSE_TYPE>
void MaxMixRunner<POSE_TYPE>::handleOdometry(irl::Odometry<POSE_TYPE>& odom) {
  // In experiments odometry are never multi-hypothesis
  gtsam::NonlinearFactorGraph new_factors;
  new_factors.push_back(odom.asFactor(0));
  solver_->update(new_factors, odom.getGenerator(0)(current_estimate_));
  current_mode_sequence_.push_back(0);
  get_mode_from_factor_.push_back(false);
  current_estimate_ = solver_->calculateEstimate();
}

/*************************************************************************************************/
template <class POSE_TYPE>
void MaxMixRunner<POSE_TYPE>::handleLoop(irl::Loop<POSE_TYPE>& loop) {
  // Handle Single Hypothesis case
  if (loop.num_modes == 1) {
    gtsam::NonlinearFactorGraph new_factors;
    new_factors.push_back(loop.asFactor(0));
    solver_->update(new_factors, loop.getGenerator(0)(current_estimate_));
    current_mode_sequence_.push_back(0);
    get_mode_from_factor_.push_back(false);
  }
  // Handle MultiHypothesis Case
  else {
    MaxMixtureFactor::MixtureModel mixture_components;
    gtsam::FastVector<double> weights;

    for (size_t i = 0; i < loop.num_modes; i++) {
      if (!loop.measurements[i]->isNull()) {
        // Add measurement
        mixture_components.push_back(loop.asFactor(i));
        weights.push_back(1.0 - null_hypo_weight_);
      }
      // Null Hypothesis
      else {
        POSE_TYPE measure = boost::dynamic_pointer_cast<irl::LoopMeasure<POSE_TYPE>>(loop.measurements[0])->rel_pose;
        size_t dim = gtsam::traits<POSE_TYPE>::GetDimension(measure);
        mixture_components.push_back(boost::make_shared<gtsam::BetweenFactor<POSE_TYPE>>(
            loop.keys().front(), loop.keys().back(), measure,
            gtsam::noiseModel::Isotropic::Sigma(dim, null_hypo_cov_scale_)));
        weights.push_back(null_hypo_weight_);
      }
    }
    gtsam::NonlinearFactorGraph new_factors;
    new_factors.emplace_shared<MaxMixtureFactor>(loop.keys(), mixture_components, weights);
    // Preform update
    solver_->update(new_factors, loop.getGenerator(0)(current_estimate_));
    current_mode_sequence_.push_back(0);
    get_mode_from_factor_.push_back(true);
  }

  // Update current estimate info
  current_estimate_ = solver_->calculateEstimate();
}

/*************************************************************************************************/
template <class POSE_TYPE>
MethodEstimateResults MaxMixRunner<POSE_TYPE>::getEstimate() {
  MethodEstimateResults result;
  result.push_back(current_estimate_);
  return result;
}

/*************************************************************************************************/
template <class POSE_TYPE>
MethodModeSeqResults MaxMixRunner<POSE_TYPE>::getModeSequence() {
  gtsam::NonlinearFactorGraph factors = solver_->getFactorsUnsafe();
  // Update the ModeSequence with the sparse info from the hypothesis
  for (size_t i = 0; i < current_mode_sequence_.size(); i++) {
    if (get_mode_from_factor_[i]) {
      MaxMixtureFactor::shared_ptr mm_ptr = boost::dynamic_pointer_cast<MaxMixtureFactor>(factors.at(i));
      current_mode_sequence_[i] = mm_ptr->computeMaxIndexAndNormalizedError(current_estimate_).first;
    }
  }

  MethodModeSeqResults result;
  // Make copy of sequence
  ModeSequence seq;
  seq.assign(current_mode_sequence_.begin(), current_mode_sequence_.end());
  result.push_back(seq);
  return result;
}

/*************************************************************************************************/
template <class POSE_TYPE>
std::string MaxMixRunner<POSE_TYPE>::getName() {
  return "maxmix";
}
}  // namespace exp_runner