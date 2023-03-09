/** @brief Factor that implements a graduated robust cost function.
 * A Graduated factor overrides a portion of the NoiseModelFactor interface to implement a graduated robust cost
 * function for the factor.
 *
 *  @author Dan McGann
 *  @date Mar 2022
 */
#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "risam/GraduatedKernel.h"

namespace risam {
class GraduatedFactor {
  /** TYPES **/
 public:
  typedef boost::shared_ptr<GraduatedFactor> shared_ptr;

  /** FIELDS **/
 protected:
  /// @brief the Graduated Robust Kernel for this factor
  GraduatedKernel::shared_ptr kernel_;

  /**INTERFACE**/
 public:
  /** @brief Constructor
   * @param kernel: The graduated kernel to apply to this factor
   * @param args: The arguments required to construct the FACTOR_TYPE
   */
  GraduatedFactor(GraduatedKernel::shared_ptr kernel);

  /// @brief Copy constructor
  GraduatedFactor(const GraduatedFactor& other);

  /** @brief Linearize this factor at a specific point, using the specified convexification parameter mu
   *  @param current_estimate: the variable estimate at which to linearize the Factor
   *  @param mu: the current value of the convexification parameter for this factor
   */
  virtual gtsam::GaussianFactor::shared_ptr linearizeRobust(const gtsam::Values& current_estimate,
                                                            const double& mu) const = 0;

  /// @brief returns the residual of the factor
  virtual double residual(const gtsam::Values& current_estimate) const = 0;

  /// @brief returns \rho(r) of the factor
  virtual double robustResidual(const gtsam::Values& current_estimate, const double& mu) const = 0;

  /// @brief Returns the value of \mu_{init} for this graduated kernel
  const GraduatedKernel::shared_ptr kernel() const;
};

template <class FACTOR_TYPE>
class GenericGraduatedFactor : public FACTOR_TYPE, public GraduatedFactor {
  static_assert(std::is_base_of<gtsam::NonlinearFactor, FACTOR_TYPE>::value,
                "GraduatedFactor Must be instantiated with a Factor Derived from gtsam::NonlinearFactor.");

  /** TYPES **/
 public:
  typedef boost::shared_ptr<GenericGraduatedFactor<FACTOR_TYPE>> shared_ptr;

  /**INTERFACE**/
 public:
  /** @brief Constructor
   * @param args: The arguments required to construct the FACTOR_TYPE
   */
  template <class... Args>
  GenericGraduatedFactor(GraduatedKernel::shared_ptr kernel, Args&&... args)
      : FACTOR_TYPE(std::forward<Args>(args)...), GraduatedFactor(kernel) {}

  /// @brief Copy constructor
  GenericGraduatedFactor(const GenericGraduatedFactor<FACTOR_TYPE>& other)
      : FACTOR_TYPE(other), GraduatedFactor(other.kernel_) {}

  /// @brief makes a deep copy
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new GenericGraduatedFactor<FACTOR_TYPE>(*this)));
  }

  /** GRADUATED INTERFACE **/
  /* ************************************************************************* */
  gtsam::GaussianFactor::shared_ptr linearizeRobust(const gtsam::Values& current_estimate,
                                                    const double& mu) const override {
    // Compute the current residual of the underlying factor
    double r = residual(current_estimate);

    // Linearize the underlying factor
    gtsam::Matrix A;
    gtsam::Vector b;
    auto whitened_linear_system = FACTOR_TYPE::linearize(current_estimate);
    std::tie(A, b) = whitened_linear_system->jacobian();
    size_t output_dim = b.size();

    // Extract the non-dense linear system
    auto keys = whitened_linear_system->keys();
    std::vector<Matrix> Ablocks;
    size_t idx_start = 0;
    for (const auto& key : keys) {
      size_t d = current_estimate.at(key).dim();
      gtsam::Matrix vblock = gtsam::Matrix::Zero(output_dim, d);
      Ablocks.push_back(A.block(0, idx_start, output_dim, d));
      idx_start += d;
    }

    // Weight the Linearized Blocks using given control parameter value
    kernel_->weightSystem(Ablocks, b, r, mu);

    // Construct a jacobian factor from the weighted system
    FastMap<Key, Matrix> Ablock_map;
    for (int i = 0; i < Ablocks.size(); i++) {
      Ablock_map[keys[i]] = Ablocks[i];
    }
    return boost::make_shared<gtsam::JacobianFactor>(Ablock_map, b);
  }
  /* ************************************************************************* */
  double residual(const gtsam::Values& current_estimate) const override {
    return sqrt(2.0 * FACTOR_TYPE::error(current_estimate));
  }

  double robustResidual(const gtsam::Values& current_estimate, const double& mu) const override {
    return kernel_->error(residual(current_estimate), mu);
  }
};

/** HELPERS **/
template <class FACTOR_TYPE, class... Args>
GenericGraduatedFactor<FACTOR_TYPE> make_graduated(Args&&... args) {
  return GenericGraduatedFactor<FACTOR_TYPE>(std::forward<Args>(args)...);
}

template <class FACTOR_TYPE, class... Args>
typename GenericGraduatedFactor<FACTOR_TYPE>::shared_ptr make_shared_graduated(Args&&... args) {
  return boost::make_shared<GenericGraduatedFactor<FACTOR_TYPE>>(std::forward<Args>(args)...);
}
}  // namespace risam
