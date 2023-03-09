/** @brief Interface for a graduated robust kernel.
 * That is a robust kernel \rho(r_i, \mu) that varies from highly convex (quadratic) when \mu = \mu_{init} to non-convex
 * (robust cost) as \mu trends towards \mu_{final}
 *
 *  @author Dan McGann
 *  @date Mar 2022
 */
#pragma once
#include <gtsam/base/FastVector.h>
#include <gtsam/base/Matrix.h>

#include <iostream>

using namespace gtsam;
namespace risam {
class GraduatedKernel {
  /** TYPES **/
 public:
  typedef boost::shared_ptr<GraduatedKernel> shared_ptr;

  /** FIELDS **/
 protected:
  /// @brief The initial Value for mu
  const double mu_init_;
  /// @brief The threshold at which to consider mu to be converged
  const double convergence_thresh_;

  /** INTERFACE **/
 public:
  GraduatedKernel(double mu_init, double convergence_thresh)
      : mu_init_(mu_init), convergence_thresh_(convergence_thresh) {}

  /** @brief Computes the graduated robust error of the function
   * Error = \rho(r_i)
   *
   * @param residual: r_i the current (whitened) residual
   * @param mu: \mu the current convexity parameter
   */
  virtual double error(const double &residual, const double &mu) const = 0;

  /** @brief Computes the weight of the graduated robust cost function
   * weight = w(r_i)
   *
   * @param residual: r_i the current (whitened) residual
   * @param mu: \mu the current convexity parameter
   */
  virtual double weight(const double &residual, const double &mu) const = 0;

  /** @brief Weights a linearized system
   * Weight results in the solving of: w_i * r_i^2 = \sqrt(w_i) (A_i x + b_i) = \sqrt(w_i)* A_i x + \sqrt(w_i) * b_i
   *
   * @param A: Whitened linear system from a factor (\Sigma^{-0.5} H_i)
   * @param b: Whitened linear system error from a factor (0.5 * \Sigma^(-0.5) ||h_i(x_i) - z_i||^2 = 0.5 * r_i^2)
   */
  void weightSystem(std::vector<Matrix> &A, Vector &b, const double &residual, const double &mu) const {
    double sqrt_weight = sqrt(weight(residual, mu));
    // std::cout << "w: " << sqrt_weight << " mu: " << mu << std::endl;
    for (Matrix &Aj : A) {
      Aj *= sqrt_weight;
    }
    b *= sqrt_weight;
  }

  /// @brief Returns the value of \mu_{init} for this graduated kernel
  double muInit() const { return mu_init_; }

  /// @brief Returns the next value of \mu for this graduated kernel
  virtual double updateMu(const double &mu) const = 0;

  /// @brief Returns the previous value of \mu for this graduated kernel
  virtual double updateMuInv(const double &mu) const = 0;

  /// @brief Returns true iff the value of \mu has converged to a non-convex state
  virtual bool isMuConverged(const double &mu) const = 0;
};

/* ************************************************************************* */
class GraduatedTLSKernel : public GraduatedKernel {
  /**Graduated version of the truncated least squares kernel
   * ref: Graduated Non-Convexity for Robust Spatial Perception: From Non-Minimal Solvers to Global Outlier Rejection
   */

  /** FIELDS **/
 public:
  /// @brief The sigma thresh at which to truncate from quadratic to constant cost
  const double truncation_thresh_;

  /** @brief Constructor
   * @param truncation_thresh: the normalized distance from the mean at which to truncate the error
   */
  GraduatedTLSKernel(double truncation_thresh = 1.0, double mu_init = 1e-2, double convergence_thresh = 1e5)
      : GraduatedKernel(mu_init, convergence_thresh), truncation_thresh_(truncation_thresh){};

  // Interface
  double error(const double &residual, const double &mu) const override;
  double weight(const double &residual, const double &mu) const;
  double updateMu(const double &mu) const override;
  double updateMuInv(const double &mu) const override;
  bool isMuConverged(const double &mu) const override;
};

/* ************************************************************************* */
class GraduatedGMKernel : public GraduatedKernel {
  /**Graduated version of the Geman-McClure kernel
   * ref: Graduated Non-Convexity for Robust Spatial Perception: From Non-Minimal Solvers to Global Outlier Rejection
   */

  /** FIELDS **/
 public:
  /// @brief The shape parameter of the GM kernel
  const double shape_param_;

  /** @brief Constructor
   * @param shape_param: the shape parameter for the GM
   */
  GraduatedGMKernel(double shape_param = 1.0, double mu_init = 1e4, double convergence_thresh = 1.1)
      : GraduatedKernel(mu_init, convergence_thresh), shape_param_(shape_param){};

  // Interface
  double error(const double &residual, const double &mu) const override;
  double weight(const double &residual, const double &mu) const override;
  double updateMu(const double &mu) const override;
  double updateMuInv(const double &mu) const override;
  bool isMuConverged(const double &mu) const override;
};

/* ************************************************************************* */
class SIGKernel : public GraduatedKernel {
  /** My version of the Graduated version of the Geman-McClure kernel */

  /** FIELDS **/
 public:
  /// @brief The shape parameter of the GM kernel
  const double shape_param_;

  /** @brief Constructor
   * @param shape_param: the shape parameter for the GM
   */
  SIGKernel(double shape_param = 3.0) : GraduatedKernel(0.0, 1.0), shape_param_(shape_param){};

  // Interface
  double error(const double &residual, const double &mu) const override;
  double weight(const double &residual, const double &mu) const override;
  double updateMu(const double &mu) const override;
  double updateMuInv(const double &mu) const override;
  bool isMuConverged(const double &mu) const override;
};
}  // namespace risam