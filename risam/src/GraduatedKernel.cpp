#include "risam/GraduatedKernel.h"

#include <limits>
namespace risam {

/**
 * ######## ##        ######
 *    ##    ##       ##    ##
 *    ##    ##       ##
 *    ##    ##        ######
 *    ##    ##             ##
 *    ##    ##       ##    ##
 *    ##    ########  ######
 */
/* ************************************************************************* */
double GraduatedTLSKernel::error(const double &residual, const double &mu) const {
  // Yang et al. 2020 eq: 6
  double c2 = truncation_thresh_ * truncation_thresh_;
  double r2 = residual * residual;
  if (r2 > ((mu + 1) / mu) * c2) {
    return r2;
  } else if (r2 < (mu / (mu + 1)) * c2) {
    return c2;
  } else {
    return 2 * truncation_thresh_ * abs(residual) * sqrt(mu * (mu + 1)) - mu * (c2 + r2);
  }
}
/* ************************************************************************* */
double GraduatedTLSKernel::weight(const double &residual, const double &mu) const {
  // Yang et al. 2020 eq: 14
  double c2 = truncation_thresh_ * truncation_thresh_;
  double r2 = residual * residual;
  if (r2 > ((mu + 1) / mu) * c2) {
    return 0;
  } else if (r2 < (mu / (mu + 1)) * c2) {
    return 1;
  } else {
    return (truncation_thresh_ / residual) * sqrt(mu * (mu + 1)) - 1;
  }
}

/* ************************************************************************* */
double GraduatedTLSKernel::updateMu(const double &mu) const {
  // Yang et al. 2020 Remark 5
  return mu * 1.4;
}

/* ************************************************************************* */
double GraduatedTLSKernel::updateMuInv(const double &mu) const {
  double prev_mu = mu / 1.4;
  return prev_mu < mu_init_ ? 1.0 : prev_mu;
}

/* ************************************************************************* */
bool GraduatedTLSKernel::isMuConverged(const double &mu) const { return mu >= convergence_thresh_; }

/**
 *  ######   ##     ##
 * ##    ##  ###   ###
 * ##        #### ####
 * ##   #### ## ### ##
 * ##    ##  ##     ##
 * ##    ##  ##     ##
 *  ######   ##     ##
 */
/* ************************************************************************* */
double GraduatedGMKernel::error(const double &residual, const double &mu) const {
  // Yang et al. 2020 eq: 4
  double r2 = residual * residual;
  double c2 = shape_param_ * shape_param_;
  return 0.5 * (mu * c2 * r2) / (mu * c2 + r2);
}

/* ************************************************************************* */
double GraduatedGMKernel::weight(const double &residual, const double &mu) const {
  // Yang et al. 2020 eq: 12
  double r2 = residual * residual;
  double c2 = shape_param_ * shape_param_;
  double sqrt_weight = ((mu * c2) / (r2 + mu * c2));
  return sqrt_weight * sqrt_weight;
}

/* ************************************************************************* */
double GraduatedGMKernel::updateMu(const double &mu) const {
  // Yang et al. 2020 Remark 5
  double next_mu = mu / 1.8;
  return next_mu < convergence_thresh_ ? 1.0 : next_mu;
}

/* ************************************************************************* */
double GraduatedGMKernel::updateMuInv(const double &mu) const {
  double prev_mu = mu * 1.8;
  return prev_mu > mu_init_ ? mu_init_ : prev_mu;
}

/* ************************************************************************* */
bool GraduatedGMKernel::isMuConverged(const double &mu) const { return mu <= convergence_thresh_; }

/**
 *  ######  ##     ##  ######  ########  #######  ##     ##     ######   ##     ##
 * ##    ## ##     ## ##    ##    ##    ##     ## ###   ###    ##    ##  ###   ###
 * ##       ##     ## ##          ##    ##     ## #### ####    ##        #### ####
 * ##       ##     ##  ######     ##    ##     ## ## ### ##    ##   #### ## ### ##
 * ##       ##     ##       ##    ##    ##     ## ##     ##    ##    ##  ##     ##
 * ##    ## ##     ## ##    ##    ##    ##     ## ##     ##    ##    ##  ##     ##
 *  ######   #######   ######     ##     #######  ##     ##     ######   ##     ##
 */
/* ************************************************************************* */
double SIGKernel::error(const double &residual, const double &mu) const {
  double r2 = residual * residual;
  double c2 = shape_param_ * shape_param_;
  return 0.5 * (c2 * r2) / (c2 + pow(r2, mu));
}

/* ************************************************************************* */
double SIGKernel::weight(const double &residual, const double &mu) const {
  double r2 = residual * residual;
  double c2 = shape_param_ * shape_param_;
  double sqrt_denom = c2 + pow(r2, mu);
  return (c2 * (c2 + pow(r2, mu) * (1 - mu))) / (sqrt_denom * sqrt_denom);
}

/* ************************************************************************* */
double SIGKernel::updateMu(const double &mu) const {
  return std::min(1.0, mu + (mu - mu_init_ + 0.1) * 1.2);
  // return std::min(1.0, mu + 0.2);
}

/* ************************************************************************* */
double SIGKernel::updateMuInv(const double &mu) const {
  return std::max(0.0, mu - 0.1);
  // return std::max(0.0, (mu + 1.2 * mu_init_ - 0.12) / 2.2);
}

/* ************************************************************************* */
bool SIGKernel::isMuConverged(const double &mu) const { return mu >= convergence_thresh_; }
}  // namespace risam