#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizerImpl.h>

#pragma once
using namespace gtsam;

namespace risam {

enum DoglegLineSearchType { OUTWARD, MIDPOINT, GOLDEN, GRID };

struct DoglegLineSearchParams {
  DoglegLineSearchType search_type{OUTWARD};  // The search type to perform
  double min_delta{1e-3};                     // Minimum allowed small delta
  double init_delta{1};                       // Initial Delta to test
  double max_delta{1e2};                      // Maximum allowed delta
  double out_step_multiple{1.5};              // Increase of trust region for outward steps
  double in_step_multiple{0.5};               // Increase of trust region for inward steps
  double sufficent_decrease_coeff{1e-4};      // Coefficient for sufficient decrease check
  double num_grid_samples{100};               // Number of samples to use for grid search
  double wildfire_threshold{1e-3};
};

struct DoglegLineSearchImpl {
  template <class Rd> // OUTWARD SEARCH IS THE BEST ACCORDING TO EXPERIMENTAL RESULTS
  static DoglegOptimizerImpl::IterationResult OutwardSearch(const double min_delta, const double max_delta,
                                                            const double step_multiple,
                                                            const double sufficent_decrease_coeff,
                                                            const VectorValues& dx_u, const VectorValues& dx_n,
                                                            const Rd& risam, const Values& x0) {
    const double F_init_error = risam.robustError(x0);
    const double gn_step = dx_n.norm();
    const double max_step = std::min(max_delta, gn_step);
    double step = std::min(min_delta, gn_step);

    DoglegOptimizerImpl::IterationResult result;
    result.dx_d = DoglegOptimizerImpl::ComputeDoglegPoint(step, dx_u, dx_n, false);
    result.f_error = risam.robustError(x0.retract(result.dx_d));
    result.delta = step;

    // search Increase delta
    bool stay = step < max_step;
    while (stay) {
      step *= step_multiple;
      VectorValues dx_d = DoglegOptimizerImpl::ComputeDoglegPoint(step, dx_u, dx_n, false);
      Values x_d(x0.retract(dx_d));
      double new_F_error = risam.robustError(x_d);
      bool update_increased_error = new_F_error > result.f_error;
      bool step_has_sufficient_decrease = new_F_error < (F_init_error - sufficent_decrease_coeff * step);
      stay = !(step_has_sufficient_decrease && update_increased_error) && step < max_step;

      if (step_has_sufficient_decrease) {
        result.f_error = new_F_error;
        result.dx_d = dx_d;
        result.delta = step;
      }
    }
    return result;
  }

  template <class Rd>
  static DoglegOptimizerImpl::IterationResult MidpointSearch(const double min_delta, const double init_delta,
                                                             const double max_delta, const double outward_step_multiple,
                                                             const double inward_step_multiple,
                                                             const VectorValues& dx_u, const VectorValues& dx_n,
                                                             const Rd& risam, const Values& x0) {
    const double F_init_error = risam.robustError(x0);
    const double gn_step = dx_n.norm();
    const double max_step = std::min(max_delta, gn_step);
    double step = std::min(init_delta, gn_step);

    DoglegOptimizerImpl::IterationResult result;
    result.dx_d = DoglegOptimizerImpl::ComputeDoglegPoint(step, dx_u, dx_n, false);
    result.f_error = risam.robustError(x0.retract(result.dx_d));
    result.delta = step;

    // Search outward or inward depending on init_error
    bool stay = min_delta < step && step < max_delta;
    double step_dir = result.f_error < F_init_error ? outward_step_multiple : inward_step_multiple;
    while (stay) {
      step *= step_dir;
      VectorValues dx_d = DoglegOptimizerImpl::ComputeDoglegPoint(step, dx_u, dx_n, false);
      Values x_d(x0.retract(dx_d));
      double new_F_error = risam.robustError(x_d);
      stay = (new_F_error <= result.f_error) && min_delta < step && step < max_delta;

      if (new_F_error <= result.f_error) {
        result.f_error = new_F_error;
        result.dx_d = dx_d;
        result.delta = step;
      }
    }
    return result;
  }

  template <class Rd>
  static DoglegOptimizerImpl::IterationResult GoldenSearch(const double min_delta, const double max_delta,
                                                           const VectorValues& dx_u, const VectorValues& dx_n,
                                                           const Rd& risam, const Values& x0) {
    double a = min_delta;
    double b = std::min(max_delta, dx_n.norm());
    const double gr = (std::sqrt(5) + 1 / 2);
    double c = b - (b - a) / gr;
    double d = a + (b - a) / gr;

    while (std::abs(b - a) > 0.1) {
      double f_error_c = risam.robustError(DoglegOptimizerImpl::ComputeDoglegPoint(c, dx_u, dx_n, false));
      double f_error_d = risam.robustError(DoglegOptimizerImpl::ComputeDoglegPoint(d, dx_u, dx_n, false));
      if (f_error_c < f_error_d) {
        b = d;
      } else {
        a = c;
      }

      c = b - (b - a) / gr;
      d = a + (b - a) / gr;
    }
    double best_delta = (a + b) / 2.0;
    DoglegOptimizerImpl::IterationResult result;
    result.dx_d = DoglegOptimizerImpl::ComputeDoglegPoint(best_delta, dx_u, dx_n, false);
    result.f_error = risam.robustError(x0.retract(result.dx_d));
    result.delta = best_delta;
    return result;
  }

  template <class Rd>
  static DoglegOptimizerImpl::IterationResult GridSearch(const double min_delta, const double max_delta,
                                                         const double num_samples, const VectorValues& dx_u,
                                                         const VectorValues& dx_n, const Rd& risam, const Values& x0) {
    const double gn_step = dx_n.norm();
    const double max_step = std::min(max_delta, gn_step);
    double step = std::min(min_delta, gn_step);
    const double step_increment = ((max_step - step) / num_samples) + 1e-6;

    DoglegOptimizerImpl::IterationResult result;
    result.dx_d = DoglegOptimizerImpl::ComputeDoglegPoint(step, dx_u, dx_n, false);
    result.f_error = risam.robustError(x0.retract(result.dx_d));
    result.delta = step;

    while (step <= max_step) {
      VectorValues dx_d = DoglegOptimizerImpl::ComputeDoglegPoint(step, dx_u, dx_n, false);
      double error = risam.robustError(x0.retract(dx_d));
      if (error < result.f_error) {
        result.dx_d = dx_d;
        result.f_error = error;
        result.delta = step;
      }
      step += step_increment;
    }
    return result;
  }

  /** @brief Computes one iteration of linesearch on the DogLeg Trajectory
   * Step locations are computed by the dogleg point for increasing sizes of the Dogleg trust region
   * @param init_delta: The initial step size usually a small number
   * @param max_delta: The maximum step size (may be clipped if magnitude of dx_n is smaller)
   * @param dx_u: Cauchy Point
   * @param dx_n: Gauss Newton Point
   * @param risam: The incremental solver used to compute linear and non-linear error
   * @param x0: The current estimate of values
   */
  template <class Rd>
  static DoglegOptimizerImpl::IterationResult Iterate(const DoglegLineSearchParams& params, const VectorValues& dx_u,
                                                      const VectorValues& dx_n, const Rd& risam, const Values& x0) {
    switch (params.search_type) {
      case DoglegLineSearchType::OUTWARD:
        return OutwardSearch(params.init_delta, params.max_delta, params.out_step_multiple,
                             params.sufficent_decrease_coeff, dx_u, dx_n, risam, x0);
      case DoglegLineSearchType::MIDPOINT:
        return MidpointSearch(params.min_delta, params.init_delta, params.max_delta, params.in_step_multiple,
                              params.out_step_multiple, dx_u, dx_n, risam, x0);
      case DoglegLineSearchType::GOLDEN:
        return GoldenSearch(params.min_delta, params.max_delta, dx_u, dx_n, risam, x0);
      case DoglegLineSearchType::GRID:
        return GridSearch(params.min_delta, params.max_delta, params.num_grid_samples, dx_u, dx_n, risam, x0);
      default:
        throw std::runtime_error("Provided DoglegSearchType not implemented");
    };
  }
};

}  // namespace risam