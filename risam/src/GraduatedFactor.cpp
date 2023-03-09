#include "risam/GraduatedFactor.h"

namespace risam {
/* ************************************************************************* */
GraduatedFactor::GraduatedFactor(GraduatedKernel::shared_ptr kernel) : kernel_(kernel) {}

/* ************************************************************************* */
GraduatedFactor::GraduatedFactor(const GraduatedFactor& other) : kernel_(other.kernel_) {}

/* ************************************************************************* */
const GraduatedKernel::shared_ptr GraduatedFactor::kernel() const { return kernel_; }

}  // namespace risam