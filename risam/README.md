# riSAM
This directory contains a generic implementation of the riSAM algorithm. 

## Modules
This package contains 4 modules that composed make up the riSAM algorithm. 
* `DoglegLineSearch` (DLLS): Implements the dog-leg linesearch algorithm (riSAM Paper Alg. 3)
    * Note: There are also many variants of DLLS implemented and can be enabled by changing around the `DoglegLineSearchParams`. The parameters of the algorithm used for the paper can be found in `experiments/exp_runner/include/exp_runner/Factory.h`.
* `GraduatedKernel` : Implements the graduated kernels presented in the original GNC work, as well as the SIG Kernel (riSAM Paper Eq. 2)
* `GraduatedFactor` : Implements a generic GTSAM factor augmented with a graduated kernel.
* `RISAM2` : Implements the riSAM algorithm as an extension of `gtsam::ISAM2`. Note there is a non-trivial chunk of code copied from the ISAM2 implementation as riSAM modifies ISAM2 behavior in ways that could not be injected with "better" coding practices. (riSAM Paper Alg. 4)

## Documentation
Documentation for the code itself is provided in-line and at a per-function level. Most of the code is well documented in this fashion, but it is research code so there are gaps. If anything is unclear open a issue and I will address gaps in documentation on an as-needed basis!