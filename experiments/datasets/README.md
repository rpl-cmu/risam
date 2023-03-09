This directory holds a number of benchmark pose-graph datasets. Some of which were used in the original riSAM publication. These benchmark datasets are all stored as g2o files and were originally meant for batch optimization. See `scripts` for info on how to convert these to IRL datasets for use with the rest of the `experiment` machinery.

Datasets:
* City1000 \[2d\] (also referred to as City10k) - Very long synthetic grid world dataset 
* CSAIL \[2d\] - Short real-world dataset with very few loop closures
* Cubicle \[3d\] (broken) - longer term 3d, but still largely planar dataset
* Garage \[3d\] - longer real-world 3d dataset with interesting elevation changes, however, the noise models are very inaccurate to reality
* Manhattan \[2d\] - mid length synthetic grid world dataset
* rim \[3d\] (broken) - longer term real-world 3d dataset, like cubicle is largely planar 
* sphere_a \[3d\] - Larger synthetic 3d dataset
* sphere2500 \[3d\] - Larger synthetic 3d dataset, different (not sure how) from sphere_a

Each subdirectory contains a readme with source from which from which the datasets were taken. These may not be the original source of the datasets.

Note: Cubicle and RIM are broken from an incremental perspective. Both pose graphs are missing at least one odometry link this means that when running incrementally there are unconstrained variables for at least one step. They can still be optimized in batch because loop-closures ensure that all variables are constrained, but are useless when converting to an irl file and running incrementally.