# Experiment Runner

This directory contains two things. First is the definition of a common "runner" interface to regularize how each method interacts with the main entry point `run-experiment`. This interface is contained in `Runner.h` + `Runner-inl.h`. Second, are the wrappers for each method to be compatible with the standard runner interface. These implementations can be found in `<METHOD>Runner.h` and `<METHOD>Runner-inl.h`.

All runners are templated by the type of the robot pose (Pose3, Pose2, Point2, Point3). Therefore they have to exist in headers. To maintain separation of definitions and implementations we treat the `-inl` ("inline") files are the source code files.