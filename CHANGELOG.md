trifinger_simulation Changelog
==============================

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html)
(at least for versions > 1.4.1).


## [Unreleased]

## [2.0.0] - 2024-07-30
### Added
- Add py.typed marker for mypy
- screenshot_mode.py to run simulation without visualisations (for nicer screenshots).

### Deprecated
- The `pinocchio_utils` module is deprecated and will be removed in a future release.
  Use the one from robot_properties_fingers instead.

### Fixed
- Fixed a rounding error that occasionally resulted in NaN values when evaluating states
  in the "move_cuboid" task.
- Fixed included URDF files of (Tri)FingerPro (see [robot_properties_fingers #15
  ](https://github.com/open-dynamic-robot-initiative/robot_properties_fingers/pull/15)).

### Removed
- trifinger_simulation does no longer maintain a copy of the robot model files for the
  pure-Python installation.  Instead it depends on robot_properties_fingers now for both
  installation types (colcon and pure Python).


## [1.4.1] - 2022-06-24

- Switch from setup.py to setup.cfg
- Set dtypes of gym spaces to np.float64


## [1.4.0] - 2022-06-23

- Nicer rendering of the boundary (#68)
- Change initial pose of camera in PyBullet GUI (#68)
- Set proper initial position based on finger type (#79)
- Fix incompatibility with newer Gym versions (#81)
- Add option to save images to `demo_cameras` (#82)
- Simulate delay of camera observations (#84)
- Optionally compute tip velocities in forward kinematics (#85)
- Cleanup the code and add more unit tests

## [1.3.0] - 2021-08-04

There is no changelog for this or earlier versions.

---

[Unreleased]: https://github.com/open-dynamic-robot-initiative/trifinger_simluation/compare/v2.0.0...HEAD
[2.0.0]: https://github.com/open-dynamic-robot-initiative/trifinger_simluation/compare/v1.4.1...v2.0.0
[1.4.1]: https://github.com/open-dynamic-robot-initiative/trifinger_simluation/compare/v1.4.0...v1.4.1
[1.4.0]: https://github.com/open-dynamic-robot-initiative/trifinger_simluation/compare/v1.3.0...v1.4.0
[1.3.0]: https://github.com/open-dynamic-robot-initiative/trifinger_simluation/releases/tag/v1.3.0
