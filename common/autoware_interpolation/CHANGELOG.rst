^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_interpolation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* refactor(interpolation): use `autoware_utils\_*` instead of `autoware_utils` (`#382 <https://github.com/autowarefoundation/autoware_core/issues/382>`_)
  feat(interpolation): use split autoware utils
* fix(autoware_path_optimizer): incorrect application of input velocity due to badly mapping output trajectory to input trajectory (`#355 <https://github.com/autowarefoundation/autoware_core/issues/355>`_)
  * changes to avoid improper mapping
  * Update common/autoware_motion_utils/include/autoware/motion_utils/trajectory/trajectory.hpp
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Arjun Jagdish Ram, Takagi, Isamu

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* fix(autoware_interpolation): add missing dependencies (`#235 <https://github.com/autowarefoundation/autoware.core/issues/235>`_)
* feat: port autoware_interpolation from autoware.universe (`#149 <https://github.com/autowarefoundation/autoware.core/issues/149>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* Contributors: mitsudome-r, ralwing, shulanbushangshu
