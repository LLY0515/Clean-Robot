Upstream: https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2
Commit: 9fb34752ada22b0bb6e207601f918f0044184140
Branch: master
License: MIT (LICENSE)

Vendored core only; no demonstration node. Changes: measured starting speed, zero starting acceleration, reject failed refinement, clear stale results, initialize trajectory counter, use refined controls, fully reset rolling grid, configurable dynamic limits, suppress verbose core stdout at compile time. Humble CMake integration is provided by the parent package.

Additional fixes: initialize A* storage, correct infinity sentinel, return stream reference; linear-time exact squared-Euclidean distance transform for obstacle inflation.

Ground-robot robustness changes: minimum eight reference samples, whole-trajectory collision checks (not the original first two thirds), time dilation without changing verified geometry, bounded A* endpoint adjustment, bounds checks while intersecting A* paths.

Field tracking fix: enforce exact cubic B-spline endpoint position, velocity and acceleration constraints before freezing the boundary controls in rebound optimization. The least-squares fit previously introduced sideways starts and backwards loops during replanning. A 20 m closed-loop test with pose noise, slow translation and actuator lag covers repeated local-plan extensions.

2026-10-26 return-to-route adaptation: configurable positive soft obstacle margin (default 0.10 m beyond the unchanged inflated grid), separately supplied original-route targets with frozen visibility/correspondences per solve, analytic route and control-polygon length gradients, route-tangent endpoint derivative proportional to the configured speed, and half-cell collision sampling bounded by spline derivative control points. This is a local ground-robot adaptation, not an upstream version update.
