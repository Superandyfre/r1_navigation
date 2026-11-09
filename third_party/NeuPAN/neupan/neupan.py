'''
neupan file is the main class for the NeuPan algorithm. It wraps the PAN class and provides a more user-friendly interface.

Developed by Ruihua Han
Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>

NeuPAN planner is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

NeuPAN planner is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with NeuPAN planner. If not, see <https://www.gnu.org/licenses/>.
'''

import yaml
import torch
from neupan.robot import robot
from neupan.blocks import InitialPath, PAN
from neupan import configuration
from neupan.util import time_it, file_check, get_transform
import numpy as np
import torch
from neupan.configuration import np_to_tensor, tensor_to_np
from math import cos, sin

class neupan(torch.nn.Module):

    """
    Args:
        receding: int, the number of steps in the receding horizon.
        step_time: float, the time step in the MPC framework.
        ref_speed: float, the reference speed of the robot.
        device: str, the device to run the algorithm on. 'cpu' or 'cuda'.
        robot_kwargs: dict, the keyword arguments for the robot class.
        ipath_kwargs: dict, the keyword arguments for the initial path class.
        pan_kwargs: dict, the keyword arguments for the PAN class.
        adjust_kwargs: dict, the keyword arguments for the adjust class
        train_kwargs: dict, the keyword arguments for the train class
        time_print: bool, whether to print the forward time of the algorithm.
        collision_threshold: float, the threshold for the collision detection. If collision, the algorithm will stop.
    """

    def __init__(
        self,
        receding: int = 10,
        step_time: float = 0.1,
        ref_speed: float = 4.0,
        device: str = "cpu",
        robot_kwargs: dict = None,
        ipath_kwargs: dict = None,
        pan_kwargs: dict = None,
        adjust_kwargs: dict = None,
        train_kwargs: dict = None,
        **kwargs,
    ) -> None:
        super(neupan, self).__init__()

        # mpc parameters
        self.T = receding
        self.dt = step_time
        self.ref_speed = ref_speed

        configuration.device = torch.device(device)
        configuration.time_print = kwargs.get("time_print", False)
        self.collision_threshold = kwargs.get("collision_threshold", 0.1)
        # initialization
        self.cur_vel_array = np.zeros((2, self.T))
        self.robot = robot(receding, step_time, **robot_kwargs)

        self.ipath = InitialPath(
            receding, step_time, ref_speed, self.robot, **ipath_kwargs
        )
            
        pan_kwargs["adjust_kwargs"] = adjust_kwargs
        pan_kwargs["train_kwargs"] = train_kwargs
        self.dune_train_kwargs = train_kwargs

        self.pan = PAN(receding, step_time, self.robot, **pan_kwargs)
        self.candidate_avoidance_enabled = bool(pan_kwargs.get("candidate_avoidance_enabled", False))
        self.preferred_avoidance_side = 0
        self.preferred_avoidance_mode = None
        self.candidate_search_cursor = 0
        self.info = {"stop": False, "arrive": False, "collision": False}

    @classmethod
    def init_from_yaml(cls, yaml_file, **kwargs):
        abs_path = file_check(yaml_file)

        with open(abs_path, "r") as f:
            config = yaml.safe_load(f)
            config.update(kwargs)

        config["robot_kwargs"] = config.pop("robot", dict())
        config["ipath_kwargs"] = config.pop("ipath", dict())
        config["pan_kwargs"] = config.pop("pan", dict())
        config["adjust_kwargs"] = config.pop("adjust", dict())
        config["train_kwargs"] = config.pop("train", dict())

        return cls(**config)

    @time_it("neupan forward")
    def forward(self, state, points, velocities=None,
                measured_velocity=None):
        """
        state: current state of the robot, matrix (3, 1), x, y, theta
        points: current input obstacle point positions, matrix (2, N), N is the number of obstacle points.
        velocities: current velocity of each obstacle point, matrix (2, N), N is the number of obstacle points. vx, vy
        """

        assert state.shape[0] >= 3
        self.info["stop_reason"] = "none"
        self.info.pop("avoidance_candidates", None)
        self.info.pop("avoidance_side", None)
        self.info.pop("candidate_infeasible", None)
        if self.ipath.check_arrive(state):
            self.info["arrive"] = True
            self.info["stop"] = False
            return np.zeros((2, 1)), self.info
        self.info["arrive"] = False
        nom_input_np = self.ipath.generate_nom_ref_state(
            state, self.cur_vel_array, self.ref_speed)

        # convert to tensor
        nom_input_tensor = [np_to_tensor(n) for n in nom_input_np]
        obstacle_points_tensor = np_to_tensor(points) if points is not None else None
        point_velocities_tensor = (
            np_to_tensor(velocities) if velocities is not None else None
        )

        measured_velocity_tensor = (
            np_to_tensor(measured_velocity) if measured_velocity is not None else None
        )
        if (self.candidate_avoidance_enabled and self.robot.kinematics == "diff" and
                points is not None and points.shape[1] and not self.pan.no_obs):
            opt_state_tensor, opt_vel_tensor, opt_distance_tensor = self._solve_candidates(
                state, nom_input_np, points, velocities, measured_velocity)
        else:
            opt_state_tensor, opt_vel_tensor, opt_distance_tensor = self.pan(
                *nom_input_tensor, obstacle_points_tensor, point_velocities_tensor,
                measured_velocity_tensor,
            )

        opt_state_np, opt_vel_np = tensor_to_np(opt_state_tensor), tensor_to_np(
            opt_vel_tensor
        )

        # Shift the previous solution with the receding horizon. Reusing its
        # first knot forever can repeatedly choose a zero first command while
        # postponing all movement to the second knot.
        self.cur_vel_array = (np.hstack((opt_vel_np[:, 1:], opt_vel_np[:, -1:]))
                              if self.candidate_avoidance_enabled and opt_vel_np.shape[1] > 1
                              else opt_vel_np)

        self.info["state_tensor"] = opt_state_tensor
        self.info["vel_tensor"] = opt_vel_tensor
        self.info["distance_tensor"] = opt_distance_tensor
        self.info['ref_state_tensor'] = nom_input_tensor[2]
        self.info['ref_speed_tensor'] = nom_input_tensor[3]

        self.info["ref_state_list"] = [
            state[:, np.newaxis] for state in nom_input_np[2].T
        ]
        self.info["opt_state_list"] = [state[:, np.newaxis] for state in opt_state_np.T]

        candidate_checked = "avoidance_candidates" in self.info
        candidate_infeasible = self.info.get("candidate_infeasible", False)
        # The original current-pose threshold blocks escape even when every
        # point is outside the model and a candidate moves clear. With the
        # internal candidate planner enabled, use its full-horizon feasibility;
        # otherwise retain the original threshold semantics.
        if candidate_infeasible or (self.check_stop() and not candidate_checked):
            self.info["stop"] = True
            self.info["stop_reason"] = ("candidate_infeasible" if candidate_infeasible
                                        else "collision_threshold")
            return np.zeros((2, 1)), self.info
        self.info["stop"] = False

        action = opt_vel_np[:, 0:1]

        if self.robot.kinematics == 'omni':
            vel = opt_vel_np[:, 0:1]
            vx = vel[0, 0] * cos(vel[1, 0])
            vy = vel[0, 0] * sin(vel[1, 0])
            action = np.array([[vx], [vy]])

            self.info['omni_linear_speed'] = vel[0, 0]
            self.info['omni_orientation'] = vel[1, 0]

        return action, self.info

    def _rollout_controls(self, state, controls, subdivisions=1):
        states = [np.asarray(state[:3], dtype=np.float64).reshape(3, 1).copy()]
        for t in range(controls.shape[1]):
            for _ in range(subdivisions):
                states.append(self.ipath.motion_predict_model(
                    states[-1], controls[:, t:t+1], self.robot.L, self.dt / subdivisions))
        return np.hstack(states)

    def validate_remaining_trajectory(self, state, controls, elapsed, points, velocities=None, with_reason=False):
        """Re-evaluate a previously optimized control horizon with current observations.

        This is a NeuPAN geometry check, not a separate collision-distance policy.
        Callers must independently verify sensor/pose freshness, task identity,
        and authorization; passing this test alone cannot authorize motion.
        """
        state = np.asarray(state, dtype=np.float64)
        controls = np.asarray(controls, dtype=np.float64)
        if (state.shape != (3, 1) or controls.ndim != 2 or controls.shape[0] != 2 or
                not np.isfinite(state).all() or not np.isfinite(controls).all() or
                not np.isfinite(elapsed) or elapsed < 0. or self.dt <= 0.):
            raise ValueError("invalid_trajectory_continuation")
        def outcome(value, reason):
            return (value, reason) if with_reason else value
        if points is None:
            return outcome(None, "missing_scene")  # not an empty, clear scene
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[0] != 2 or not np.isfinite(points).all():
            raise ValueError("invalid_trajectory_obstacles")
        if velocities is not None:
            velocities = np.asarray(velocities, dtype=np.float64)
            if velocities.shape != points.shape or not np.isfinite(velocities).all():
                raise ValueError("invalid_trajectory_obstacle_velocity")
        index = int(elapsed / self.dt)
        if index >= controls.shape[1]:
            return outcome(None, "horizon_expired")  # heartbeat cannot renew a horizon
        remaining = controls[:, index:]
        if not points.shape[1]:
            return outcome((remaining[:, 0].copy(), None), "validated")
        if self.pan.dune_layer is None:
            return outcome(None, "distance_model_unavailable")
        subdivisions = 3
        # The first knot is already partly elapsed. Propagate for exactly its
        # remaining duration before switching control; a full first knot can
        # trace a different and potentially unsafe path around an obstacle.
        first_duration = self.dt - (elapsed - index * self.dt)
        trajectory = [state.copy()]
        times = [0.0]
        for k in range(remaining.shape[1]):
            interval = first_duration if k == 0 else self.dt
            for _ in range(subdivisions):
                trajectory.append(self.ipath.motion_predict_model(
                    trajectory[-1], remaining[:, k:k+1], self.robot.L, interval / subdivisions))
                times.append(times[-1] + interval / subdivisions)
        states = np.hstack(trajectory)
        # Evaluate every observed point, not only the DUNE sampling budget.
        distances = self._candidate_distances(states, points, velocities, sample_times=np.asarray(times))
        feasible = self._candidate_feasibility(distances)[0]
        if not feasible:
            return outcome(None, "obstacle_conflict")
        return outcome((remaining[:, 0].copy(), float(np.min(distances))), "validated")

    def _candidate_distances(self, states, points, velocities=None, sample_dt=None, sample_times=None):
        """Evaluate DUNE's dual distances along an actual dynamics rollout."""
        from neupan.configuration import to_device
        samples = np.asarray(points, dtype=np.float64)
        times = (np.arange(states.shape[1], dtype=np.float64) * (self.dt if sample_dt is None else sample_dt)
                 if sample_times is None else np.asarray(sample_times, dtype=np.float64))
        if times.shape != (states.shape[1],) or not np.isfinite(times).all():
            raise ValueError("invalid_candidate_sample_times")
        predicted = samples[None, :, :]
        if velocities is not None:
            predicted = predicted + times[:, None, None] * np.asarray(velocities)[None, :, :]
        dx = predicted[:, 0, :] - states[0, :, None]
        dy = predicted[:, 1, :] - states[1, :, None]
        c = np.cos(states[2, :])[:, None]
        s = np.sin(states[2, :])[:, None]
        local = np.stack((c * dx + s * dy, -s * dx + c * dy), axis=1)
        local = to_device(torch.as_tensor(local.transpose(1, 0, 2).reshape(2, -1), dtype=torch.float32))
        dune = self.pan.dune_layer
        with torch.no_grad():
            mu = dune.model(local.T).T
            mu = dune.project_dual_feasible(mu)
            distance = dune.cal_objective_distance(mu, local).reshape(states.shape[1], -1)
        return distance.detach().cpu().numpy()

    @staticmethod
    def _candidate_feasibility(distances):
        """Accept clear rollouts or an improving escape from existing overlap.

        Initially clear points must remain clear. For already-overlapping
        points, aggregate model penetration must not increase, and must be
        reduced by the end of the horizon. The only tolerance is float32
        numerical precision, scaled to the magnitude of the computation.
        """
        initial = distances[0]
        inside = initial < 0.0
        eps = np.finfo(np.float32).eps * max(1.0, float(np.max(np.abs(distances)))) * 16.0
        new_overlap = bool(np.any(distances[:, ~inside] < -eps)) if np.any(~inside) else False
        if np.any(inside):
            penetration_by_point = np.maximum(-distances[:, inside], 0.0)
            penetration = penetration_by_point.sum(axis=1)
            # Do not hide a serious worsening at one point behind clearance
            # gained at other points. A transient worsening must be no larger
            # than the net reduction in total penetration at that instant.
            worst_deepening = np.maximum(
                penetration_by_point - penetration_by_point[0], 0.0).max(axis=1)
            net_improvement = np.maximum(penetration[0] - penetration, 0.0)
        else:
            penetration = np.zeros(distances.shape[0])
            worst_deepening = net_improvement = penetration
        escaping = bool(np.any(inside) and not new_overlap and
                        np.all(penetration <= penetration[0] + eps) and
                        np.all(worst_deepening <= net_improvement + eps) and
                        penetration[-1] < penetration[0] - eps)
        feasible = bool(float(np.min(distances)) >= -eps or escaping)
        return (feasible, escaping, float(np.min(initial)),
                float(penetration[0]), float(penetration[-1]))

    def _candidate_cost(self, state, rollout, controls, ref, ref_speed, points, velocities):
        # Evaluate between optimizer knots too, so small obstacles are not
        # skipped when the robot translates or rotates over one solver step.
        subdivisions = 3
        dense = self._rollout_controls(state, controls, subdivisions=subdivisions)
        point_tensor = np_to_tensor(points)
        velocity_tensor = (torch.zeros_like(point_tensor) if velocities is None
                           else np_to_tensor(velocities))
        indices = self.pan.select_obstacle_indices(np_to_tensor(dense), point_tensor,
                    velocity_tensor, self.pan.dune_max_num, self.dt / subdivisions,
                    robot_geometry=(self.robot.G, self.robot.h)).cpu().numpy()
        sampled_velocities = None if velocities is None else velocities[:, indices]
        distances = self._candidate_distances(dense, points[:, indices], sampled_velocities,
                                               sample_dt=self.dt / subdivisions)
        k = min(self.pan.nrmp_max_num, distances.shape[1])
        nearest = np.partition(distances, k - 1, axis=1)[:, :k]
        feasible, escaping, initial_minimum, initial_penetration, final_penetration = (
            self._candidate_feasibility(distances))
        minimum = float(np.min(distances))
        d_min = float(self.pan.nrmp_layer.d_min.detach().cpu().item())
        violation = np.maximum(d_min - nearest, 0.0)
        obstacle_cost = 0.5 * float(self.pan.nrmp_layer.ro_obs) * np.square(violation).sum()
        q = self.pan.nrmp_layer.q_s.detach().cpu().numpy().reshape(-1)
        if q.size == 1:
            q = np.repeat(q, 3)
        delta = rollout - ref
        delta[2] = np.arctan2(np.sin(delta[2]), np.cos(delta[2]))
        tangent = np.vstack((np.cos(ref[2]), np.sin(ref[2])))
        along = np.sum(delta[:2] * tangent, axis=0)
        across = -delta[0] * tangent[1] + delta[1] * tangent[0]
        tracking = np.square(q[0] * along).sum() + np.square(q[1] * across).sum() + np.square(q[2] * delta[2]).sum()
        p = float(self.pan.nrmp_layer.p_u.detach().cpu().item())
        speed = np.square(p * (controls[0] - ref_speed)).sum()
        return (float(tracking + speed + obstacle_cost), minimum, float(obstacle_cost),
                feasible, escaping, initial_minimum, initial_penetration, final_penetration)

    def _avoidance_seed(self, state, original_ref, ref_speed, side, reverse_first=True, retreat_only=False, turn_first=False):
        # Generate an initial hypothesis inside NeuPAN, not a steering command.
        # Its scale follows the configured robot width and optimizer distance.
        extent = float(np.ptp(np.asarray(self.robot.vertices)[1, :]))
        d_max = float(self.pan.nrmp_layer.d_max.detach().cpu().item())
        amplitude = extent + d_max
        phase = np.linspace(0.0, np.pi, self.T + 1)
        ref = original_ref.copy()
        ref[0] -= side * amplitude * np.sin(phase) * np.sin(original_ref[2])
        ref[1] += side * amplitude * np.sin(phase) * np.cos(original_ref[2])
        ref[2] += side * amplitude * np.sin(phase)
        ref[2, 0] = original_ref[2, 0]
        # Gear changes rewrite path metadata; infer goal direction from path
        # geometry so a temporary reverse does not invert the bypass itself.
        route = original_ref[:2, -1] - np.asarray(state[:2, 0])
        heading = np.array([np.cos(state[2, 0]), np.sin(state[2, 0])])
        alignment = float(np.dot(route, heading))
        reference_sign = (-1.0 if alignment < 0.0 else 1.0)
        linear = min(abs(float(self.ref_speed)), float(self.robot.max_speed[0, 0]))
        angular = float(self.robot.max_speed[1, 0])
        controls = np.zeros((2, self.T), dtype=np.float64)
        controls[0, :] = reference_sign * linear * 0.7
        if reverse_first or retreat_only:
            controls[0, :max(1, self.T // 5)] = -reference_sign * linear * 0.4
        controls[1, :] = side * angular
        candidate_speed = np.full_like(ref_speed, reference_sign * linear)
        if retreat_only:
            controls[0, :] = -reference_sign * linear * 0.4
            controls[1, :] = side * angular * 0.4
            ref = self._rollout_controls(state, controls)
            candidate_speed = np.full_like(ref_speed, -reference_sign * linear * 0.4)
        elif turn_first:
            controls[0, :max(1, self.T // 3)] = 0.0
            ref = self._rollout_controls(state, controls)
            candidate_speed = controls[0].copy()
        return self._rollout_controls(state, controls), controls, ref, candidate_speed

    def _solve_candidates(self, state, nominal, points, velocities, measured_velocity):
        from diffcp.cone_program import SolverError
        nom_s, nom_u, ref_s, ref_us = nominal
        measured = None if measured_velocity is None else np_to_tensor(measured_velocity)
        obstacle = np_to_tensor(points)
        velocity = None if velocities is None else np_to_tensor(velocities)
        ref_dist = self._candidate_distances(ref_s, points, velocities)
        d_max = float(self.pan.nrmp_layer.d_max.detach().cpu().item())
        explore = float(np.min(ref_dist)) < d_max
        observed_v = 0.0 if measured_velocity is None else float(np.asarray(measured_velocity).reshape(-1)[0])
        previous_side = self.preferred_avoidance_side
        reverse_first = not (previous_side in (-1, 1) and observed_v >= 0.0)
        queue = [(0, None, nom_s, nom_u, ref_s, ref_us)]
        seen = set()

        def enqueue(side, mode):
            key = (side, mode)
            if key not in seen:
                seen.add(key)
                seed_s, seed_u, seed_ref, seed_speed = self._avoidance_seed(
                    state, ref_s, ref_us, side, reverse_first=(mode == "reverse"),
                    retreat_only=(mode == "retreat"), turn_first=(mode == "turn"))
                queue.append((side, mode, seed_s, seed_u, seed_ref, seed_speed))

        initial_mode = "reverse" if reverse_first else "forward"
        if previous_side in (-1, 1) and self.preferred_avoidance_mode is not None:
            initial_mode = self.preferred_avoidance_mode
        if explore:
            # Compare both initial longitudinal directions in the same solve.
            # The execution gear is not evidence that a direction is blocked.
            sides = ((previous_side,) if previous_side in (-1, 1) else (1, -1))
            for side in sides:
                enqueue(side, "forward")
                enqueue(side, "reverse")
        # Try the other seed direction or bypass side when the first choice
        # is not feasible. These are optimization initializations only.
        first_side = previous_side if previous_side in (-1, 1) else 1
        alternate_mode = "forward" if initial_mode == "reverse" else "reverse"
        fallbacks = [(first_side, alternate_mode), (first_side, "turn"),
                     (first_side, "retreat"), (-first_side, initial_mode),
                     (-first_side, "turn"), (-first_side, alternate_mode),
                     (-first_side, "retreat")]
        candidates, last_error = [], None
        route_delta = ref_s[:2, -1] - ref_s[:2, 0]
        route_length = float(np.linalg.norm(route_delta))
        route_direction = (route_delta / route_length if route_length > 1e-9
                           else np.array([np.cos(state[2, 0]), np.sin(state[2, 0])]))
        footprint_width = float(np.ptp(np.asarray(self.robot.vertices)[1, :]))
        useful_progress = min(footprint_width, route_length * 0.25)
        index = 0
        fallback_added = False
        while index < len(queue):
            side, mode, seed_s, seed_u, seed_ref, seed_speed = queue[index]
            index += 1
            try:
                result = self.pan(np_to_tensor(seed_s), np_to_tensor(seed_u),
                                  np_to_tensor(seed_ref), np_to_tensor(seed_speed),
                                  obstacle, velocity, measured)
                planned, controls, _ = result
                control_np = tensor_to_np(controls)
                planned_np = tensor_to_np(planned)
                if np.isfinite(control_np).all() and np.isfinite(planned_np).all():
                    rollout = self._rollout_controls(state, control_np)
                    (cost, minimum, obstacle_cost, feasible, escaping, initial_minimum,
                     initial_penetration, final_penetration) = self._candidate_cost(
                        state, rollout, control_np, ref_s, ref_us, points, velocities)
                    if np.isfinite(cost) and np.isfinite(minimum):
                        linearization_error = float(np.max(np.linalg.norm(
                            rollout[:2] - planned_np[:2], axis=0)))
                        # Expose the dynamically rolled trajectory that was
                        # evaluated, not the linearized intermediate states.
                        evaluated_result = (np_to_tensor(rollout), controls, result[2])
                        progress = float(np.dot(rollout[:2, -1] - rollout[:2, 0], route_direction))
                        step_bound = np.asarray(self.robot.acce_bound).reshape(2)
                        first_effort = float(np.max(np.abs(control_np[:, 0]) /
                                                    np.maximum(step_bound, 1e-9)))
                        candidates.append(dict(side=side, mode=mode, result=evaluated_result,
                            cost=cost, minimum=minimum, obstacle_cost=obstacle_cost,
                            feasible=feasible, escaping=escaping, initial_minimum=initial_minimum,
                            initial_penetration=initial_penetration, final_penetration=final_penetration,
                            progress=progress, first_effort=first_effort,
                            linearization_error=linearization_error,
                            dune_min=self.pan.dune_layer.min_distance,
                            dune_points=self.pan.dune_layer.obstacle_points,
                            nrmp_points=self.pan.nrmp_layer.obstacle_points,
                            nom_values=self.pan.current_nom_values))
            except SolverError as error:
                last_error = error
            # A reference may appear clear while its dynamically feasible
            # rollout approaches an obstacle. Also try alternate seeds after
            # a failed nominal solve instead of immediately aborting.
            if not explore and index == len(queue):
                explore = (not candidates or candidates[-1]["minimum"] < 0.0)
            # A trajectory that merely waits in front of the obstacle is not
            # a useful bypass. Try alternative seeds if every feasible candidate
            # makes little progress along the original route horizon.
            viable_progress = any(c["feasible"] and c["progress"] >= useful_progress
                                  and c["first_effort"] > 0.05 for c in candidates)
            if explore and index == len(queue) and not viable_progress and not fallback_added:
                # Keep planning within the command freshness window. Search
                # one additional hypothesis per cycle and rotate alternatives
                # across cycles rather than blocking on every possible seed.
                for offset in range(len(fallbacks)):
                    fallback_side, fallback_mode = fallbacks[(self.candidate_search_cursor + offset) % len(fallbacks)]
                    if (fallback_side, fallback_mode) not in seen:
                        enqueue(fallback_side, fallback_mode)
                        fallback_added = True
                        break
        if not candidates:
            if last_error is not None:
                raise last_error
            raise RuntimeError("NeuPAN candidate optimization produced no finite result")
        nonoverlap = [candidate for candidate in candidates if candidate["feasible"]]
        best_progress = max((candidate["progress"] for candidate in nonoverlap), default=0.0)
        progress_goal = max(useful_progress, best_progress * 0.8)
        progressing = [candidate for candidate in nonoverlap
                       if candidate["progress"] >= progress_goal and candidate["first_effort"] > 0.05] if explore else []
        moving = [candidate for candidate in nonoverlap if candidate["first_effort"] > 0.05]
        pool = progressing or moving or nonoverlap or candidates
        best = min(pool, key=lambda candidate: candidate["cost"])
        previous = next((candidate for candidate in pool
                         if candidate["side"] == previous_side and candidate["side"] != 0), None)
        if previous is not None and previous["cost"] <= best["cost"] + abs(best["cost"]) * 0.1:
            best = previous
        if best["side"]:
            self.preferred_avoidance_side = best["side"]
            self.preferred_avoidance_mode = best["mode"]
        elif not explore:
            self.preferred_avoidance_side = 0
            self.preferred_avoidance_mode = None
        self.info["candidate_infeasible"] = not bool(nonoverlap)
        if explore and not progressing:
            self.candidate_search_cursor += 1
        else:
            self.candidate_search_cursor = 0
        self.pan.dune_layer.min_distance = best["dune_min"]
        self.pan.dune_layer.obstacle_points = best["dune_points"]
        self.pan.nrmp_layer.obstacle_points = best["nrmp_points"]
        self.pan.current_nom_values = best["nom_values"]
        self.info["avoidance_candidates"] = [
            dict(side=c["side"], seed_mode=c["mode"], cost=c["cost"],
                 min_distance=c["minimum"], obstacle_cost=c["obstacle_cost"],
                 feasible=c["feasible"], escaping=c["escaping"], initial_minimum=c["initial_minimum"],
                 initial_penetration=c["initial_penetration"], final_penetration=c["final_penetration"],
                 progress=c["progress"], first_effort=c["first_effort"],
                 linearization_error=c["linearization_error"], selected=c is best)
            for c in candidates]
        self.info["avoidance_side"] = best["side"]
        return best["result"]

    def check_stop(self):
        return self.min_distance < self.collision_threshold
    

    def scan_to_point(
        self,
        state: np.ndarray,
        scan: dict,
        scan_offset: list[float] = [0, 0, 0],
        angle_range: list[float] = [-np.pi, np.pi],
        down_sample: int = 1,
    ) -> np.ndarray | None:
        
        """
        input:
            state: [x, y, theta]
            scan: {}
                ranges: list[float], the range of the scan
                angle_min: float, the minimum angle of the scan
                angle_max: float, the maximum angle of the scan
                range_max: float, the maximum range of the scan
                range_min: float, the minimum range of the scan

            scan_offset: [x, y, theta], the relative position of the sensor to the robot state coordinate

        return point cloud: (2, n)
        """
        point_cloud = []

        ranges = np.array(scan["ranges"])
        angles = np.linspace(scan["angle_min"], scan["angle_max"], len(ranges))

        for i in range(len(ranges)):
            scan_range = ranges[i]
            angle = angles[i]

            if scan_range < (scan["range_max"] - 0.02) and scan_range > scan["range_min"]:
                if angle > angle_range[0] and angle < angle_range[1]:
                    point = np.array(
                        [[scan_range * cos(angle)], [scan_range * sin(angle)]]
                    )
                    point_cloud.append(point)

        if len(point_cloud) == 0:
            return None

        point_array = np.hstack(point_cloud)
        s_trans, s_R = get_transform(np.c_[scan_offset])
        temp_points = s_R @ point_array + s_trans

        trans, R = get_transform(state)
        points = (R @ temp_points + trans)[:, ::down_sample]

        return points

    def scan_to_point_velocity(
        self,
        state,
        scan,
        scan_offset=[0, 0, 0],
        angle_range=[-np.pi, np.pi],
        down_sample=1,
    ):
        """
        input:
            state: [x, y, theta]
            scan: {}
                ranges: list[float], the ranges of the scan
                angle_min: float, the minimum angle of the scan
                angle_max: float, the maximum angle of the scan
                range_max: float, the maximum range of the scan
                range_min: float, the minimum range of the scan
                velocity: list[float], the velocity of the scan

            scan_offset: [x, y, theta], the relative position of the sensor to the robot state coordinate

        return point cloud: (2, n)
        """
        point_cloud = []
        velocity_points = []

        ranges = np.array(scan["ranges"])
        angles = np.linspace(scan["angle_min"], scan["angle_max"], len(ranges))
        scan_velocity = scan.get("velocity", np.zeros((2, len(ranges))))

        # lidar_state = self.lidar_state_transform(state, np.c_[self.lidar_offset])
        for i in range(len(ranges)):
            scan_range = ranges[i]
            angle = angles[i]

            if scan_range < (scan["range_max"] - 0.02) and scan_range >= scan["range_min"]:
                if angle > angle_range[0] and angle < angle_range[1]:
                    point = np.array(
                        [[scan_range * cos(angle)], [scan_range * sin(angle)]]
                    )
                    point_cloud.append(point)
                    velocity_points.append(scan_velocity[:, i : i + 1])

        if len(point_cloud) == 0:
            return None, None

        point_array = np.hstack(point_cloud)
        s_trans, s_R = get_transform(np.c_[scan_offset])
        temp_points = s_R.T @ (
            point_array - s_trans
        )  # points in the robot state coordinate

        trans, R = get_transform(state)
        points = (R @ temp_points + trans)[:, ::down_sample]

        velocity = np.hstack(velocity_points)[:, ::down_sample]

        return points, velocity


    def train_dune(self):
        self.pan.dune_layer.train_dune(self.dune_train_kwargs)


    def reset(self):
        self.ipath.point_index = 0
        self.ipath.curve_index = 0
        self.ipath.arrive_flag = False
        self.info["stop"] = False
        self.info["arrive"] = False
        self.cur_vel_array = np.zeros_like(self.cur_vel_array)
        self.preferred_avoidance_side = 0
        self.preferred_avoidance_mode = None
        self.candidate_search_cursor = 0

    def set_initial_path(self, path):

        '''
        set the initial path from the given path
        path: list of [x, y, theta, gear] 4x1 vector, gear is -1 (back gear) or 1 (forward gear)
        '''

        self.info["arrive"] = False
        self.ipath.set_initial_path(path)

    def set_initial_path_from_state(self, state):
        """
        Args:
            states: [x, y, theta] or 3x1 vector

        """
        self.ipath.init_check(state)
    
    def set_reference_speed(self, speed: float):

        """
        Args:
            speed: float, the reference speed of the robot
        """

        self.ipath.ref_speed = speed
        self.ref_speed = speed
    
    def update_initial_path_from_goal(self, start, goal):

        """
        Args:
            start: [x, y, theta] or 3x1 vector
            goal: [x, y, theta] or 3x1 vector
        """

        self.ipath.update_initial_path_from_goal(start, goal)


    def update_initial_path_from_waypoints(self, waypoints):

        """
        Args:
            waypoints: list of [x, y, theta] or 3x1 vector
        """

        self.ipath.set_ipath_with_waypoints(waypoints)


    def update_adjust_parameters(self, **kwargs):

        """
        update the adjust parameters value: q_s, p_u, eta, d_max, d_min

        Args:
            q_s: float or list[float], the weight of the state cost. Can be a scalar or a 3-element list for x, y, theta dimensions.
                 Note: The q_s type must be aligned with the initial dimension. If initialized as scalar, only scalar updates are allowed;
                 if initialized as vector, only 3-element vector updates are allowed. Re-initialize the planner to switch types.
            p_u: float, the weight of the speed cost
            eta: float, the weight of the collision avoidance cost
            d_max: float, the maximum distance to the obstacle
            d_min: float, the minimum distance to the obstacle
        """

        self.pan.nrmp_layer.update_adjust_parameters_value(**kwargs)

    @property
    def min_distance(self):
        return self.pan.min_distance
    
    @property
    def dune_points(self):
        return self.pan.dune_points
    
    @property
    def nrmp_points(self):
        return self.pan.nrmp_points
    
    @property
    def initial_path(self):
        return self.ipath.initial_path
    
    @property
    def adjust_parameters(self):
        return self.pan.nrmp_layer.adjust_parameters
    
    @property
    def waypoints(self):

        '''
        Waypoints for generating the initial path
        '''

        return self.ipath.waypoints
    
    @property
    def opt_trajectory(self):

        '''
        MPC receding horizon trajectory under the velocity sequence
        return a list of state sequence, each state is a 3x1 vector
        '''

        return self.info["opt_state_list"]
    
    @property
    def ref_trajectory(self):

        '''
        Reference trajectory on the initial path
        return a list of state sequence, each state is a 3x1 vector
        '''

        return self.info["ref_state_list"]

    

    
