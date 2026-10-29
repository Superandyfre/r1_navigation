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
import time
import torch
from neupan.robot import robot
from neupan.blocks import InitialPath, PAN
from neupan import configuration
from neupan.util import time_it, file_check, get_transform
import numpy as np
import torch
from diffcp.cone_program import SolverError
from neupan.configuration import np_to_tensor, tensor_to_np
from neupan.bypass import (
    make_bypass_guide, seed_controls, seed_reverse_controls, trajectory_clearance,
    BypassRecovery,
    make_route_rejoin_guide,
)
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
        self.bypass_enabled = bool(kwargs.get("bypass_enabled", False))
        self.bypass_lateral_scale = float(kwargs.get("bypass_lateral_scale", 0.20))
        self.bypass_min_clearance = float(kwargs.get("bypass_min_clearance", 0.05))
        if not 0.0 < self.bypass_lateral_scale <= 1.0 or self.bypass_min_clearance < 0.0:
            raise ValueError("invalid NeuPAN bypass configuration")
        self.bypass_side = 0
        self.bypass_invalid_cycles = 0
        self.bypass_active = False
        self.bypass_guide = None
        self.bypass_completed = False
        self.bypass_recovery = BypassRecovery()

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
        if self.bypass_enabled and not (
            self.robot.path_relative_cost and self.robot.geometry_collision_cost
        ):
            raise ValueError("bypass requires path-relative and geometric obstacle costs")

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
                reference_speed_profile=None, measured_velocity=None,
                bypass_mode=False, bypass_route_clear=False):
        """
        state: current state of the robot, matrix (3, 1), x, y, theta
        points: current input obstacle point positions, matrix (2, N), N is the number of obstacle points.
        velocities: current velocity of each obstacle point, matrix (2, N), N is the number of obstacle points. vx, vy
        """

        assert state.shape[0] >= 3
        self.info.update(bypass_candidates=[], bypass_recovery_exhausted=False,
                         bypass_rebuilds=self.bypass_recovery.rebuilds,
                         stop_reason="none")

        if self.bypass_completed:
            self.info["arrive"] = True
            self.info["stop"] = False
            self.info["bypass_valid"] = False
            self.info["bypass_phase"] = "done"
            return np.zeros((2, 1)), self.info

        if self.bypass_guide is None and self.ipath.check_arrive(state):
            self.info["arrive"] = True
            self.info["bypass_valid"] = False
            self.info["bypass_phase"] = "done" if self.bypass_guide is not None else "none"
            return np.zeros((2, 1)), self.info

        if (self.bypass_guide is not None and bypass_mode
                and (points is None or points.shape[1] == 0)):
            self.info["stop"] = True
            self.info["bypass_valid"] = False
            self.info["bypass_phase"] = self.bypass_guide.phase
            return np.zeros((2, 1)), self.info

        if not bypass_mode:
            self.bypass_completed = False
        use_bypass = (self.bypass_enabled and bypass_mode and not self.bypass_completed
                      and points is not None and points.shape[1] > 0)
        if not use_bypass:
            self.bypass_recovery = BypassRecovery()
            if self.bypass_active:
                self.cur_vel_array = np.zeros_like(self.cur_vel_array)
                self.pan.current_nom_values = [None, None, None, None]
            self.bypass_side = 0
            self.bypass_invalid_cycles = 0
            self.bypass_guide = None
        self.bypass_active = use_bypass
        # The route speed limit can reach zero at an obstacle on the original
        # path. Bypass candidates need an unconstrained solve before their own
        # swept geometry can be checked; no candidate is published here.
        if use_bypass and self.bypass_guide is not None:
            footprint_radius = float(np.max(np.linalg.norm(self.robot.vertices, axis=0)))
            margin = max(self.bypass_min_clearance, self.robot.geometry_margin_m)
            self.bypass_guide.update(state, footprint_radius, margin)
            if self.bypass_guide.phase == "done":
                self.bypass_completed = True
                self.bypass_guide = None
                self.info["arrive"] = True
                self.info["stop"] = False
                self.info["bypass_valid"] = False
                self.info["bypass_phase"] = "done"
                return np.zeros((2, 1)), self.info
        planning_profile = None if use_bypass else reference_speed_profile
        nom_input_np = self.ipath.generate_nom_ref_state(
            state, self.cur_vel_array, self.ref_speed, planning_profile
        )
        rejoined = False
        rejoin_guide = None
        if use_bypass and bypass_route_clear:
            route = np.hstack(self.ipath.initial_path)[:3]
            clearance = trajectory_clearance(
                nom_input_np[2], points, self.robot.vertices, self.robot.G, self.robot.h)
            rejoined = self.bypass_recovery.can_rejoin(
                time.monotonic(), True, state, route, clearance,
                self.bypass_min_clearance)
            if not rejoined and (self.bypass_guide is None or
                                 self.bypass_guide.phase != "route_rejoin"):
                goal_pose = np.asarray(self.ipath.initial_path[-1]).reshape(-1)[:3]
                rejoin_guide = make_route_rejoin_guide(
                    state, route, points, self.robot.vertices, self.robot.G, self.robot.h,
                    self.bypass_min_clearance, goal_pose)
        elif use_bypass:
            self.bypass_recovery.clear_since = None
            if self.bypass_guide is not None and self.bypass_guide.phase == "route_rejoin":
                self.bypass_guide = None
        if rejoin_guide is not None:
            self.bypass_guide = rejoin_guide
        if rejoined:
            use_bypass = False
            self.bypass_active = False
            self.bypass_guide = None
            self.bypass_side = 0
            self.bypass_invalid_cycles = 0
            self.bypass_recovery = BypassRecovery()
            self.cur_vel_array = np.zeros_like(self.cur_vel_array)
            self.pan.current_nom_values = [None, None, None, None]
            nom_input_np = self.ipath.generate_nom_ref_state(
                state, self.cur_vel_array, self.ref_speed, reference_speed_profile)

        # convert to tensor
        nom_input_tensor = [np_to_tensor(n) for n in nom_input_np]
        obstacle_points_tensor = np_to_tensor(points) if points is not None else None
        point_velocities_tensor = (
            np_to_tensor(velocities) if velocities is not None else None
        )

        measured_velocity_tensor = (
            np_to_tensor(measured_velocity) if measured_velocity is not None else None
        )
        speed_limits_tensor = (
            np_to_tensor(np.asarray(reference_speed_profile, dtype=np.float64))
            if planning_profile is not None else None
        )
        if use_bypass:
            nominal_state, nominal_control, reference_state, reference_speed = nom_input_tensor
            candidates = []
            solver_failures = 0
            footprint_radius = float(np.max(np.linalg.norm(self.robot.vertices, axis=0)))
            margin = max(self.bypass_min_clearance, self.robot.geometry_margin_m)
            if len(self.ipath.waypoints):
                goal_index = -2 if self.ipath.loop else -1
                goal_pose = np.asarray(self.ipath.waypoints[goal_index]).reshape(-1)[:3]
            else:
                goal_pose = np.asarray(self.ipath.initial_path[-1]).reshape(-1)[:3]
            guides = ([self.bypass_guide] if self.bypass_guide is not None
                      else [make_bypass_guide(state, points, nom_input_np[2], side,
                                              footprint_radius, margin, goal_pose,
                                              self.robot.vertices, self.robot.G, self.robot.h)
                            for side in (1, -1)])
            guides = [guide for guide in guides if guide is not None]
            if self.bypass_recovery.exhausted:
                guides = []

            def solve_candidate(seed_state, seed_control, guide, min_speed):
                nonlocal solver_failures
                self.pan.current_nom_values = [None, None, None, None]
                guided_reference = np_to_tensor(guide.reference(
                    state, self.T, footprint_radius, margin
                ))
                heading_error = float(np.arctan2(
                    np.sin(guided_reference[2, -1].item() - float(state[2, 0])),
                    np.cos(guided_reference[2, -1].item() - float(state[2, 0])),
                ))
                turn_before_pass = (
                    (guide.phase in ("pass", "align") and abs(heading_error) > 0.20)
                    or (guide.phase == "rejoin" and abs(heading_error) > 0.40)
                    or (guide.phase == "final" and abs(heading_error) > 0.40)
                    or (guide.phase == "route_rejoin" and abs(heading_error) > 0.40)
                )
                guided_speed = (
                    torch.zeros_like(reference_speed) if turn_before_pass else reference_speed
                )
                if guide.phase == "backoff":
                    guided_speed = torch.full_like(reference_speed, -0.10)
                if guide.phase in ("rejoin", "final", "route_rejoin"):
                    guided_speed = torch.clamp(guided_speed, max=0.10)
                speed_cap = (
                    torch.zeros((self.T,), dtype=nominal_state.dtype)
                    if turn_before_pass else None
                )
                if guide.phase in ("rejoin", "final", "route_rejoin") and not turn_before_pass:
                    speed_cap = torch.full((self.T,), 0.12, dtype=nominal_state.dtype)
                try:
                    result = self.pan(
                        seed_state, seed_control, guided_reference, guided_speed,
                        obstacle_points_tensor, point_velocities_tensor,
                        measured_velocity_tensor, speed_cap,
                        self.bypass_lateral_scale if guide.phase == "side" else 1.0, True,
                        min_speed,
                    )
                except SolverError:
                    solver_failures += 1
                    self.info["bypass_candidates"].append(
                        {"phase": guide.phase, "side": guide.side, "reason": "solver_error"})
                    return False
                predicted = tensor_to_np(result[0])
                controls = tensor_to_np(result[1])
                clearance = trajectory_clearance(
                    predicted, points, self.robot.vertices, self.robot.G, self.robot.h
                )
                movement = predicted[:2, -1] - predicted[:2, 0]
                along = float(guide.tangent @ movement)
                lateral = float(guide.normal @ movement)
                heading_change = float(np.arctan2(
                    np.sin(predicted[2, -1] - predicted[2, 0]),
                    np.cos(predicted[2, -1] - predicted[2, 0]),
                ))
                first_moves = abs(float(controls[0, 0])) >= 0.015 or abs(float(controls[1, 0])) >= 0.04
                if measured_velocity is not None:
                    measured = np.asarray(measured_velocity).reshape(-1)
                    first_moves = first_moves or (
                        abs(float(measured[1])) >= 0.04
                        and abs(float(controls[1, 0])) < abs(float(measured[1])) - 0.02
                    )
                if turn_before_pass:
                    next_error = abs(float(np.arctan2(
                        np.sin(guided_reference[2, -1].item() - predicted[2, -1]),
                        np.cos(guided_reference[2, -1].item() - predicted[2, -1]))))
                    progress = .15 * (abs(heading_error) - next_error)
                elif guide.phase == "backoff":
                    progress = -along
                elif guide.phase == "side":
                    progress = guide.side * (lateral + 0.15 * heading_change)
                elif guide.phase in ("pass", "align"):
                    heading_error = abs(float(np.arctan2(
                        np.sin(predicted[2, 0] - guided_reference[2, -1].item()),
                        np.cos(predicted[2, 0] - guided_reference[2, -1].item()),
                    )))
                    next_heading_error = abs(float(np.arctan2(
                        np.sin(predicted[2, -1] - guided_reference[2, -1].item()),
                        np.cos(predicted[2, -1] - guided_reference[2, -1].item()),
                    )))
                    progress = (0.0 if guide.phase == "align" else along) + 0.15 * (
                        heading_error - next_heading_error
                    )
                elif guide.phase in ("final", "route_rejoin"):
                    target = guide.route_target if guide.phase == "route_rejoin" else guide.goal_pose
                    start_distance = float(np.linalg.norm(
                        predicted[:2, 0] - target[:2]
                    ))
                    end_distance = float(np.linalg.norm(
                        predicted[:2, -1] - target[:2]
                    ))
                    start_yaw_error = abs(float(np.arctan2(
                        np.sin(predicted[2, 0] - target[2]),
                        np.cos(predicted[2, 0] - target[2]),
                    )))
                    end_yaw_error = abs(float(np.arctan2(
                        np.sin(predicted[2, -1] - target[2]),
                        np.cos(predicted[2, -1] - target[2]),
                    )))
                    progress = (start_distance - end_distance) + 0.10 * (
                        start_yaw_error - end_yaw_error
                    )
                else:
                    if turn_before_pass:
                        start_error = abs(heading_error)
                        end_error = abs(float(np.arctan2(
                            np.sin(guided_reference[2, -1].item() - predicted[2, -1]),
                            np.cos(guided_reference[2, -1].item() - predicted[2, -1]),
                        )))
                        progress = 0.15 * (start_error - end_error)
                    else:
                        progress = along - guide.side * 0.5 * lateral
                valid = (
                    np.isfinite(predicted).all() and np.isfinite(controls).all()
                    and clearance >= self.bypass_min_clearance
                    and first_moves
                    and progress > (0.005 if turn_before_pass else
                                    0.01 if guide.phase in ("final", "route_rejoin") else 0.05)
                )
                reason = ("nonfinite" if not (np.isfinite(predicted).all() and np.isfinite(controls).all())
                          else "clearance" if clearance < self.bypass_min_clearance
                          else "first_motion" if not first_moves
                          else "progress" if not valid else "accepted")
                self.info["bypass_candidates"].append({
                    "phase": guide.phase, "side": guide.side, "reason": reason,
                    "clearance_m": float(clearance) if np.isfinite(clearance) else None,
                    "progress": float(progress) if np.isfinite(progress) else None,
                    "linear_x": float(controls[0, 0]) if np.isfinite(controls[0, 0]) else None,
                    "angular_z": float(controls[1, 0]) if np.isfinite(controls[1, 0]) else None,
                })
                if valid:
                    score = progress + 0.25 * min(clearance, 0.20)
                    score -= 0.10 * abs(float(result[1][0, 0])) if result[1][0, 0] < 0 else 0.0
                    candidates.append((score, guide, clearance, result, guided_reference))
                return valid

            reverse_in_motion = (
                measured_velocity is not None
                and float(np.asarray(measured_velocity).reshape(-1)[0]) < -0.015
            )
            warm_valid = bool(self.bypass_guide is not None and guides and solve_candidate(
                nominal_state, nominal_control, guides[0],
                -float(self.robot.max_speed[0, 0])
                if guides[0].phase == "backoff" or reverse_in_motion else 0.0,
            ))
            if not warm_valid:
                self.bypass_invalid_cycles += 1
                seeds = []
                for guide in guides:
                    if guide.phase == "backoff":
                        seed_state, seed_control = seed_reverse_controls(
                            state, self.T, self.dt, min(self.ref_speed, 0.10)
                        )
                    else:
                        seed_state, seed_control = seed_controls(
                            state, self.T, self.dt, guide.side,
                            float(self.robot.max_speed[1, 0]),
                            min(self.ref_speed, 0.12),
                        )
                    seeds.append((np_to_tensor(seed_state), np_to_tensor(seed_control), guide))
                for seed_state, seed_control, guide in seeds:
                    solve_candidate(
                        seed_state, seed_control, guide,
                        -float(self.robot.max_speed[0, 0])
                        if guide.phase == "backoff" or reverse_in_motion else 0.0,
                    )
                if not candidates:
                    for seed_state, seed_control, guide in seeds:
                        if guide.phase != "backoff":
                            solve_candidate(
                                seed_state, seed_control, guide,
                                -float(self.robot.max_speed[0, 0]),
                            )
            else:
                self.bypass_invalid_cycles = 0
            if candidates:
                self.bypass_recovery.accepted()
                _, selected_guide, selected_clearance, result, guided_reference = max(
                    candidates, key=lambda item: item[0]
                )
                self.bypass_guide = selected_guide
                self.bypass_side = selected_guide.side
                self.bypass_invalid_cycles = 0
                opt_state_tensor, opt_vel_tensor, opt_distance_tensor = result
                self.info["bypass_valid"] = True
                self.info["bypass_side"] = selected_guide.side
                self.info["bypass_clearance_m"] = selected_clearance
                self.info["bypass_phase"] = selected_guide.phase
            else:
                if self.bypass_recovery.rejected(time.monotonic()):
                    self.bypass_guide = None
                    self.cur_vel_array = np.zeros_like(self.cur_vel_array)
                opt_state_tensor = nominal_state
                opt_vel_tensor = torch.zeros_like(nominal_control)
                opt_distance_tensor = torch.zeros((1, self.T), dtype=nominal_state.dtype)
                self.info["bypass_valid"] = False
                self.info["bypass_side"] = self.bypass_side
                self.info["bypass_clearance_m"] = None
                self.info["bypass_phase"] = self.bypass_guide.phase if self.bypass_guide else "none"
            self.info["bypass_solver_failures"] = solver_failures
            self.info["bypass_rebuilds"] = self.bypass_recovery.rebuilds
            self.info["bypass_recovery_exhausted"] = self.bypass_recovery.exhausted
        else:
            opt_state_tensor, opt_vel_tensor, opt_distance_tensor = self.pan(
                *nom_input_tensor, obstacle_points_tensor, point_velocities_tensor,
                measured_velocity_tensor, speed_limits_tensor,
            )
            self.info["bypass_valid"] = False
            self.info["bypass_side"] = 0
            self.info["bypass_clearance_m"] = None
            self.info["bypass_phase"] = "rejoined" if rejoined else "done" if self.bypass_completed else "none"
            self.info["bypass_solver_failures"] = 0

        opt_state_np, opt_vel_np = tensor_to_np(opt_state_tensor), tensor_to_np(
            opt_vel_tensor
        )

        self.cur_vel_array = opt_vel_np

        self.info["state_tensor"] = opt_state_tensor
        self.info["vel_tensor"] = opt_vel_tensor
        self.info["distance_tensor"] = opt_distance_tensor
        self.info['ref_state_tensor'] = guided_reference if use_bypass and candidates else nom_input_tensor[2]
        self.info['ref_speed_tensor'] = nom_input_tensor[3]

        self.info["ref_state_list"] = [
            state[:, np.newaxis] for state in nom_input_np[2].T
        ]
        self.info["opt_state_list"] = [state[:, np.newaxis] for state in opt_state_np.T]

        if (use_bypass and not self.info["bypass_valid"]) or self.check_stop():
            self.info["stop"] = True
            self.info["stop_reason"] = ("bypass_recovery_exhausted" if self.bypass_recovery.exhausted
                else "bypass_no_valid_candidate" if use_bypass and not self.info["bypass_valid"]
                else "collision_threshold")
            return np.zeros((2, 1)), self.info
        else:
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
        self.bypass_recovery = BypassRecovery()
        self.ipath.point_index = 0
        self.ipath.curve_index = 0
        self.ipath.arrive_flag = False
        self.info["stop"] = False
        self.info["arrive"] = False
        self.cur_vel_array = np.zeros_like(self.cur_vel_array)
        self.bypass_side = 0
        self.bypass_invalid_cycles = 0
        self.bypass_active = False
        self.bypass_guide = None
        self.bypass_completed = False

    def set_initial_path(self, path):

        '''
        set the initial path from the given path
        path: list of [x, y, theta, gear] 4x1 vector, gear is -1 (back gear) or 1 (forward gear)
        '''

        self.bypass_recovery = BypassRecovery()
        self.bypass_guide = None
        self.bypass_side = 0
        self.bypass_completed = False
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

    

    
