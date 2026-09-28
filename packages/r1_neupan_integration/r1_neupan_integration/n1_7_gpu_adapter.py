"""Opt-in N1.7 Moreau adapter for the frozen NeuPAN core.

This module deliberately monkey-patches only the live Python process used by
the N1.7 experiment.  It does not modify the N0 source tree or the N1/N1.6
runtime.  The adapter preserves the CVXPY problem, its variables, parameters,
objective, and constraints; it changes only CvxpyLayer construction from the
legacy diffcp/ECOS call path to CVXPYLayers' documented ``solver='MOREAU'``
backend and removes the legacy ``solve_method`` keyword from the call.
"""

from __future__ import annotations

from typing import Any


def install_moreau_adapter() -> None:
    """Install the process-local Moreau CvxpyLayer adapter exactly once."""
    from cvxpylayers.torch import CvxpyLayer
    from neupan.blocks.nrmp import NRMP
    from neupan.configuration import tensor_dtype, to_device

    if getattr(NRMP, "_n1_7_moreau_adapter", False):
        return

    def problem_definition_moreau(self: Any) -> None:
        problem = self.construct_prob()
        self.nrmp_layer = to_device(
            CvxpyLayer(
                problem,
                parameters=self.para_list,
                variables=self.indep_list,
                solver="MOREAU",
            )
        )

    def forward_moreau(
        self: Any,
        nom_s: Any,
        nom_u: Any,
        ref_s: Any,
        ref_us: Any,
        mu_list: Any = None,
        lam_list: Any = None,
        point_list: Any = None,
    ) -> tuple[Any, Any, Any]:
        if point_list:
            self.obstacle_points = point_list[0][:, : self.max_num]
        parameter_values = self.generate_parameter_value(
            nom_s, nom_u, ref_s, ref_us, mu_list, lam_list, point_list
        )
        # Moreau is selected at layer construction.  The upstream
        # ``solve_method=ECOS`` keyword is specific to the diffcp path.
        solutions = self.nrmp_layer(*parameter_values)
        opt_solution_state = solutions[0].type(tensor_dtype)
        opt_solution_vel = solutions[1].type(tensor_dtype)
        nom_d = None if self.no_obs else solutions[2].type(tensor_dtype)
        return opt_solution_state, opt_solution_vel, nom_d

    NRMP.problem_definition = problem_definition_moreau
    NRMP.forward = forward_moreau
    NRMP._n1_7_moreau_adapter = True


def install_cuclarabel_adapter() -> None:
    """Install the process-local CuClarabel CvxpyLayer adapter exactly once."""
    from cvxpylayers.torch import CvxpyLayer
    from neupan.blocks.nrmp import NRMP
    from neupan.configuration import tensor_dtype, to_device

    if getattr(NRMP, "_n1_7_cuclarabel_adapter", False):
        return

    def problem_definition_cuclarabel(self: Any) -> None:
        problem = self.construct_prob()
        self.nrmp_layer = to_device(
            CvxpyLayer(
                problem,
                parameters=self.para_list,
                variables=self.indep_list,
                solver="CUCLARABEL",
            )
        )

    def forward_cuclarabel(
        self: Any,
        nom_s: Any,
        nom_u: Any,
        ref_s: Any,
        ref_us: Any,
        mu_list: Any = None,
        lam_list: Any = None,
        point_list: Any = None,
    ) -> tuple[Any, Any, Any]:
        if point_list:
            self.obstacle_points = point_list[0][:, : self.max_num]
        parameter_values = self.generate_parameter_value(
            nom_s, nom_u, ref_s, ref_us, mu_list, lam_list, point_list
        )
        solutions = self.nrmp_layer(*parameter_values)
        opt_solution_state = solutions[0].type(tensor_dtype)
        opt_solution_vel = solutions[1].type(tensor_dtype)
        nom_d = None if self.no_obs else solutions[2].type(tensor_dtype)
        return opt_solution_state, opt_solution_vel, nom_d

    NRMP.problem_definition = problem_definition_cuclarabel
    NRMP.forward = forward_cuclarabel
    NRMP._n1_7_cuclarabel_adapter = True
