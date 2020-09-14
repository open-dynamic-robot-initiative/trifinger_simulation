import typing

import numpy as np
import pinocchio


class Kinematics:
    """Forward and inverse kinematics for arbitrary Finger robots.

    Provides forward and inverse kinematics functions for a Finger robot with
    arbitrarily many independent fingers.
    """

    def __init__(
        self, finger_urdf_path: str, tip_link_names: typing.Iterable[str]
    ):
        """Initializes the robot model.

        Args:
            finger_urdf_path:  Path to the URDF file describing the robot.
            tip_link_names:  Names of the finger tip frames, one per finger.
        """
        self.robot_model = pinocchio.buildModelFromUrdf(finger_urdf_path)
        self.data = self.robot_model.createData()
        self.tip_link_ids = [
            self.robot_model.getFrameId(link_name)
            for link_name in tip_link_names
        ]

    def forward_kinematics(self, joint_positions) -> typing.List[np.ndarray]:
        """Compute end-effector positions for the given joint configuration.

        Args:
            joint_positions:  Flat list of angular joint positions.

        Returns:
            List of end-effector positions. Each position is given as an
            np.array with x,y,z positions.
        """
        pinocchio.framesForwardKinematics(
            self.robot_model,
            self.data,
            joint_positions,
        )

        return [
            np.asarray(self.data.oMf[link_id].translation).reshape(-1).tolist()
            for link_id in self.tip_link_ids
        ]

    def _inverse_kinematics_step(
        self, frame_id: int, xdes: np.ndarray, q0: np.ndarray
    ) -> typing.Tuple[np.ndarray, np.ndarray]:
        """Compute one IK iteration for a single finger."""
        dt = 1.0e-1
        pinocchio.computeJointJacobians(
            self.robot_model,
            self.data,
            q0,
        )
        pinocchio.framesForwardKinematics(
            self.robot_model,
            self.data,
            q0,
        )
        Ji = pinocchio.getFrameJacobian(
            self.robot_model,
            self.data,
            frame_id,
            pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )[:3, :]
        xcurrent = self.data.oMf[frame_id].translation
        try:
            Jinv = np.linalg.inv(Ji)
        except Exception:
            Jinv = np.linalg.pinv(Ji)
        err = xdes - xcurrent
        dq = Jinv.dot(xdes - xcurrent)
        qnext = pinocchio.integrate(self.robot_model, q0, dt * dq)
        return qnext, err

    def inverse_kinematics_one_finger(
        self,
        finger_idx: int,
        tip_target_position: np.ndarray,
        joint_angles_guess: np.ndarray,
        tolerance: float = 0.005,
        max_iterations: int = 1000,
    ) -> typing.Tuple[np.ndarray, np.ndarray]:
        """Inverse kinematics for a single finger.

        Args:
            finger_idx: Index of the finger.
            tip_target_positions: Target position for the finger tip in world
                frame.
            joint_angles_guess: Initial guess for the joint angles.
            tolerance: Position error tolerance.  Stop if the error is less
                then that.
            max_iterations: Max. number of iterations.

        Returns:
            tuple: First element is the joint configuration (for joints that
                are not part of the specified finger, the values from the
                initial guess are kept).
                Second element is (x,y,z)-error of the tip position.
        """
        q = joint_angles_guess
        for i in range(max_iterations):
            q, err = self._inverse_kinematics_step(
                self.tip_link_ids[finger_idx], tip_target_position, q
            )

            if np.linalg.norm(err) < tolerance:
                break
        return q, err

    def inverse_kinematics(
        self,
        tip_target_positions: typing.Iterable[np.ndarray],
        joint_angles_guess: np.ndarray,
        tolerance: float = 0.005,
        max_iterations: int = 1000,
    ) -> typing.Tuple[np.ndarray, typing.List[np.ndarray]]:
        """Inverse kinematics for the whole manipulator.

        Args:
            tip_target_positions: List of finger tip target positions, one for
                each finger.
            joint_angles_guess: See :meth:`inverse_kinematics_one_finger`.
            tolerance: See :meth:`inverse_kinematics_one_finger`.
            max_iterations: See :meth:`inverse_kinematics_one_finger`.

        Returns:
            tuple: First element is the joint configuration, second element is
            a list of (x,y,z)-errors of the tip positions.
        """
        q = joint_angles_guess
        errors = []
        for i, pos in enumerate(tip_target_positions):
            q, err = self.inverse_kinematics_one_finger(
                i, pos, q, tolerance, max_iterations
            )
            errors.append(err)

        return q, errors
