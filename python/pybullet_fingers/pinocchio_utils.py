import numpy as np

import pinocchio


class PinocchioUtils:
    """
    Consists of kinematic methods for the finger platform.
    """

    def __init__(self, finger_urdf_path, tip_link_names):
        """
        Initializes the finger model on which control's to be performed.

        Args:
            finger (SimFinger): An instance of the SimFinger class
        """
        self.robot_model = pinocchio.buildModelFromUrdf(finger_urdf_path)
        self.data = self.robot_model.createData()
        self.tip_link_ids = [
            self.robot_model.getFrameId(link_name)
            for link_name in tip_link_names
        ]

    def forward_kinematics(self, joint_positions):
        """
        Compute end effector positions for the given joint configuration.

        Args:
            finger (SimFinger): a SimFinger object
            joint_positions (list): Flat list of angular joint positions.

        Returns:
            List of end-effector positions. Each position is given as an
            np.array with x,y,z positions.
        """
        pinocchio.framesForwardKinematics(
            self.robot_model, self.data, joint_positions,
        )

        return [
            np.asarray(self.data.oMf[link_id].translation).reshape(-1).tolist()
            for link_id in self.tip_link_ids
        ]

    def inverse_kinematics(self, fid, xdes, q0):
        """
        Method not in use right now, but is here with the intention
        of using pinocchio for inverse kinematics instead of using
        the in-house IK solver of pybullet.
        """
        raise NotImplementedError()
        dt = 1.0e-3
        pinocchio.computeJointJacobians(
            self.robot_model, self.data, q0,
        )
        pinocchio.framesKinematics(
            self.robot_model, self.data, q0,
        )
        pinocchio.framesForwardKinematics(
            self.robot_model, self.data, q0,
        )
        Ji = pinocchio.getFrameJacobian(
            self.robot_model,
            self.data,
            fid,
            pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )[:3, :]
        xcurrent = self.data.oMf[fid].translation
        try:
            Jinv = np.linalg.inv(Ji)
        except Exception:
            Jinv = np.linalg.pinv(Ji)
        dq = Jinv.dot(xdes - xcurrent)
        qnext = pinocchio.integrate(self.robot_model, q0, dt * dq)
        return qnext
