import pickle


class EpisodeData:
    """
    The structure in which the data from each episode
    will be logged.
    """

    def __init__(self, joint_goal, tip_goal):
        self.joint_goal = joint_goal
        self.tip_goal = tip_goal
        self.joint_positions = []
        self.tip_positions = []
        self.timestamps = []

    def append(self, joint_pos, tip_pos, timestamp):
        self.joint_positions.append(joint_pos)
        self.tip_positions.append(tip_pos)
        self.timestamps.append(timestamp)


class DataLogger:
    """
    Dumps the env episodic data to a pickle file
    """

    def __init__(self):
        self.episodes = []
        self._curr = None

    def new_episode(self, joint_goal, tip_goal):
        if self._curr:
            # convert to dict for saving so loading has no dependencies
            self.episodes.append(self._curr.__dict__)

        self._curr = EpisodeData(joint_goal, tip_goal)

    def append(self, joint_pos, tip_pos, timestamp):
        self._curr.append(joint_pos, tip_pos, timestamp)

    def store(self, filename):
        with open(filename, "wb") as file_handle:
            pickle.dump(self.episodes, file_handle)
