#!/usr/bin/env python3
"""Plot the position error over all episodes.

Plots for each episode the position error between desired and actual finger tip
position at the last step of the episode.

Also prints the mean error over all episodes.  This is only useful when using a
log file from testing a final policy.
"""
import argparse
import pickle
import numpy as np
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("data_file")
    args = parser.parse_args()

    with open(args.data_file, "rb") as file_handle:
        episodes = pickle.load(file_handle)

    dist = []
    for data in episodes[1:]:
        tip_goal = data["tip_goal"][0]
        trajectory = np.vstack(data["tip_positions"])
        dist.append(np.linalg.norm(trajectory[-1] - tip_goal))

    print("Mean position error over all episodes: {}".format(np.mean(dist)))

    fig, ax = plt.subplots(1, 1, dpi=300)
    ax.plot(dist, linewidth=2)

    ax.set_xlabel("Episode")
    ax.set_ylabel("Distance to goal at end of episode")

    plt.show()


if __name__ == "__main__":
    main()
