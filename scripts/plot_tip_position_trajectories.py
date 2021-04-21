#!/usr/bin/env python3
"""Script for plotting tip position data from a log file."""
import argparse
import pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "data_file",
        help="""Path to the pickle file containing the logged
                        data""",
    )
    parser.add_argument(
        "--episode",
        type=int,
        help="""Number of the episode
                        When this is set, only the given episode will be
                        plotted, --start-episode and --end-episode will be
                        ignored.""",
    )
    parser.add_argument(
        "-s",
        "--start-episode",
        type=int,
        help="""Number of the first episode when plotting a
                        range of episodes.""",
    )
    parser.add_argument(
        "-e",
        "--end-episode",
        type=int,
        help="""Number of the last episode when plotting a
                        range of episodes.""",
    )
    parser.add_argument(
        "--plain",
        choices=["xy", "xz"],
        help="""Specifies on which plane the data is
                        plotted.""",
    )
    parser.add_argument(
        "--output",
        type=str,
        help="""Path to a file to store the plot.
                        If not set, the plot will be displayed instead.""",
    )
    args = parser.parse_args()

    with open(args.data_file, "rb") as file_handle:
        episodes = pickle.load(file_handle)

    x = 0

    plt.rcParams.update(
        {
            "font.size": 20,
            "font.weight": "bold",
            "axes.linewidth": 3,
        }
    )

    if args.episode:
        if args.plain == "xy":
            y = 1
        else:
            y = 2
        data = episodes[args.episode]

        tip_goal = data["tip_goal"][0]
        trajectory = np.vstack(data["tip_positions"])

        plt.plot(tip_goal[x], tip_goal[y], "rx")
        plt.plot(trajectory[:, x], trajectory[:, y], "b-o")
    else:
        if args.start_episode:
            start = args.start_episode
            end = args.end_episode + 1
        else:
            start = 1
            end = len(episodes)

        # if args.plain is not set, plot both, otherwise only one
        plains = []
        num_subplots = 0
        if args.plain in ("xy", None):
            plains.append(("xy-plane", "y", num_subplots, 1))
            num_subplots += 1
        if args.plain in ("xz", None):
            plains.append(("xz-plane", "z", num_subplots, 2))
            num_subplots += 1

        fig, ax = plt.subplots(1, num_subplots, figsize=(10, 10), dpi=300)
        if num_subplots == 1:
            ax = [ax]

        for title, ylabel, a, y in plains:

            for i in range(start, end):
                data = episodes[i]

                tip_goal = np.asarray(data["tip_goal"][0])
                trajectory = np.vstack(data["tip_positions"])
                dist = trajectory - tip_goal

                ax[a].plot(
                    dist[:, x], dist[:, y], color=(0.3, 0.3, 0.3), linewidth=2
                )
                ax[a].plot(dist[0, x], dist[0, y], "bo", zorder=40)
                ax[a].plot(dist[-1, x], dist[-1, y], "g^", zorder=40)

            circ = patches.Circle(
                (0, 0),
                0.02,
                edgecolor="orange",
                facecolor="none",
                linewidth=4,
                zorder=50,
            )

            ax[a].add_patch(circ)

            ax[a].set_xlabel("x")
            ax[a].set_ylabel(ylabel + "  ", rotation="horizontal")

            ax[a].axis("equal")

            # Move left y-axis and bottim x-axis to centre, passing
            #  through (0,0)
            ax[a].spines["left"].set_position("zero")
            ax[a].spines["bottom"].set_position("zero")

            ax[a].spines["left"].set_color("grey")
            ax[a].spines["bottom"].set_color("grey")
            ax[a].tick_params(axis="x", colors="grey")
            ax[a].tick_params(axis="y", colors="grey")

            # Eliminate upper and right axes
            ax[a].spines["right"].set_color("none")
            ax[a].spines["top"].set_color("none")

            # Show ticks in the left and lower axes only
            ax[a].xaxis.set_ticks_position("bottom")
            ax[a].yaxis.set_ticks_position("left")

            ax[a].xaxis.set_label_coords(1, 0.5)
            ax[a].yaxis.set_label_coords(0.5, 1)

    if args.output:
        plt.savefig(args.output, bbox_inches="tight")
    else:
        plt.show()


if __name__ == "__main__":
    main()
