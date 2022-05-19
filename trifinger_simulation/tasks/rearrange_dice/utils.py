"""Utility functions for the rearrange_dice task."""
import typing
import os

import numpy as np

from . import (
    NUM_DICE,
    N_CELLS_PER_ROW,
    Goal,
    _cell_center_position,
    _get_grid_cells,
    _is_cell_inside_arena,
)


#: Character that are used to mark cells outside the circular arena.
CHAR_OUTSIDE = "#"
#: Character that are used to mark empty cells inside the arena.
CHAR_EMPTY = "."
#: Character that are used to mark goal positions.
CHAR_FILLED = "X"


def create_empty_pattern() -> str:
    """Get an ASCII template with an empty pattern."""
    cells = np.full((N_CELLS_PER_ROW, N_CELLS_PER_ROW), CHAR_OUTSIDE)
    for i, j in _get_grid_cells():
        cells[i, j] = CHAR_EMPTY

    output = np.apply_along_axis(
        "\n".join, axis=0, arr=np.apply_along_axis("".join, axis=1, arr=cells)
    )

    return str(output)


def create_pattern_template_file(filename: typing.Union[str, os.PathLike]):
    """Create a goal pattern template file."""
    pattern = create_empty_pattern()

    with open(filename, "w") as f:
        f.write(pattern)
        f.write("\n")
        f.write("-- \n")
        f.write("Text after the empty line is ignored.\n")


def parse_pattern(goal_pattern: typing.Sequence[str]) -> Goal:
    """Parse goal pattern."""
    goal = []
    for i, line in enumerate(goal_pattern):
        if not line or line == "\n":
            break

        if i >= N_CELLS_PER_ROW:
            raise RuntimeError("Invalid pattern: too many rows")

        if len(line.strip()) != N_CELLS_PER_ROW:
            raise RuntimeError("Invalid pattern: wrong row length")

        filled_mask = np.array(list(line)) == CHAR_FILLED
        filled_cells = np.nonzero(filled_mask)[0]

        for c in filled_cells:
            if not _is_cell_inside_arena((i, c)):
                raise RuntimeError("Invalid pattern: goal outside the arena")

            goal.append(_cell_center_position((i, c)))

    if len(goal) != NUM_DICE:
        raise RuntimeError(
            "Invalid number of goals.  Expected %d, got %d"
            % (NUM_DICE, len(goal))
        )

    return goal


def parse_pattern_file(f: typing.TextIO) -> Goal:
    """Parse goal pattern from file."""
    return parse_pattern(f.readlines())
