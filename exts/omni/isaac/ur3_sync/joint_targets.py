"""Isaac-independent validation helpers for UR joint targets."""

import math


def normalize_joint_positions(
    dof_names,
    position_rows,
    expected_joint_names,
):
    """Return one articulation position row in controller joint order."""
    names = list(dof_names)
    expected = list(expected_joint_names)

    if len(position_rows) != 1:
        raise RuntimeError(
            "Simulation articulation returned an unexpected joint position "
            f"row count: {len(position_rows)}"
        )

    row = list(position_rows[0])
    if len(row) != len(names):
        raise RuntimeError(
            "Simulation articulation returned mismatched DOF names and "
            f"positions: {len(names)} names, {len(row)} positions"
        )

    if len(set(names)) != len(names):
        raise RuntimeError(
            "Simulation articulation returned duplicate DOF names"
        )

    index_by_name = {
        name: index
        for index, name in enumerate(names)
    }
    missing = [
        name
        for name in expected
        if name not in index_by_name
    ]
    if missing:
        raise RuntimeError(
            f"Simulation articulation is missing UR joints: {missing}"
        )

    positions = [
        float(row[index_by_name[name]])
        for name in expected
    ]
    if not all(math.isfinite(value) for value in positions):
        raise RuntimeError(
            "Current simulation pose contains NaN or infinite values"
        )
    return positions
