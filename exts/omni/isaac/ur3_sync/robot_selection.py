"""Pure selection policy for Active Robot discovery results."""


DEFAULT_ROBOT_PRIM_PATH = "/World/ur3"


def format_selected_robot_label(selected_path):
    """Format the persistent UI label for the selected robot path."""
    display_path = selected_path if selected_path is not None else "None"
    return f"Selected robot: {display_path}"


def resolve_robot_selection(
    robot_paths,
    previous_path,
    prefer_default=False,
    default_path=DEFAULT_ROBOT_PRIM_PATH,
):
    """Sort discovered paths and choose a retained or initial selection.

    Later rescans never select a different robot implicitly. If the previous
    path disappeared, the returned selection is ``None`` even when the default
    robot or another robot is available.
    """
    sorted_paths = sorted({str(path) for path in robot_paths})

    if previous_path in sorted_paths:
        selected_path = previous_path
    elif prefer_default and default_path in sorted_paths:
        selected_path = default_path
    else:
        selected_path = None

    return sorted_paths, selected_path
