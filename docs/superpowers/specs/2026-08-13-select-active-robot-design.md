# Active Robot Selection Design

## Goal

Allow the operator to choose which IsaacRobotAPI prim supplies Robot Poser
named poses and current articulation positions, without weakening the existing
six-joint UR3 validation before physical execution.

## Selection behavior

- The top of the extension window contains an `Active Robot` combo box and a
  `Refresh` button.
- Immediately below that row, a dedicated status label displays
  `Selected robot: <full prim path>`. When no robot is selected, it displays
  `Selected robot: None`. Its visual style matches the existing selected-pose
  label so the active robot remains visible even when the combo box is not
  open.
- Discovery walks the current USD stage exactly like Robot Poser: include prims
  carrying `IsaacRobotAPI`, exclude prims inside prototypes, and display full
  prim paths.
- Paths are sorted deterministically. `None` is a sentinel UI entry rather than
  a USD path.
- On the extension's initial scan, `/World/ur3` is selected when present.
- On every later scan, the previous full path is retained only while it remains
  discoverable. If it disappears, the selection becomes `None`; another robot
  is never chosen implicitly.

## State and lifecycle

The extension stores the discovered paths and selected path separately from
the combo-box model. Rebuilding the model is callback-suppressed. A real user
selection change immediately invalidates any validated/captured target and
loads the selected robot's named-pose list. Selecting `None` clears the pose
list and disables target-producing operations through their normal validation
paths.

The selected-robot label is updated after every combo-box rebuild and every
user selection change. Therefore initial discovery, manual Refresh, Stage
`OPENED`, Stage `ASSETS_LOADED`, selection fallback to `None`, and explicit
robot changes all leave the label synchronized with `_selected_robot_path`.

Manual Refresh rescans both robots and the selected robot's named poses. Stage
`OPENED` and `ASSETS_LOADED` events perform the same rescan. Subscriptions are
released on shutdown.

## Motion safety

Named-pose queries, prim lookup, and `Articulation(...)` construction use the
selected robot path. The existing fixed six-name UR3 validation remains the
last gate for both named poses and current simulation captures, so selecting a
non-UR robot cannot produce an incomplete physical target.

While a physical trajectory is active, the Active Robot combo box and Refresh
button are disabled alongside the existing target controls. All controls are
restored on every execution completion/failure path. The active trajectory
continues to use its immutable target snapshot.

## Testing and documentation

An Isaac-independent helper owns sorting, old-selection retention, the initial
`/World/ur3` preference, and fallback to `None`. `unittest` covers each branch.
The README documents the new UI/state behavior and adds mock-hardware checks
for discovery, selection invalidation, stage refresh, non-UR rejection, and
control locking. Extension metadata receives a minor-version bump.
