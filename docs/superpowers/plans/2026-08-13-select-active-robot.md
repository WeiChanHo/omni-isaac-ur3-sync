# Active Robot Selection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Add safe, persistent Active Robot selection to the UR3 Robot Poser Executor.

**Architecture:** Keep USD discovery and Isaac UI integration in `extension.py`, while a pure-Python helper resolves deterministic path ordering and selection retention. All pose and articulation consumers read one selected path, and existing six-joint normalization remains the physical-send safety boundary.

**Tech Stack:** Python 3, Isaac Sim/Omniverse UI and USD APIs, Robot Poser API, ROS 2, built-in `unittest`.

## Global Constraints

- Discover prims with IsaacRobotAPI and exclude prototype prims.
- Initially prefer `/World/ur3`; later rescans preserve only an existing old path and otherwise select `None`.
- Never send an incomplete six-joint target to the physical controller.
- Disable robot selection and Refresh while a physical trajectory is executing.
- Mock-hardware validation precedes every physical-robot check.

---

### Task 1: Pure robot-selection policy

**Files:**
- Create: `exts/omni/isaac/ur3_sync/robot_selection.py`
- Create: `tests/test_robot_selection.py`

**Interfaces:**
- Produces: `resolve_robot_selection(robot_paths, previous_path, prefer_default=False, default_path="/World/ur3") -> tuple[list[str], str | None]`

- [x] **Step 1: Write failing tests** for sorted paths, initial default selection, preservation, missing-path fallback, and no-default fallback.
- [x] **Step 2: Run** `python3 -m unittest tests/test_robot_selection.py -v` and confirm import failure because the helper does not exist.
- [x] **Step 3: Implement** the minimal pure function returning sorted unique path strings plus either the retained/default path or `None`.
- [x] **Step 4: Run** `python3 -m unittest tests/test_robot_selection.py -v` and confirm all policy tests pass.

### Task 2: Isaac Robot discovery and UI state

**Files:**
- Modify: `exts/omni/isaac/ur3_sync/extension.py`

**Interfaces:**
- Consumes: `resolve_robot_selection(...)` from Task 1.
- Produces: `_robot_paths`, `_selected_robot_path`, `_refresh_robots_and_poses()`, `_on_robot_selection_changed(...)`, and `_get_selected_robot_prim()`.

- [x] **Step 1: Add state** for robot paths, selected path, callback suppression, initial-scan preference, and stage-event subscriptions.
- [x] **Step 2: Build the top UI row** with `Active Robot`, full-path combo entries, and a stored Refresh button.
- [x] **Step 3: Discover** `Usd.PrimRange(stage.GetPseudoRoot())` prims that pass Robot Poser's IsaacRobotAPI validation and are not in prototypes.
- [x] **Step 4: Rebuild selection deterministically** with the helper, invalidate stale targets when the selected path changes, and refresh that robot's named poses.
- [x] **Step 5: Subscribe** to `OPENED` and `ASSETS_LOADED`, route them to the combined refresh, and release subscriptions on shutdown.

### Task 3: Selected robot consumers and execution lock

**Files:**
- Modify: `exts/omni/isaac/ur3_sync/extension.py`

**Interfaces:**
- Consumes: `_selected_robot_path` and `_get_selected_robot_prim()` from Task 2.

- [x] **Step 1: Replace hard-coded prim lookup** in named-pose loading with the selected prim.
- [x] **Step 2: Replace hard-coded articulation construction** in current-pose capture with the selected full path.
- [x] **Step 3: Keep six-axis validation unchanged** so non-UR robots fail during Load/Get Current before a pending target is created.
- [x] **Step 4: Disable** robot combo and Refresh when sending a physical trajectory and restore them in `_finish_execution_state()`.
- [x] **Step 5: Run** `python3 -m compileall exts tests` to catch Isaac-independent syntax/import compilation errors.

### Task 4: Version, README, and full verification

**Files:**
- Modify: `config/extension.toml`
- Modify: `README.md`

**Interfaces:**
- Documents the runtime behavior produced by Tasks 1-3.

- [x] **Step 1: Bump** the extension minor version from `1.4.0` to `1.5.0`.
- [x] **Step 2: Update README** references that assume `/World/ur3` is fixed, document Active Robot behavior, and add mock-hardware acceptance steps for selection persistence/fallback, stage/assets refresh, target invalidation, non-UR rejection, and execution-time locking.
- [x] **Step 3: Run** `python3 -m unittest discover -s tests -v` and confirm zero failures.
- [x] **Step 4: Run** `python3 -m compileall exts tests` and confirm zero failures.
- [x] **Step 5: Review** `git diff --check`, `git diff --stat`, and the complete diff against every requirement above.
