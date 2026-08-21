# Pose Acquisition Method Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace ambiguous target “source” terminology with “method” everywhere that describes how a pose was acquired.

**Architecture:** Add an Isaac-independent formatter for the target summary so both acquisition workflows use one tested UI contract. Rename internal constants, state, parameters, tests, and documentation from target source to pose acquisition method without changing trajectory behavior.

**Tech Stack:** Python 3, `unittest`, Isaac Sim extension modules, Markdown

**Spec:** User request in the 2026-08-21 conversation; no separate specification file.

## Global Constraints

- Display `Method: Robot Poser Named Pose` for a loaded Robot Poser pose.
- Display `Method: Current Simulation Pose` for a captured simulation pose.
- Use method-oriented identifiers for pose acquisition state and constants.
- Do not rename unrelated shell `source` commands or generic source-data variables.
- Keep Isaac-independent behavior covered by `unittest`.

---

### Task 1: Target summary UI contract

**Files:**
- Modify: `exts/omni/isaac/ur3_sync/joint_targets.py`
- Modify: `exts/omni/isaac/ur3_sync/target_workflow.py`
- Test: `tests/test_joint_targets.py`

**Interfaces:**
- Consumes: method display name, target label, and already formatted joint positions.
- Produces: `format_target_summary(method, label, positions_text) -> str`.

- [ ] **Step 1: Write the failing test**

Add table-driven assertions that the formatter returns summaries beginning with the literal `Method:` line for both acquisition methods.

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.test_joint_targets -v`

Expected: import failure because `format_target_summary` does not exist.

- [ ] **Step 3: Write minimal implementation**

Implement the formatter in `joint_targets.py`, import it into `target_workflow.py`, and replace both hand-built `Source:` summaries with formatter calls.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest tests.test_joint_targets -v`

Expected: all joint-target tests pass.

### Task 2: Internal terminology and documentation

**Files:**
- Modify: `exts/omni/isaac/ur3_sync/extension.py`
- Modify: `exts/omni/isaac/ur3_sync/target_workflow.py`
- Modify: `exts/omni/isaac/ur3_sync/trajectory_workflow.py`
- Modify: `tests/test_ui_layout.py`
- Modify: `README.md`

**Interfaces:**
- Consumes: the existing pending-target workflow.
- Produces: `POSE_METHOD_NAMED_POSE`, `POSE_METHOD_CURRENT_SIMULATION`, `_pending_pose_method`, and `_set_pending_target(method, label, positions)`.

- [ ] **Step 1: Refactor identifiers while green**

Rename target-source constants, pending state, method parameter, and the UI-layout test name to method terminology. Update the README data-flow signature and prose that asks the operator to review the acquisition method.

- [ ] **Step 2: Verify terminology**

Run: `rg -n "pending_target_source|TARGET_SOURCE|Source:|set_pending_target\(source|target source" exts tests README.md`

Expected: no matches.

- [ ] **Step 3: Run full verification**

Run: `python3 -m unittest discover -s tests -v`

Run: `python3 -m compileall exts tests`

Run: `git diff --check`

Expected: all tests pass, compilation succeeds, and no whitespace errors are reported.
