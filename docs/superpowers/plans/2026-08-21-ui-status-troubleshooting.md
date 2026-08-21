# UI Status Troubleshooting Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Document every red `STATUS_ERROR` message shown by the extension UI, including its trigger and an actionable recovery procedure.

**Architecture:** Treat `_set_status(..., self.STATUS_ERROR)` calls in the target and trajectory workflows as the authoritative message inventory. Include validation exceptions that flow into those calls, group duplicate message patterns, and preserve dynamic placeholders so operators can match runtime text to the documentation.

**Tech Stack:** Markdown, Python source inspection, `unittest`

**Spec:** User request in the 2026-08-21 conversation; repository rules in `AGENTS.md` supplied with that request.

## Global Constraints

- Modify only documentation; do not change runtime behavior.
- List every UI Status error message, its triggering condition, and its recovery steps.
- Preserve the safety distinction that **Cancel Goal** is not an emergency stop.
- Require mock-hardware validation before physical-robot checks.

---

### Task 1: Replace the abbreviated troubleshooting table with the complete UI error reference

**Files:**
- Modify: `README.md` under `## 7. Quick Troubleshooting`
- Test: `exts/omni/isaac/ur3_sync/target_workflow.py`
- Test: `exts/omni/isaac/ur3_sync/trajectory_workflow.py`
- Test: `exts/omni/isaac/ur3_sync/joint_targets.py`

**Interfaces:**
- Consumes: UI messages passed to `_set_status(message, self.STATUS_ERROR)` and validation exceptions displayed through error-status catch blocks.
- Produces: A Markdown troubleshooting reference with columns for UI message, trigger, and handling procedure.

- [x] **Step 1: Inventory direct and propagated error messages**

Run:

```bash
rg -n -U '_set_status\([\s\S]{0,240}?STATUS_ERROR|raise RuntimeError' exts/omni/isaac/ur3_sync
```

Expected: every error-status call and every validation error routed to the status label is visible for comparison.

- [x] **Step 2: Expand the README troubleshooting section**

Replace the abbreviated symptom table with grouped tables covering robot/stage discovery, target acquisition, trajectory execution, result handling, and cancellation. Represent runtime values as placeholders such as `<exception>`, `<pose_name>`, and `<status>`.

- [x] **Step 3: Verify source-to-document coverage**

Run a source/document comparison to confirm each static error prefix and each propagated `RuntimeError` text appears in `README.md`. Review generic `<exception>` entries separately because their suffix comes from Isaac Sim, ROS 2, or the UR controller.

- [x] **Step 4: Run repository verification**

Run:

```bash
python3 -m unittest discover -s tests -v
python3 -m compileall exts tests
```

Expected: all unit tests pass and Python compilation reports no errors.
