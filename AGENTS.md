# Repository Guidelines

## Project Structure & Module Organization

Runtime code lives in `exts/omni/isaac/ur3_sync/`. `extension.py` owns the Isaac Sim UI, ROS 2 lifecycle, trajectory action client, and motion watchdog; keep Isaac-independent validation in focused helpers such as `joint_targets.py`. Extension metadata and Isaac dependencies are declared in `config/extension.toml`. Unit tests are in `tests/`, with filenames following `test_*.py`. `scenes/` contains the development USD stage, while `feedback/` and `weekly_report_0801/` are reference material rather than runtime inputs.

## Build, Test, and Development Commands

This extension has no separate build step. Run the Isaac-independent test suite from the repository root:

```bash
python3 -m unittest discover -s tests -v
```

Optionally check Python syntax before opening Isaac Sim:

```bash
python3 -m compileall exts tests
```

For interactive development, launch Isaac Sim from the same sourced ROS 2 environment as the UR driver, add this repository's parent directory to Extension Manager's search paths, and enable `omni.isaac.ur3_sync`. Start integration checks with UR mock hardware.

## Coding Style & Naming Conventions

Use four-space indentation and follow the existing PEP 8-style layout. Name functions and variables with `snake_case`, classes with `PascalCase`, and constants with `UPPER_SNAKE_CASE`. Keep methods small and grouped by responsibility, as in `Ur3SyncExtension`. Prefer clear docstrings and actionable error messages. No formatter or linter is configured, so match nearby code and avoid unrelated formatting churn.

## Testing Guidelines

Tests use Python's built-in `unittest`. Add a `test_<behavior>` method for each success path and rejected input, including missing joints, duplicate names, malformed rows, and non-finite values. Keep unit-testable logic free of Isaac Sim imports. There is no stated coverage threshold; every behavioral change should include regression tests. Treat mock-hardware validation as mandatory before any physical-robot check.

## Commit & Pull Request Guidelines

Recent history favors concise, imperative Conventional Commit prefixes such as `feat:`, `fix:`, `refactor:`, and `chore:`. Keep each commit focused. Pull requests should explain the behavior and safety impact, list test commands and results, and link relevant issues. Include screenshots for UI changes and describe mock-hardware results for ROS or motion changes. Never present `Cancel Goal` as an emergency-stop mechanism; document any changed ROS topic, action, prim path, or speed assumption.
