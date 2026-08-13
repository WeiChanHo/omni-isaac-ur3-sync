"""Source-level regression tests for the Isaac Sim UI layout."""

import ast
import unittest
from pathlib import Path


EXTENSION_PATH = (
    Path(__file__).resolve().parents[1]
    / "exts"
    / "omni"
    / "isaac"
    / "ur3_sync"
    / "extension.py"
)


def _assigned_attributes(nodes):
    attributes = []
    for node in nodes:
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if (
                isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name)
                and target.value.id == "self"
            ):
                attributes.append(target.attr)
    return attributes


class UiLayoutTests(unittest.TestCase):
    def test_target_source_buttons_share_horizontal_row(self):
        tree = ast.parse(EXTENSION_PATH.read_text(encoding="utf-8"))
        build_ui = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef) and node.name == "_build_ui"
        )

        horizontal_rows = [
            node
            for node in ast.walk(build_ui)
            if isinstance(node, ast.With)
            and any(
                isinstance(item.context_expr, ast.Call)
                and isinstance(item.context_expr.func, ast.Attribute)
                and isinstance(item.context_expr.func.value, ast.Name)
                and item.context_expr.func.value.id == "ui"
                and item.context_expr.func.attr == "HStack"
                for item in node.items
            )
        ]

        self.assertTrue(
            any(
                _assigned_attributes(row.body)[:2]
                == ["load_btn", "get_current_btn"]
                for row in horizontal_rows
            ),
            "Load and current-pose buttons must share one horizontal row",
        )


if __name__ == "__main__":
    unittest.main()
