"""Configure grSim's existing physical walls before starting the simulator."""

import argparse
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


FIELD = {
    "Length": 9.0, "Width": 6.0,
    "Margin along the touch lines": 0.3,
    "Margin behind the goal lines": 0.3,
    "Referee margin": 0.0, "Wall thickness": 0.05,
    "Penalty width": 2.0, "Penalty depth": 1.0,
    "Goal width": 1.0, "Goal depth": 0.18,
    "Goal height": 0.16, "Goal thickness": 0.02,
}


def configure(path, check=False):
    tree = ET.parse(path) if path.exists() else ET.ElementTree(ET.Element("VarXML"))
    changed = False

    def node(parent, name, kind):
        nonlocal changed
        found = parent.find(f"Var[@name='{name}']")
        if found is None:
            found = ET.SubElement(parent, "Var", name=name, type=kind)
            changed = True
        return found

    geometry = node(tree.getroot(), "Geometry", "list")
    game = node(geometry, "Game", "list")
    division = node(game, "Division", "stringenum")
    if (division.text or "").strip() != "Division B":
        division.text = "Division B"
        changed = True
    field = node(node(geometry, "Field", "list"), "Division B", "list")
    for key, value in FIELD.items():
        entry = node(field, key, "double")
        try:
            matches = float(entry.text or "") == value
        except ValueError:
            matches = False
        if not matches:
            entry.text = str(value)
            changed = True
    if changed and not check:
        backup = path.with_suffix(path.suffix + ".before-division-b")
        if path.exists() and not backup.exists():
            shutil.copy2(path, backup)
        tree.write(path, encoding="utf-8", xml_declaration=True)
    return changed


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--config", type=Path, default=Path.home() / ".grsim.xml")
    args = parser.parse_args()
    changed = configure(args.config, args.check)
    raise SystemExit(1 if args.check and changed else 0)
