# starling2max/render.py

""" Jinja environment, formatting filters, and writing/checking generated files. """

import difflib
import sys
from pathlib import Path
import jinja2

TEMPLATE_DIR = Path(__file__).resolve().parent / "templates"

COLORS = {
    "grey": (0.1, 0.1, 0.1, 1.0),
    "green": (0.0, 0.8, 0.0, 1.0),
    "red": (0.8, 0.0, 0.0, 1.0),
    "blue": (0.0, 0.0, 0.8, 1.0),
}

# formatting filters...
def num(v, digits: int = 6) -> str:
    return f"{float(v):.{digits}g}"

def vec(values, digits: int = 6) -> str:
    return " ".join(num(v, digits) for v in values)

def rgba(color: str) -> str:
    return vec(COLORS[color])

def make_env() -> jinja2.Environment:
    env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(TEMPLATE_DIR),
        undefined=jinja2.StrictUndefined,
        keep_trailing_newline=True,
        trim_blocks=True,
        lstrip_blocks=True,
    )
    env.filters.update(num=num, vec=vec, rgba=rgba) # give custom filters for formatting
    return env

# writing / checking...
def write(files: dict[Path, str], root: Path) -> None:
    """ Write every file whose content changed """
    for rel, text in files.items():
        path = root / rel
        if path.exists() and path.read_text() == text:
            continue # has not changed
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
        print(f"wrote {rel}")

def check(files: dict[Path, str], root: Path) -> list[Path]:
    """ print a diff for every stale file and return their paths """
    stale = []
    for rel, text in files.items():
        path = root / rel
        old = path.read_text() if path.exists() else ""
        if old != text:
            stale.append(rel)
            sys.stdout.writelines(difflib.unified_diff(
                old.splitlines(keepends=True), text.splitlines(keepends=True),
                fromfile=f"a/{rel}",
                tofile=f"b/{rel}"
            ))
    return stale 
