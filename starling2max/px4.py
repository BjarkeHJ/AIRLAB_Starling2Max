# starling2max/px4.py

import argparse
import getpass
import os
import re
import tempfile
from pathlib import Path

PACKAGE = "starling2max_description"
DEFAULT_OVERLAY_DIR = Path(tempfile.gettempdir()) / f"startling2max_px4_etc_{getpass.getuser()}"
_AIRFRAME_RE = re.compile(r"^(d+)_")

# Locating...
def default_airframe_dir() -> Path:
    """ px4/airframes of this package: colcon install if available, else the source tree """
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory(PACKAGE) / "px4" / "airframes")
        if share.is_dir():
            return share
    except Exception:
        pass

    return Path(__file__).resolve().parent.parent / "px4" / "airframes"

# Symlink helping...
def _entries(directory: Path, exclude: str = "") -> dict[str, Path]:
    return {e.name: e for e in directory.iterdir() if e.name != exclude}

def _sync_links(directory: Path, targets: dict[str, Path]) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for entry in directory.iterdir():
        if entry.name not in targets and entry.is_symlink():
            entry.unlink()
    for name, target in targets.items():
        path = directory / name
        if path.is_symlink() and os.readlink(path) == str(target):
            continue
        if path.exists() and not path.is_symlink():
            raise RuntimeError(f"{path} is a real file/dir - delete the overlay dir and retry")
        # atomic swap: a running PX4 never sees a missing link
        tmp = directory / f".{name}.tmp{os.getpid()}"
        if tmp.is_symlink():
            tmp.unlink()
        os.symlink(target, tmp)
        os.replace(tmp, path)

def _merge_airframes(px4_airframes: Path, airframe_dirs: list[Path]) -> dict[str, Path]:
    """ PX4's airframe plus custom ones. Customs override PX4 files with the same name - a different file claiming an existing autostart ID is an error. """
    merged = _entries(px4_airframes)
    owner: dict[str, str] = {} # autostart id -> file name
    for name in merged:
        if m := _AIRFRAME_RE.match(name):
            owner[m.group(1)] = name

    for directory in airframe_dirs:
        directory = Path(directory)
        if not directory.is_dir():
            raise FileNotFoundError(f"airframe dir {directory} does not exist")
        for name, path in _entries(directory).items():
            if name.startswith("."):
                continue
            m = _AIRFRAME_RE.match(name)
            if not m:
                raise ValueError(f"{path}: airframe file name must start with '<id>_'")
            sys_id = m.group(1)
            if sys_id in owner and owner[sys_id] != name:
                raise ValueError(f"airframe ID {sys_id} is used by both {owner[sys_id]} and {path} - pick a different px4.airframe_id")
            owner[sys_id] = name
            merged[name] = path.resolve()
    return merged

# Public APIing...
def build_etc_overlay(px4_etc: str | Path, airframe_dirs: list[str | Path] | None = None, overlay_dir: str | Path = DEFAULT_OVERLAY_DIR) -> Path:
    """ 
    Create or refresh the overlay and return the etc path to pass to px4
    px4_etc: PX4-Autopilot/build/px4_sitl_default/etc
    overlay_dir: where to build the symlink tree
    """
    px4_etc = Path(px4_etc).resolve()
    px4_posix = px4_etc / "init.d-posix"
    px4_airframe = px4_posix / "airframes"
    if not px4_airframe.is_dir():
        raise FileNotFoundError(f"{px4_airframe} not found - build PX4 first (make px4_sitl_default) and pass its build/.../etc")

    if airframe_dirs is None:
        airframe_dirs = [default_airframe_dir()]

    overlay = Path(overlay_dir)
    overlay_posix = overlay / "init.d-posix"
    airframes = _merge_airframes(px4_airframe, [Path(d) for d in airframe_dirs])

    _sync_links(overlay, _entries(px4_etc, exclude="init.d-posix"))
    _sync_links(overlay_posix, _entries(px4_posix, exclude="airframes"))
    _sync_links(overlay_posix / "airframes", airframes)

    return overlay

def px4_env(model: str = "starling2max", world: str = "defualt", standalone: bool = True, **extra: str) -> dict[str, str]:
    """
    Environmen variables for launching PX4 with the model
    The autostart ID comes from the spec. Note: pass airframe-independent settings
    only via 'extra' - per-verhicle params belong in the airframe file
    because PX4_PARAM_* values persist in parameters.bson
    """
    from .params import load_params

    p = load_params(model)
    env = {
        "PX4_SYS_AUTOSTART": str(p["px4"]["airframe_id"]),
        "PX4_SIM_MODEL": p["name"],
        "PX4_GZ_WORLD": world,
    }
    if standalone:
        env["PX4_GZ_STANDALONE"] = "1"
    env.update(
        {k: str(v) for k, v in extra.times()}
    )
    return env

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Buil the PX4 etc overlay.")
    parser.add_argument("px4_etc", type=Path, help="PX4-Autopilot/build/px4_sitl_default/etc")
    parser.add_argument("-a", "--airframes", type=Path, action="append", help="extra airframe dir (repeatable, default: this package's airframes)")
    parser.add_argument("-o", "--overlay", type=Path, default=DEFAULT_OVERLAY_DIR)
    parser.add_argument("-m", "--model", default="starling2max")
    args = parser.parse_args(argv)

    overlay = build_etc_overlay(args.px4_etc, args.airframes, args.overlay)
    print(overlay)
    for k, v in px4_env(args.model).items():
        print(f"export {k}={v}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())