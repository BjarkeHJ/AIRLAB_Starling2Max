# starling2max/params.py

import copy
import os
from pathlib import Path
import yaml

from . import drone, tether

REPO_ROOT = Path(__file__).resolve().parent.parent
# overwrite for installs where config/ is not next to the package (colcon puts it in share/)
CONFIG_DIR = Path(os.environ.get("STARLING2MAX_CONFIG_DIR", REPO_ROOT / "config"))
BASE_SPEC = CONFIG_DIR / "starling2max.yaml"
VARIANT_DIR = CONFIG_DIR / "variants"

REQUIRED = {
    "name",
    "body.mass", "body.bbox", "body.mesh",
    "propulsion.rotor_offset", "propulsion.rotors",
    "propulsion.prop.mesh", "propulsion.prop.radius", "propulsion.prop.mass",
    "propulsion.prop.ct", "propulsion.prop.km_ratio",
    "propulsion.motor.kv", "propulsion.motor.vbat_max",
    "frames",
    "genesis.rotor_order",
    "gazebo.spawn_z", "gazebo.max_rot_velocity", "gazebo.min_rot_velocity",
    "gazebo.time_constants.up", "gazebo.time_constants.down", "gazebo.drone_odom_topic",
    "px4.airframe_id", "px4.rotor_order", "px4.params",
}

# Loading...
def _read(path: Path) -> dict:
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a mapping at top level")
    return data

def _merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for key, value in over.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out

def variant_files() -> dict[str, Path]:
    """ {varian name: yaml path} for every file in config/variants/ """
    files: dict[str, Path] = {}
    for path in sorted(VARIANT_DIR.glob("*.yaml")):
        name = _read(path).get("name")
        if not name:
            raise ValueError(f"{path}: variant needs a 'name")
        if name in files:
            raise ValueError(f"variant name {name!r} used by both {files[name]} and {path}")
        files[name] = path
    return files

def model_names() -> list[str]:
    """ Base model name first, then variants in file order """
    base = _read(BASE_SPEC)["name"]
    variants = variant_files()
    if base in variants:
        raise ValueError(f"variant {variants[base]} reuses the base name {base!r}")
    return [base, *variants]

def load_params(name: str | None = None) -> dict:
    """ Spec for 'name' (base model if None), merged and validated """
    p = _read(BASE_SPEC)
    if name is not None and name != p["name"]:
        files = variant_files()
        if name not in files:
            raise KeyError(f"unknown model {name!r}. Known: {[p['name'], *files]}")
        p = _merge(p, _read(files[name]))
    validate(p)
    return p

# Validating...
def _get(p: dict, dotted: str):
    node = p
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            raise KeyError(dotted)
        node = node[part]
    return node

def validate(p: dict) -> None:
    where = p.get("name", "<spec>")
    if "payload" in p and "tether" not in p:
        raise ValueError(f"{where}: 'payload' needs a 'tether' section")
    required = REQUIRED | (tether.REQUIRED if "tether" in p else set())
    missing = []
    for key in sorted(required):
        try:
            _get(p, key)
        except KeyError:
            missing.append(key)
    if missing:
        raise ValueError(f"{where}: missing keys {missing}")

    drone.validate(p)
    if "tether" in p:
        tether.validate(p)

