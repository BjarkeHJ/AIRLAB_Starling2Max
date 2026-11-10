# starling2max/params.py

import copy
import math
import os
from pathlib import Path
import yaml

from .rotors import rotors_for

REPO_ROOT = Path(__file__).resolve().parent.parent
# overwrite for installs where config/ is not next to the package (colcon puts it in share/)
CONFIG_DIR = Path(os.environ.get("STARLING2MAX_CONFIG_DIR", REPO_ROOT / "config"))
BASE_SPEC = CONFIG_DIR / "starling2max.yaml"
VARIANT_DIR = CONFIG_DIR / "variants"

# Physical constants
RHO_AIR = 1.225
GRAVITY = 9.81
RPM_TO_RAD = 2 * math.pi / 60

# x500 reference values (PX4 x500/model.sdf). Rotor drag and rolling moment are
# scaled by motorConstant / X500_MOTOR_CONSTANT. Uncalibrated.
X500_MOTOR_CONSTANT = 8.54858e-06
X500_ROTOR_DRAG = 8.06428e-05
X500_ROLLING_MOMENT = 1e-06

PX4_EC_MAX_LIMIT = 1000  # SIM_GZ_EC_MAX cannot exceed this

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
    "gazebo.time_constants.up", "gazebo.time_constants.down", "gazebo.ground_truth_topic",
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

def _positive(p: dict, *keys: str) -> None:
    for key in keys:
        v = _get(p, key)
        if not isinstance(v, (int, float)) or v <= 0:
            raise ValueError(f"{key} must be a positive number, got {v!r}")

def validate(p: dict) -> None:
    where = p.get("name", "<spec>")
    missing = []
    for key in REQUIRED:
        try:
            _get(p, key)
        except:
            missing.append(key)
    if missing:
        raise ValueError(f"{where}: missing keys {missing}")

    _positive(p, "body.mass", "propulsion.prop.radius", "propulsion.prop.mass",
              "propulsion.prop.ct", "propulsion.prop.km_ratio",
              "gazebo.max_rot_velocity", "gazebo.min_rot_velocity",
              "gazebo.time_constants.up", "gazebo.time_constants.down")

    for key in ("body.bbox", "propulsion.rotor_offset"):
        v = _get(p, key)
        if len(v) != 3:
            raise ValueError(f"{where}: {key} must have 3 values, got {v}")
        
    if any(s <= 0 for s in p["body"]["bbox"]):
        raise ValueError(f"{where}: body.bbox must be positive")

    for frame, spec in (p["frames"] or {}).items():
        if len(spec.get("xyz", ())) != 3:
            raise ValueError(f"{where}: frames.{frame}.xyz must have 3 values")

    gz = p["gazebo"]
    if not gz["min_rot_velocity"] < gz["max_rot_velocity"] <= PX4_EC_MAX_LIMIT:
        raise ValueError(f"{where}: need min_rot_velocity < max_rot_velocity <= {PX4_EC_MAX_LIMIT}; "
                         "scale motorConstant for more thrust")

    n = len(p["propulsion"]["rotors"])
    if p["body"]["mass"] <= n * p["propulsion"]["prop"]["mass"]:
        raise ValueError(f"{where}: body.mass must exceed the total prop mass")

    # Rotor names, spins, diagonals and both orderings (raises on error).
    rotors_for(p, "genesis")
    rotors_for(p, "px4")

    # The vehicle has to be able to hover inside the commanded speed range.
    d = derive(p)
    if not 0 < d["hover_throttle"] < 1:
        raise ValueError(f"{where}: hover needs {d['hover_velocity']:.0f} rad/s, outside "
                         f"[{gz['min_rot_velocity']}, {gz['max_rot_velocity']}]")

# Deriving...
def box_inertia(mass: float, sx: float, sy: float, sz: float) -> tuple[float, float, float]:
    """Solid box about its centre: (ixx, iyy, izz)."""
    return (
        mass / 12.0 * (sy**2 + sz**2),
        mass / 12.0 * (sx**2 + sz**2),
        mass / 12.0 * (sx**2 + sy**2),
    )

def derive(p: dict) -> dict:
    body = p["body"]
    prop = p["propulsion"]["prop"]
    motor = p["propulsion"]["motor"]
    gz = p["gazebo"]
    n = len(p["propulsion"]["rotors"])

    # T = kf * RPM^2 with T = ct * rho * n^2 * D^4 and n = RPM / 60
    kf = prop["ct"] * RHO_AIR * (2 * prop["radius"]) ** 4 / 3600.0
    km = kf * prop["km_ratio"] # Q = km_ratio * T
    motor_constant = kf / RPM_TO_RAD**2 # N/(rad/s)^2

    hover_velocity = math.sqrt(body["mass"] * GRAVITY / n / motor_constant)
    lo, hi = gz["min_rot_velocity"], gz["max_rot_velocity"]

    base_mass = body["mass"] - n * prop["mass"]
    prop_izz = prop["mass"] * prop["radius"] ** 2 / 3.0 # thin blades: m R^2 / 3
    ratio = motor_constant / X500_MOTOR_CONSTANT

    return {
        # aerodynamics
        "kf": kf, # N/RPM^2 (Genesis <properties>)
        "km": km, # N m/RPM^2 (Genesis <properties>)
        "motor_constant": motor_constant, # N/(rad/s)^2 (Gazebo)
        "moment_constant": prop["km_ratio"], # m (Gazebo)
        "rotor_drag": X500_ROTOR_DRAG * ratio,
        "rolling_moment": X500_ROLLING_MOMENT * ratio,
        # operating point
        "hover_velocity": hover_velocity, # rad/s
        "hover_rpm": hover_velocity / RPM_TO_RAD,
        "hover_throttle": (hover_velocity - lo) / (hi - lo),
        "max_t2w": n * motor_constant * hi**2 / (body["mass"] * GRAVITY),
        "noload_rpm": motor["kv"] * motor["vbat_max"],
        # mass and inertia
        "mass": body["mass"],
        "base_mass": base_mass,
        "base_inertia": box_inertia(base_mass, *body["bbox"]),
        "prop_mass": prop["mass"],
        "prop_inertia": (prop_izz / 2, prop_izz / 2, prop_izz),
    }