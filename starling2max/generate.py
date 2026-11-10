# starling2max/generate.py

import argparse
import copy
import difflib
import sys
import os
from pathlib import Path
import jinja2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from starling2max.params import REPO_ROOT, derive, load_params, model_names
from starling2max.rotors import rotors_for

TEMPLATE_DIR = Path(__file__).resolve().parent / "templates"
MODELS = Path("models")
AIRFRAMES = Path("px4") / "airframes"
MESH_SCALE = (0.001, 0.001, 0.001)  # meshes are in mm

COLORS = {
    "grey": (0.1, 0.1, 0.1, 1.0),
    "green": (0.0, 0.8, 0.0, 1.0),
    "red": (0.8, 0.0, 0.0, 1.0),
}

# Keys a variant may change. Everything else is baked into the shared
# starling2max_base SDF and the single PX4 airframe.
VARIANTS_KEYS = {"name", "frames", "gazebo.ground_truth_topic"}

# Formatting filters...
def num(v, digits: int = 6) -> str:
    return f"{float(v):.{digits}g}"

def vec(values, digits: int = 6) -> str:
    return " ".join(num(v, digits) for  v in  values)

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
    env.filters.update(num=num, vec=vec, rgba=rgba)
    return env

# Context...
def px4_rotors(p: dict) -> list[dict]:
    """ Rotors in PX4 order. Gazebo rotor_N / motorNumber N and SIM_GZ_EC_FUNC(N+1) all use this same order."""
    km_ratio = p["propulsion"]["prop"]["km_ratio"]
    out = []
    for i, r in enumerate(rotors_for(p, "px4")):
        ca_px, ca_py, _ = r.xyz_frd
        out.append({
            "index": i,
            "name": r.name,
            "xyz": r.xyz,
            "direction": r.spin,
            "color": r.color,
            "ca_px": ca_px,
            "ca_py": ca_py,
            "ca_km": r.spin_sign * km_ratio
        })
    return out

def context(p: dict, base_model: str) -> dict:
    gz = p["gazebo"]
    return {
        **derive(p),          # kf, km, motor_constant, base_mass, base_inertia, prop_*, ...
        "p": p,
        "name": p["name"],
        "base_model": base_model,
        "mesh_scale": MESH_SCALE,
        "body_mesh": p["body"]["mesh"],
        "prop_mesh": p["propulsion"]["prop"]["mesh"],
        "bbox": p["body"]["bbox"],
        "frames": p["frames"] or {},
        "rotors": px4_rotors(p),                       # SDF + airframe
        "genesis_rotors": rotors_for(p, "genesis"),    # URDF prop0..prop3
        "spawn_z": gz["spawn_z"],
        "max_rot_velocity": gz["max_rot_velocity"],
        "min_rot_velocity": gz["min_rot_velocity"],
        "time_constant_up": gz["time_constants"]["up"],
        "time_constant_down": gz["time_constants"]["down"],
        "ground_truth_topic": gz["ground_truth_topic"],
        "airframe_id": p["px4"]["airframe_id"],
        "px4_params": p["px4"]["params"],
    }

def _shared_part(p: dict) -> dict:
    """ p without the keys a variant is allowed to change """
    q = copy.deepcopy(p)
    for dotted in VARIANTS_KEYS:
        *parents, leaf = dotted.split(".")
        node = q
        for part in parents:
            node = node.get(part, {})
        node.pop(leaf, None)
    return q

def _check_variant(p: dict, base: dict) -> None:
    if _shared_part(p) != _shared_part(base):
        raise ValueError(
            f"variant '{p['name']}' changes keys other than {list(VARIANTS_KEYS)}. The Base SDF and PX4 airframe are shared, so Gazebo/PX4 would ignore that change."
        )

# Rendering... writing... checking...
def render_all() -> dict[Path, str]:
    """ {path relative to repo root: file content} for every generated file. """
    env = make_env()
    base = load_params()
    base_model = f"{base['name']}_base"

    def render(template: str, p: dict, **extra) -> str:
        return env.get_template(template).render({**context(p, base_model), **extra})

    files = {
        MODELS / base_model / "model.sdf": render("sdf/base.sdf.j2", base),
        MODELS / base_model / "model.config": render("sdf/model.config.j2", base, model=base_model),
        AIRFRAMES / f"{base['px4']['airframe_id']}_gz_{base['name']}": render("px4/airframe.j2", base),
    }
    for name in model_names():
        p = load_params(name)
        _check_variant(p, base)
        out = MODELS / name
        files[out / "model.sdf"] = render("sdf/flight.sdf.j2", p)
        files[out / "model.urdf"] = render("urdf/model.urdf.j2", p)
        files[out / "model.config"] = render("sdf/model.config.j2", p, model=name)
    return files

def write(root: Path) -> None:
    for rel, text in render_all().items():
        path = root / rel
        if path.exists() and path.read_text() == text:
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
        print(f"wrote {rel}")

def check(root: Path) -> list[Path]:
    """ Print a diff for every stal file and return their paths """
    stale = []
    for rel, text in render_all().items():
        path = root / rel
        old = path.read_text() if path.exists else ""
        if old != text:
            stale.append(rel)
            sys.stdout.writelines(difflib.unified_diff(
                old.splitlines(keepends=True), text.splitlines(keepends=True),
                fromfile=f"a/{rel}", tofile=f"b/{rel}"))
    return stale

def summary(p: dict) -> None:
    d = derive(p)
    print(f"[{p['name']}] kf={d['kf']:.4e} N/RPM^2  km={d['km']:.4e}  "
          f"motorConstant={d['motor_constant']:.4e} N/(rad/s)^2")
    print(f"[{p['name']}] hover={d['hover_velocity']:.0f} rad/s ({d['hover_rpm']:.0f} rpm, "
          f"{d['hover_rpm'] / d['noload_rpm']:.0%} of KV*Vbat)  "
          f"throttle={d['hover_throttle']:.3f}  max T/W={d['max_t2w']:.2f}")

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, default=REPO_ROOT, help="root to write into / check against (default: repo root)")
    parser.add_argument("--check", action="store_true", help="only diff against existing files. Exit 1 if anythig is stale")
    args = parser.parse_args(argv)

    if args.check:
        stale = check(args.out)
        if stale:
            print(f"{len(stale)} generated file(s) out of date. Run 'make generate'", file=sys.stderr)
            return 1
        print("generated files up to date")
        return 0

    write(args.out)
    summary(load_params())

    return 0

if __name__ == "__main__":
    sys.exit(main())