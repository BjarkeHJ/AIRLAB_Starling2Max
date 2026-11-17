# starling2max/generate.py

import argparse
import copy
import sys
import os
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


from starling2max import drone, tether
from starling2max.params import REPO_ROOT, load_params, model_names
from starling2max.render import check, make_env, write
from starling2max.rotors import rotors_for

TEMPLATE_DIR = Path(__file__).resolve().parent / "templates"
MODELS = Path("models")
AIRFRAMES = Path("px4") / "airframes"
MESH_SCALE = (0.001, 0.001, 0.001)  # meshes are in mm

# Keys a variant may change. Everything else is baked into the shared
# starling2max_base SDF and the single PX4 airframe.
VARIANTS_KEYS = {"name", "frames",  "tether", "payload", "gazebo.drone_odom_topic", "gazebo.payload_odom_topic"}


def context(p: dict, base_model: str) -> dict:
    gz = p["gazebo"]
    return {
        **drone.derive(p),
        **tether.derive(p),
        "p": p,
        "name": p["name"],
        "base_model": base_model,
        "mesh_scale": MESH_SCALE,
        "body_mesh": p["body"]["mesh"],
        "prop_mesh": p["propulsion"]["prop"]["mesh"],
        "bbox": p["body"]["bbox"],
        "frames": {**(p["frames"] or {}), **tether.attach_frames(p)},
        "rotors": drone.px4_rotors(p),
        "genesis_rotors": rotors_for(p, "genesis"),
        "spawn_z": gz["spawn_z"],
        "max_rot_velocity": gz["max_rot_velocity"],
        "min_rot_velocity": gz["min_rot_velocity"],
        "time_constant_up": gz["time_constants"]["up"],
        "time_constant_down": gz["time_constants"]["down"],
        "drone_odom_topic": gz["drone_odom_topic"],
        "payload_odom_topic": gz.get("payload_odom_topic"), # tether variants only (tether.REQUIRED)
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
        files[out / "uav_sim.yaml"] = render("uav_sim.yaml.j2", p)
        if "tether" in p:
            files[out / "payload.urdf"] = render("urdf/payload.urdf.j2", p)
    return files

def summary(p: dict) -> None:
    d = drone.derive(p)
    t = tether.derive(p)["tether"]
    print(f"[{p['name']}] kf={d['kf']:.4e} N/RPM^2  km={d['km']:.4e}  "
          f"motorConstant={d['motor_constant']:.4e} N/(rad/s)^2")
    print(f"[{p['name']}] hover={d['hover_velocity']:.0f} rad/s ({d['hover_rpm']:.0f} rpm, "
          f"{d['hover_rpm'] / d['noload_rpm']:.0%} of KV*Vbat)  "
          f"throttle={d['hover_throttle']:.3f}  max T/W={d['max_t2w']:.2f}")
    if t:
        print(f"[{p['name']}] tether: {t['count']} cables   L0={t['params']['rest_length']} m "
              f"payload={t['payload']['mass']} kg   mass_eff={t['mass_eff']:.3f} kg")

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, default=REPO_ROOT, help="root to write into / check against (default: repo root)")
    parser.add_argument("--check", action="store_true", help="only diff against existing files. Exit 1 if anythig is stale")
    args = parser.parse_args(argv)

    if args.check:
        stale = check(render_all(), args.out)
        if stale:
            print(f"{len(stale)} generated file(s) out of date. Run 'make generate'", file=sys.stderr)
            return 1
        print("generated files up to date")
        return 0
    
    write(render_all(), args.out)
    summary(load_params())

    return 0

if __name__ == "__main__":
    sys.exit(main())