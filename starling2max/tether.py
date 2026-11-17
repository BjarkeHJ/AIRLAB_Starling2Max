# starling2max/tether.py
""" tether + slung payload: attach point 'rings', payload mass/inertia/visuals, plugin params. Only variants with a 'tether' section gets any of this """

import math

from .drone import box_inertia, positive

PAYLOAD_LINK = "payload" # genesis base link and gazebo link name
PAYLOAD_MODEL = "payload" # gazebo nested model wrapping the payload link (gets its own OdometryPublisher)
PAYLOAD_SHAPES = ("box", ) # could be extended in future
PLUGINS = ("TetherPlugin", "TetherVisual") # gz plugin libraries (uav_sim requires_plugins)
PLUGIN_PARAMS= ("rest_length", "stiffness", "damping", "torsional_damping", "gate_tension")
ATTACH_MARKER_RADIUS = 0.01 # [m] for visual only

REQUIRED = {
    "tether.count",
    "tether.rest_length",
    "tether.stiffness",
    "tether.damping",
    "tether.torsional_damping",
    "tether.gate_tension",
    "tether.drone_attach.radius",
    "tether.drone_attach.z",
    "payload.shape",
    "payload.mass",
    "payload.size",
    "payload.attach.radius_frac",
    "payload.spawn_offset",
    "gazebo.payload_odom_topic",
}

# geometry...
def ring(count: int, radius: float, z: float, phase_deg: float = 0.0) -> list[tuple[float, float, float]]:
    """ count points evenly spaced on a horizontal circle, the first at phase_deg from +x """
    phase = math.radians(phase_deg)
    step = 2 * math.pi / count
    return [(radius * math.cos(phase + i * step), radius * math.sin(phase + i * step), z) for i in range(count)]

def drone_attach(p: dict) -> list[tuple[float, float, float]]:
    """ attach points in the drone base_link frame """
    t = p["tether"]
    a = t["drone_attach"]
    return ring(t["count"], a["radius"], a["z"], a.get("phase_deg", 0.0))

def payload_attach(p: dict) -> list[tuple[float, float, float]]:
    """ attach points on the payload top face, in the payload link frame """
    pl = p["payload"]
    sx, sy, sz = pl["size"]
    a = pl["attach"]
    return ring(p["tether"]["count"], a["radius_frac"] * min(sx, sy) / 2, sz / 2, a.get("phase_deg", 0.0))

def attach_frames(p: dict) -> dict:
    """ drone attach points as named frames {attach{i}: {xyz}} """
    if "tether" not in p:
        return {}
    return {f"attach{i}": {"xyz": xyz} for i, xyz in enumerate(drone_attach(p))}

def payload_visuals(size) -> list[dict]:
    """ Box body + top panel + arrow pointing to +x (heading marker). All boxes: {name, xyz, rpy, size, color} """
    sx, sy, sz = size
    panel_h, arrow_h = 0.004, 0.01
    arrow_len = 0.35 * sx
    head = 0.30 * arrow_len
    shaft = arrow_len - head
    z_panel = sz / 2 + panel_h / 2
    z_arrow = sz / 2 + panel_h + arrow_h / 2
    return [
        {"name": "body", "xyz": (0, 0, 0), "rpy": (0, 0, 0), "size": (sx, sy, sz), "color": "blue"},
        {"name": "top_panel", "xyz": (0, 0, z_panel), "rpy": (0, 0, 0), "size": (sx, sy, panel_h), "color": "red"},
        {"name": "arrow_shaft", "xyz": (-arrow_len / 2 + shaft / 2, 0, z_arrow), "rpy": (0, 0, 0),
         "size": (shaft, 0.12 * arrow_len, arrow_h), "color": "green"},
        {"name": "arrow_head", "xyz": (arrow_len / 2 - head / 2, 0, z_arrow), "rpy": (0, 0, math.pi / 4),
         "size": (head, head, arrow_h), "color": "green"},
    ]

def spawn_pose(p: dict) -> tuple[float, ...]:
    """ Payload (nested model) pose in the drone model frame, resting on the ground.
    The model frame is at ground level; base_link sits gazebo.spawn_z above it (sdf/base.sdf.j2). """
    ox, oy = p["payload"]["spawn_offset"]
    sz = p["payload"]["size"][2]
    return (ox, oy, sz / 2, 0.0, 0.0, 0.0)

# deriving...
def derive(p: dict) -> dict:
    """ template context for the tether system """
    if "tether" not in p:
        return {"tether": None}
    t, pl = p["tether"], p["payload"]
    m_d, m_p = p["body"]["mass"], pl["mass"]
    d_att, p_att = drone_attach(p), payload_attach(p)
    return {"tether": {
        "count": t["count"],
        "params": {k: t[k] for k in PLUGIN_PARAMS},
        "cables": list(zip(d_att, p_att)),
        "plugins":  PLUGINS,
        "mass_eff": m_d * m_p / (m_d + m_p), # info/tests, the plugin computes its own from the SDF
        "payload": {
            "model": PAYLOAD_MODEL,
            "link": PAYLOAD_LINK,
            "scoped_link": f"{PAYLOAD_MODEL}::{PAYLOAD_LINK}", # how the plugins reach it from the drone model
            "odom_topic": p["gazebo"]["payload_odom_topic"],
            "mass": m_p,
            "size": tuple(pl["size"]),
            "attach": p_att,
            "inertia": box_inertia(m_p, *pl["size"]),
            "visuals": payload_visuals(pl["size"]),
            "marker_radius": ATTACH_MARKER_RADIUS,
            "spawn_pose": spawn_pose(p),
        },
    }}

# Validating...
def validate(p: dict) -> None:
    """ Tether/payload checks. Required keys are already checked by params.validate. """
    where = p["name"]
    t, pl = p["tether"], p["payload"]

    count = t["count"]
    if isinstance(count, bool) or not isinstance(count, int) or count < 1:
        raise ValueError(f"{where}: tether.count must be an integer >= 1, got {count!r}")
    if pl["shape"] not in PAYLOAD_SHAPES:
        raise ValueError(f"{where}: payload.shape must be one of {PAYLOAD_SHAPES}, got {pl['shape']!r}")
    if len(pl["size"]) != 3:
        raise ValueError(f"{where}: payload.size must have 3 values")
    if len(pl["spawn_offset"]) != 2:
        raise ValueError(f"{where}: payload.spawn_offset must have 2 values (x, y)")

    positive(where, {
        "tether.rest_length": t["rest_length"],
        "tether.stiffness": t["stiffness"],
        "tether.damping": t["damping"],
        "tether.gate_tension": t["gate_tension"],
        "tether.drone_attach.radius": t["drone_attach"]["radius"],
        "payload.mass": pl["mass"],
        **{f"payload.size[{i}]": s for i, s in enumerate(pl["size"])},
    })
    if t["torsional_damping"] < 0:
        raise ValueError(f"{where}: tether.torsional_damping must be >= 0")
    if not 0 < pl["attach"]["radius_frac"] <= 1:
        raise ValueError(f"{where}: payload.attach.radius_frac must be in (0, 1]")

    clash = set(p["frames"] or {}) & set(attach_frames(p))
    if clash:
        raise ValueError(f"{where}: frames {sorted(clash)} collide with generated tether attach frames")

    # Spawn: payload must not overlap the drone, and every cable must start slack
    bx, by, _ = p["body"]["bbox"]
    sx, sy, _ = pl["size"]
    ox, oy = pl["spawn_offset"]
    if abs(ox) < (bx + sx) / 2 and abs(oy) < (by + sy) / 2:
        raise ValueError(f"{where}: payload.spawn_offset {pl['spawn_offset']} overlaps the drone body")

    # Both ends in the model frame: drone attach points are in base_link (spawn_z up),
    # payload attach points in the payload link (at spawn_pose)
    pose = spawn_pose(p)
    base_z = p["gazebo"]["spawn_z"]
    for i, (a, b) in enumerate(zip(drone_attach(p), payload_attach(p))):
        a_model = (a[0], a[1], a[2] + base_z)
        b_model = (b[0] + pose[0], b[1] + pose[1], b[2] + pose[2])
        span = math.dist(a_model, b_model)
        if span >= t["rest_length"]:
            raise ValueError(f"{where}: cable {i} spans {span:.3f} m at spawn >= rest_length {t['rest_length']} m")