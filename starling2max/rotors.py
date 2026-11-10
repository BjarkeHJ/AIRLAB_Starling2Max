# starling2max/rotos.py
from dataclasses import dataclass

QUADRANTS: dict[str, tuple[int, int]] = {
    "front_left": (1, 1),
    "front_right": (1, -1),
    "rear_left": (-1, 1),
    "rear_right": (-1, -1)
}
DIAGONALS = (("front_left", "rear_right"), ("front_right", "rear_left"))
SPINS = ("cw", "ccw")

@dataclass(frozen=True)
class Rotor:
    name: str
    xyz: tuple[float, float, float] # FLU
    spin: str # "cw" | "ccw", seen from above
    color: str

    @property
    def xyz_frd(self) -> tuple[float, float, float]:
        """ Same position in FRD (PX4 Control allocation). """
        x, y, z = self.xyz
        return x, -y, -z

    @property
    def spin_sign(self) -> int:
        """ +1 for ccw, -1 for cw (Genesis propellers_spin, sign of PX4 CA_ROTORn_KM). """
        return 1 if self.spin == "ccw" else -1

def all_rotors(p: dict) -> dict[str, Rotor]:
    """ Every rorot in the spec, keyed by name. Validates the propulsion block. """
    prop = p["propulsion"]
    spec = prop["rotors"]

    if set(spec) != set(QUADRANTS):
        raise ValueError(f"propulsion.rotors must be exactly {sorted(QUADRANTS)}, got {sorted(spec)}")

    ox, oy, oz = prop["rotor_offset"]
    if ox <= 0 or oy <= 0:
        raise ValueError("propulsion.rotor_offset x/y must be positive. Signs come form the rotor name")

    rotors = {}
    for name, (sx, sy) in QUADRANTS.items():
        r = spec[name]
        if r["spin"] not in SPINS:
            raise ValueError(f"rotor {name}: spin must be one of {SPINS}, got {r['spin']!r}")
        rotors[name] = Rotor(name, (sx * ox, sy * oy, oz), r["spin"], r["color"])

    for a, b in DIAGONALS:
        if rotors[a].spin != rotors[b].spin:
            raise ValueError(f"{a} and {b} are diagonal and must spin the same way")

    if rotors["front_left"].spin == rotors["front_right"].spin:
        raise ValueError("adjacent rotors must spin in opposite directions")

    return rotors

def rotors_for(p: dict, target: str) -> list[Rotor]:
    """ Return rotos in the order the given 'target' expects """
    rotors = all_rotors(p)
    try:
        order = p[target]["rotor_order"]
    except KeyError:
        raise KeyError(f"spec has no {target}.rotor_order") from None

    if sorted(order) != sorted(rotors):
        raise ValueError(f"{target}.rotor_order must list eac of {sorted(rotors)} exactly once, got {order}")

    return [rotors[name] for name in order]