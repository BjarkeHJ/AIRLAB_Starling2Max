
import os
import jinja2
import math

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))

MM_TO_M = 0.001
def scale_mm(xyz: list[float]) -> str:
    return " ".join(str(v * MM_TO_M) for v in xyz)

# Datasheet
MASS = 0.566
MAX_TAKEOFF_MASS = 0.930 
NROT = 4
KV = 1500
VBAT_MAX = 16.8
VBAT_NOM = 14.8
T2W = 2.5  # rough target, not from datasheet - used only as a sanity check below
PROP_RADIUS = 0.09 #[m] (180mm diameter, tri-blade)
# Reaction torque / thrust [m]. Static prop: Q/T = Cp / (2*pi*Ct) * D; Cp/Ct ~ 0.5 gives
# ~0.015 m for D = 0.18 m. Estimate - replace with thrust-stand torque data for the 7040 prop.
PROP_KM_RATIO = 0.015
PROP_CT = 0.11
RHO_AIR = 1.225  # kg/m^3

# Measured Quantities
# Drone BBOX
SIZE_X = 217 * MM_TO_M
SIZE_Y = 287 * MM_TO_M
SIZE_Z = 129 * MM_TO_M

# Motor shafts relative to baselink (from datasheet - z is measured)
PROP0 = scale_mm([95.0, 130.0, 24.0])
PROP1 = scale_mm([95.0, -130.0, 24.0])
PROP2 = scale_mm([-95.0, -130.0, 24.0])
PROP3 = scale_mm([-95.0, 130.0, 24.0])

# Tether attachment points: equilateral triangle centered on base_link (xy),
# 2 cm below base_link, 5 cm side length.
ATTACH = True
ATTACH_SIDE = 0.05 # [m]
ATTACH_Z = -0.02 # [m]

def attachment_points(side: float, z: float) -> list[str]:
    r = side / math.sqrt(3)
    return [
        f"{round(r * math.cos(math.radians(angle)), 12)} {round(r * math.sin(math.radians(angle)), 12)} {z}"
        for angle in (0, 120, 240)
    ]

ATTACH0, ATTACH1, ATTACH2 = attachment_points(ATTACH_SIDE, ATTACH_Z)

def box_inertia(mass: float, size_x: float, size_y: float, size_z: float) -> tuple[float, float, float]:
    ixx = mass / 12.0 * (pow(size_y,2) + pow(size_z,2))
    iyy = mass / 12.0 * (pow(size_x,2) + pow(size_z,2))
    izz = mass / 12.0 * (pow(size_x,2) + pow(size_y,2))
    return ixx, iyy, izz

def coefs(
    mass: float, nrot: int, thrust2weight: float, prop_radius: float, prop_ct: float, prop_km_ratio: float, rho: float
) -> tuple[float, float]:
    g = 9.82

    # T = kf * RPM² ; T = CT * rho * n² * D⁴  (n = RPM/60), so
    # kf = CT * rho * D⁴ / 3600. Derived from prop aerodynamics rather than
    # an assumed motor RPM derate, which is unreliable under heavy load.
    prop_diameter = 2 * prop_radius
    kf = prop_ct * rho * prop_diameter**4 / 3600.0

    # Q = km * RPM² and Q = prop_km_ratio * T, so km = kf * prop_km_ratio
    km = kf * prop_km_ratio

    # Sanity check: RPM (and fraction of no-load KV*Vbat_max) needed to hit
    # the target thrust-to-weight, for comparison against a plausible loaded derate.
    weight = mass * g
    thrust_max_per_rotor = weight * thrust2weight / nrot
    rpm_needed = math.sqrt(thrust_max_per_rotor / kf)
    implied_derate = rpm_needed / (KV * VBAT_MAX)
    print(
        f"[coefs] kf={kf:.4e} km={km:.4e} -> rpm needed for T2W={thrust2weight}: "
        f"{rpm_needed:.0f} rpm ({implied_derate:.0%} of KV*Vbat_max)"
    )

    return kf, km

def generate() -> str:
    ixx, iyy, izz = box_inertia(MASS, SIZE_X, SIZE_Y, SIZE_Z)
    kf, km = coefs(MASS, NROT, T2W, PROP_RADIUS, PROP_CT, PROP_KM_RATIO, RHO_AIR)

    with open(os.path.join(SCRIPT_DIR, "starling2max.urdf.j2")) as f:
        template = jinja2.Template(f.read(), undefined=jinja2.StrictUndefined)

    urdf = template.render(
        kf=kf,
        km=km,
        mass=MASS,
        ixx=ixx,
        iyy=iyy,
        izz=izz,
        size_x=SIZE_X,
        size_y=SIZE_Y,
        size_z=SIZE_Z,
        scale_x=MM_TO_M,
        scale_y=MM_TO_M,
        scale_z=MM_TO_M,
        prop0=PROP0,
        prop1=PROP1,
        prop2=PROP2,
        prop3=PROP3,
        attach=ATTACH,
        attach0=ATTACH0,
        attach1=ATTACH1,
        attach2=ATTACH2,
    )

    with open(os.path.join(SCRIPT_DIR, "starling2max.urdf"), "w") as f:
        f.write(urdf)
    print("Wrote starling2max.urdf")

if __name__ == "__main__":
    generate()