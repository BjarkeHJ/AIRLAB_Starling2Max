""" ModalAI Starling 2 Max simulation model: paths, spec, and Genesis entry point. """

from pathlib import Path
from .params import derive, load_params, model_names
from .rotors import rotors_for

__all__ = ["models_dir", "urdf_path", "sdf_path", "load_params", "derive", "model_names", "rotors_for", "genesis_morph"]

PACKAGE = "starling2max_description"

def models_dir() -> Path:
    """ models/ of colcon install if available, else the source tree """
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory(PACKAGE)) / "models"
        if share.is_dir():
            return share
    except Exception:
        pass
    return Path(__file__).resolve().parent.parent / "models"

def urdf_path(variant: str = "starling2max") -> Path:
    path = models_dir() / variant / "model.urdf"
    if not path.is_file():
        raise FileNotFoundError(f"{path} (known models: {model_names()})")
    return path

def sdf_path(variant: str = "starling2max") -> Path:
    return models_dir() / variant / "model.sdf"




# CONSIDER THIS??

# def genesis_morph(variant: str = "starling2max") -> Path:
#     """ gs.morph.Drone for this model; extra kwargs (pos, euler, ...) pass through """
#     import genesis as gs
#     rotors = rotors_for(load_params(variant), "genesis")
#     return gs.morphs.Drone(
#         file=str(urdf_path(variant)),
#         propellers_link_name=[f"prop{i}_link" for i in range(len(rotors))]
#         propellers_spin=[r.spin_sign for r in rotors],
#         **kwargs,
#     )