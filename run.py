__author__ = "Antoine Richard"
__copyright__ = (
    "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
)
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from omegaconf import DictConfig, OmegaConf, ListConfig
from src.configurations import configFactory
from src.environments_wrappers import startSim

from typing import Dict, List
import logging
import hydra
import sys

# Global list to store Isaac Sim arguments temporarily
ISAAC_ARGS = []

numba_logger = logging.getLogger("numba")
numba_logger.setLevel(logging.WARNING)
matplotlib_logger = logging.getLogger("matplotlib")
matplotlib_logger.setLevel(logging.WARNING)


def resolve_tuple(*args):
    return tuple(args)


OmegaConf.register_new_resolver("as_tuple", resolve_tuple)


def omegaconfToDict(d: DictConfig) -> Dict:
    """Converts an omegaconf DictConfig to a python Dict, respecting variable interpolation.

    Args:
        d (DictConfig): OmegaConf DictConfig.

    Returns:
        Dict: Python dict."""

    if isinstance(d, DictConfig):
        ret = {}
        for k, v in d.items():
            if isinstance(v, DictConfig):
                ret[k] = omegaconfToDict(v)
            elif isinstance(v, ListConfig):
                ret[k] = [omegaconfToDict(i) for i in v]
            else:
                ret[k] = v
    elif isinstance(d, ListConfig):
        ret = [omegaconfToDict(i) for i in d]
    else:
        ret = d

    return ret


def instantiateConfigs(cfg: dict) -> dict:
    """
    Instantiates the configurations. That is if the name of the configuration is in the instantiable_configs list,
    it will create an instance of it.
    """

    instantiable_configs = configFactory.getConfigs()

    ret = {}
    for k, v in cfg.items():
        if isinstance(v, dict):
            if k in instantiable_configs:
                ret[k] = configFactory(k, **v)
            else:
                ret[k] = instantiateConfigs(v)
        else:
            ret[k] = v
    return ret


@hydra.main(config_name="config", config_path="cfg", version_base="1.1")
def run(cfg: DictConfig):
    print("DEBUG: run() called", flush=True)
    # Restore arguments so SimulationApp (Isaac Sim) can see them
    sys.argv.extend(ISAAC_ARGS)

    print(f"DEBUG: Mode Name={cfg.mode.name}", flush=True)

    cfg = omegaconfToDict(cfg)
    cfg = instantiateConfigs(cfg)
    print("DEBUG: Instantiation complete. Starting Sim...", flush=True)
    SM, simulation_app = startSim(cfg)
    print("DEBUG: Sim Started. Running loop...", flush=True)

    SM.run_simulation()
    simulation_app.close()


if __name__ == "__main__":
    # Separate Hydra arguments (standard) from Isaac Sim arguments (start with --/)
    # This prevents Hydra from crashing on unknown arguments meant for the simulator
    hydra_args = [arg for arg in sys.argv if not arg.startswith("--/")]
    isaac_args = [arg for arg in sys.argv if arg.startswith("--/")]

    # Store isaac args to be restored inside the run function
    ISAAC_ARGS.extend(isaac_args)

    # trick hydra into thinking only its args are present
    sys.argv = hydra_args

    run()
