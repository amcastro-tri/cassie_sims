"""Example simulation of a walking Cassie."""

import argparse

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

# If I know the namespaces...
from pydrake.common import FindResourceOrThrow
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator

# ... and if I don't.
from pydrake.all import (MeshcatVisualizer, StartMeshcat)

def xyz_rpy_deg(xyz, rpy_deg):
    """Shorthand for defining a pose."""
    rpy_deg = np.asarray(rpy_deg)
    return RigidTransform(RollPitchYaw(rpy_deg * np.pi / 180), xyz)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target_realtime_rate", type=float, default=1.0,
        help="Desired rate relative to real time.  See documentation for "
             "Simulator::set_target_realtime_rate() for details.")
    parser.add_argument(
        "--simulation_time", type=float, default=10.0,
        help="Desired duration of the simulation in seconds.")
    parser.add_argument(
        "--time_step", type=float, default=0.005,
        help="If greater than zero, the plant is modeled as a system with "
             "discrete updates and period equal to this time_step. "
             "If 0, the plant is modeled as a continuous system.")
    args = parser.parse_args()

    # Start the visualizer.
    meshcat = StartMeshcat()

    # Add robot.    
    file_name = "./cassie_v2.urdf"
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder=builder, time_step=args.time_step)
    cassie = Parser(plant=plant).AddModelFromFile(file_name)

    # Weld pelvis so it doesn't fall.
    plant.WeldFrames(
        frame_on_parent_F=plant.world_frame(),
        frame_on_child_M=plant.GetFrameByName("pelvis", cassie),
        X_FM=xyz_rpy_deg([0, 1.5, 0], [0, 0, 0]),
    )

    plant.Finalize()

    # Add viz.
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    # Done defining the diagram.
    diagram = builder.Build()

    # Publish initial visualization
    Simulator(diagram).Initialize()

    input("Press Enter to continue...")

    # Instantiate simulator and get context.
    #simulator = Simulator(diagram)
    #context = simulator.get_mutable_context()
    #station_context = station.GetMyMutableContextFromRoot(context)
    #plant_context = plant.GetMyMutableContextFromRoot(context)

if __name__ == "__main__":
    main()
