"""Example simulation of a walking Cassie."""

import argparse

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

# If I know the namespaces...
from pydrake.common import FindResourceOrThrow
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    DiscreteContactSolver,
)
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator

# Joint teleop with Meshcat. See
# drake/examples/manipulation_station/joint_teleop.py
from pydrake.multibody.meshcat import JointSliders

# ... and if I don't.
from pydrake.all import (MeshcatVisualizer, StartMeshcat)

def xyz_rpy_deg(xyz, rpy_deg):
    """Shorthand for defining a pose."""
    rpy_deg = np.asarray(rpy_deg)
    return RigidTransform(RollPitchYaw(rpy_deg * np.pi / 180), xyz)

def LeftRodOnHeel(plant):
    """Returns the pair (position, frame) for the rod on the heel."""
    return np.array([.11877, -.01, 0.0]), plant.GetFrameByName("heel_spring_left")

def LeftRodOnThigh(plant):
  return np.array([0.0, 0.0, 0.045]), plant.GetFrameByName("thigh_left");

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
    plant.set_discrete_contact_solver(DiscreteContactSolver.kSap)
        
    cassie = Parser(plant=plant).AddModelFromFile(file_name)

    # Weld pelvis so it doesn't fall.
    plant.WeldFrames(
        frame_on_parent_F=plant.world_frame(),
        frame_on_child_M=plant.GetFrameByName("pelvis", cassie),
        X_FM=xyz_rpy_deg([0, 1.5, 0], [0, 0, 0]),
    )

    # Add Cassie's distance constraints.
    heel_spring_left = LeftRodOnHeel(plant)[1].body()
    p_HeelAttachmentPoint = LeftRodOnHeel(plant)[0]
    thigh_left = LeftRodOnThigh(plant)[1].body()
    p_ThighAttachmentPoint = LeftRodOnThigh(plant)[0]

    kAchillesLength = 0.5012
    kAchillesStiffness = 1.0e6
    kAchillesDamping = 2.0e3
    print(f"heel_spring_left = {heel_spring_left.name()}")
    print(f"p_HeelAttachmentPoint = {p_HeelAttachmentPoint}")
    constraint_index = plant.AddDistanceConstraint(
          heel_spring_left, p_HeelAttachmentPoint,
          thigh_left, p_ThighAttachmentPoint,
          kAchillesLength, kAchillesStiffness, kAchillesDamping)
    print(f"constraint_index = {constraint_index}")

    plant.Finalize()

    # Add joint teleop
    teleop = builder.AddSystem(JointSliders(meshcat, plant))

    # Add viz.
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    # Done defining the diagram.
    diagram = builder.Build()

    # Publish initial visualization
    Simulator(diagram).Initialize()

    input("Press Enter to continue...")

    # Instantiate simulator and get context.
    simulator = Simulator(diagram)
    simulator.set_publish_every_time_step(False)
    simulator.set_target_realtime_rate(args.target_realtime_rate)

    #context = simulator.get_mutable_context()    
    #plant_context = plant.GetMyMutableContextFromRoot(context)

    simulator.Initialize()
    simulator.AdvanceTo(args.simulation_time)


if __name__ == "__main__":
    main()
