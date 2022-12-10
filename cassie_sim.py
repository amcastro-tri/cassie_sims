"""Example simulation of a walking Cassie."""

import argparse

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

# If I know the namespaces...
from pydrake.common import FindResourceOrThrow
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.tree import (JointIndex, JointActuatorIndex)
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    DiscreteContactSolver,
)
from pydrake.multibody.parsing import Parser
from pydrake.systems.controllers import (
    PidController,
    PidControlledSystem)    
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.primitives import (
    MatrixGain,
    ConstantVectorSource,
    Multiplexer)

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
  return np.array([0.0, 0.0, 0.045]), plant.GetFrameByName("thigh_left") 

def RightRodOnThigh(plant):
  return np.array([0.0, 0.0, -0.045]), plant.GetFrameByName("thigh_right")

def RightRodOnHeel(plant):
  return np.array([0.11877, -.01, 0.0]), plant.GetFrameByName("heel_spring_right")

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
    #file_name = "./Cassie/cassie_v2.urdf"
    file_name = "./Cassie/cassie_fixed_springs.urdf"
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
    kAchillesLength = 0.5012
    kAchillesStiffness = 1.0e6
    kAchillesDamping = 2.0e3

    # Left
    heel_spring_left = LeftRodOnHeel(plant)[1].body()
    p_HeelAttachmentPoint = LeftRodOnHeel(plant)[0]
    thigh_left = LeftRodOnThigh(plant)[1].body()
    p_ThighAttachmentPoint = LeftRodOnThigh(plant)[0]    
    print(f"heel_spring_left = {heel_spring_left.name()}")
    print(f"p_HeelAttachmentPoint = {p_HeelAttachmentPoint}")
    constraint_index = plant.AddDistanceConstraint(
          heel_spring_left, p_HeelAttachmentPoint,
          thigh_left, p_ThighAttachmentPoint,
          kAchillesLength, kAchillesStiffness, kAchillesDamping)

    # Right
    heel_spring_right = RightRodOnHeel(plant)[1].body()
    p_RightHeelAttachmentPoint = RightRodOnHeel(plant)[0]
    thigh_right = RightRodOnThigh(plant)[1].body()
    p_RightThighAttachmentPoint = RightRodOnThigh(plant)[0]    
    print(f"heel_spring_right = {heel_spring_right.name()}")
    print(f"p_RightHeelAttachmentPoint = {p_RightHeelAttachmentPoint}")      
    constraint_index = plant.AddDistanceConstraint(
          heel_spring_right, p_RightHeelAttachmentPoint,
          thigh_right, p_RightThighAttachmentPoint,
          kAchillesLength, kAchillesStiffness, kAchillesDamping)
    print(f"num_constraints = {plant.num_constraints()}")

    plant.Finalize()
    nq = plant.num_positions()
    nv = plant.num_velocities()
    nu = plant.num_actuators()
    print(f"nq = {nq}, nv = {nv}, nu = {nu}")

    # Add joint teleop
    lower_limits = plant.GetPositionLowerLimits()
    upper_limits = plant.GetPositionUpperLimits()
    q0 = 0.5 * (lower_limits + upper_limits)
    print(f"lower_limits = {lower_limits}")
    print(f"upper_limits = {upper_limits}")
    print(f"q0 = {q0}")
    teleop = builder.AddSystem(JointSliders(meshcat, plant, q0, lower_limits, upper_limits))

    actuated_joints = []
    state_selector = np.zeros((2*nu, nq + nv))
    u = 0
    for a in range(0, plant.num_actuators()):
        actuator = plant.get_joint_actuator(JointActuatorIndex(a))
        j = actuator.joint().index()
        print(f"actuator's name: {actuator.name()}")
        actuated_joints.append(j)
        dof = actuator.joint().velocity_start()        
        state_selector[u, dof] = 1
        state_selector[nu+u, nq+dof] = 1
        u+=1
    print(f"actuated_joints = {actuated_joints}")
    print(f"state_selector = {state_selector.shape}")

    # Desired state
    zero_vs = builder.AddSystem(ConstantVectorSource(np.zeros(nq)))
    mux = builder.AddSystem(Multiplexer(input_sizes=[nq, nq]))
    builder.Connect(teleop.get_output_port(), mux.get_input_port(0))
    builder.Connect(zero_vs.get_output_port(0), mux.get_input_port(1))

    actuated_states_selector = builder.AddSystem(MatrixGain(state_selector))
    builder.Connect(mux.get_output_port(),
        actuated_states_selector.get_input_port())

    # Add controller.    
    Kp = 10 * np.ones(nu)
    Kd = 1 * np.ones(nu)
    Ki = np.zeros(nu)
    print(f"Kp = {Kp}")
    print(f"Ki = {Ki}")
    #connect_result=PidControlledSystem.ConnectController(
    #    plant_input=plant.get_applied_generalized_force_input_port(), 
    #    plant_output=plant.get_state_output_port(),
    #    Kp=Kp, Ki=Ki, Kd=Kd, builder=builder)
    controller = builder.AddSystem(PidController(state_selector, Kp, Ki, Kd))
    builder.Connect(plant.get_state_output_port(),
                    controller.get_input_port_estimated_state())

    # TODO: not all joints are actuated!! we should use plant.get_actuation_input_port()                        
    builder.Connect(controller.get_output_port_control(),
                    plant.get_actuation_input_port())                        
    builder.Connect(actuated_states_selector.get_output_port(),

                    controller.get_input_port_desired_state())

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
