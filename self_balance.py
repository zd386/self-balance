

import csv
import os
import random
import time

import pybullet as p
import pybullet_data

from pid_controller import PID

# ---------------------------------------------------------------- settings
URDF_PATH = os.path.join(os.path.dirname(__file__), "balancebot.urdf")
SIM_HZ = 240                      # PyBullet's default physics step rate
DT = 1.0 / SIM_HZ
RUN_SECONDS = 40
START_HEIGHT = 0.05               # matches the wheel radius in the URDF
FALLEN_THRESHOLD = 1.0            # radians (~57 deg) -> consider it "fallen"


PID_KP = 8.0
PID_KI = 0.5
PID_KD = 0.5
MAX_WHEEL_TORQUE = 3.0   

WHEEL_JOINTS = ["left_wheel_joint", "right_wheel_joint"]


def get_joint_index_map(body_id):
    """PyBullet identifies joints by integer index, not name, so build a
    name -> index lookup once after loading the URDF."""
    name_to_index = {}
    for i in range(p.getNumJoints(body_id)):
        joint_info = p.getJointInfo(body_id, i)
        joint_name = joint_info[1].decode("utf-8")
        name_to_index[joint_name] = i
    return name_to_index


def get_pitch(body_id):
    """Returns the robot's forward/backward tilt in radians. 0 = upright."""
    _, orientation = p.getBasePositionAndOrientation(body_id)
    roll, pitch, yaw = p.getEulerFromQuaternion(orientation)
    return pitch


def apply_random_push(body_id):
    """Simulates someone bumping the robot: a short horizontal force at the
    chassis. linkIndex=-1 means 'apply to the base link'."""
    force_magnitude = random.uniform(3.0, 6.0)
    direction = random.choice([-1.0, 1.0])
    force = [force_magnitude * direction, 0, 0]
    position = [0, 0, 0.2]  # roughly chest-height on the chassis
    p.applyExternalForce(body_id, -1, force, position, p.WORLD_FRAME)


def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(DT)
    p.setRealTimeSimulation(0)  # we step manually so logging/timing stays exact

    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(URDF_PATH, [0, 0, START_HEIGHT])

    joints = get_joint_index_map(robot_id)
    left_wheel = joints[WHEEL_JOINTS[0]]
    right_wheel = joints[WHEEL_JOINTS[1]]

    # PyBullet's revolute/continuous joints have a built-in velocity motor
    # (like brake friction) enabled by default. We disable it (force=0) so
    # we can drive the wheels with pure torque instead -- torque control is
    # what makes this a genuine reaction-wheel-style balance problem: the
    # motor's reaction torque on the chassis is what corrects the tilt.
    p.setJointMotorControl2(robot_id, left_wheel, p.VELOCITY_CONTROL, force=0)
    p.setJointMotorControl2(robot_id, right_wheel, p.VELOCITY_CONTROL, force=0)

    pid = PID(PID_KP, PID_KI, PID_KD, setpoint=0.0,
              output_limits=(-MAX_WHEEL_TORQUE, MAX_WHEEL_TORQUE))

    log_rows = []
    total_steps = int(RUN_SECONDS * SIM_HZ)
    next_random_push_step = random.randint(SIM_HZ * 1, SIM_HZ * 2)

    print("Simulation running. Close the PyBullet window to stop early.")

    for step in range(total_steps):
        pitch = get_pitch(robot_id)

        # If it's fully tipped over, there's nothing left for the
        # controller to do -- stop early instead of logging junk data.
        if abs(pitch) > FALLEN_THRESHOLD:
            print(f"Robot fell over at t={step * DT:.2f}s (pitch={pitch:.2f} rad). Stopping.")
            break

        # Negative sign: if the robot tips forward (+pitch), applying
        # positive wheel torque would spin the wheels in the direction that
        # ACCELERATES the fall (reaction torque pushes the chassis further
        # over) -- verified empirically in simulation. So the correcting
        # torque is the negative of the raw PID output.
        wheel_torque = -pid.compute(pitch, DT)

        p.setJointMotorControl2(robot_id, left_wheel, p.TORQUE_CONTROL, force=wheel_torque)
        p.setJointMotorControl2(robot_id, right_wheel, p.TORQUE_CONTROL, force=wheel_torque)

        # periodic disturbance so you can see/prove recovery, not just
        # static balance
        if step == next_random_push_step:
            apply_random_push(robot_id)
            next_random_push_step = step + random.randint(SIM_HZ * 3, SIM_HZ * 6)

        log_rows.append({
            "time": round(step * DT, 4),
            "pitch_rad": pitch,
            "pid_output": wheel_torque,
        })

        p.stepSimulation()
        time.sleep(DT)  # real-time playback so the GUI is watchable

    p.disconnect()

    log_path = os.path.join(os.path.dirname(__file__), "balance_log.csv")
    with open(log_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["time", "pitch_rad", "pid_output"])
        writer.writeheader()
        writer.writerows(log_rows)

    print(f"Logged {len(log_rows)} steps to {log_path}")
    print("Run 'python plot_results.py' to graph the tilt response.")


if __name__ == "__main__":
    main()
