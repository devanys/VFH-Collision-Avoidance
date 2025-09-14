import pybullet as p
import pybullet_data
import numpy as np
import time
import os

DT = 0.05

LINEAR_SPEED = 2.0            
WHEEL_RADIUS = 0.1            
HALF_WHEELBASE = 0.25         
TOLERANCE = 0.2               
TARGET = np.array([10.0, 10.0])
NUM_OBSTACLES = 5
OBSTACLE_RADIUS = 0.5
OBSTACLE_SPEED = 0.01
OBSTACLE_AREA_MIN = 1.0
OBSTACLE_AREA_MAX = 9.0

NUM_BEAMS = 40
SENSOR_RANGE = 8.0
FIELD_OF_VIEW = np.pi / 3
AVOIDANCE_THRESHOLD = 1.5
SMOOTHING_FACTOR = 0.8

robot_path = "C:/Users/Devan/Downloads/pybullet_robots-master/pybullet_robots-master/data/turtlebot.urdf"

def find_wheel_joints(robot):
   
    n = p.getNumJoints(robot)
    left = []
    right = []
    candidates = []
    for i in range(n):
        info = p.getJointInfo(robot, i)
        name = info[1].decode('utf-8').lower()
        candidates.append((i, name))
        if 'wheel' in name or 'left' in name or 'right' in name:
            pass
    
    for i, name in candidates:
        if 'left' in name and 'wheel' in name:
            left.append(i)
        elif 'right' in name and 'wheel' in name:
            right.append(i)
    if not left and not right:
        for i, name in candidates:
            jinfo = p.getJointInfo(robot, i)
            jtype = jinfo[2]
            if jtype == p.JOINT_REVOLUTE:
                if len(left) <= len(right):
                    left.append(i)
                else:
                    right.append(i)

    if not left or not right:
        left = []
        right = []
        for i, name in candidates:
            jinfo = p.getJointInfo(robot, i)
            if jinfo[2] == p.JOINT_REVOLUTE:
                if len(left) <= len(right):
                    left.append(i)
                else:
                    right.append(i)
    left.sort(); right.sort()
    return left, right

def pure_pursuit_angle(target, x, y, yaw):
    dx = target[0] - x
    dy = target[1] - y
    desired = np.arctan2(dy, dx)
    alpha = np.arctan2(np.sin(desired - yaw), np.cos(desired - yaw))
    return alpha, desired

def lidar_beams_from_pose(px, py, yaw):
    angles_local = np.linspace(-FIELD_OF_VIEW, FIELD_OF_VIEW, NUM_BEAMS)
    angles_world = angles_local + yaw
    from_positions = []
    to_positions = []
    z = 0.15
    for ang in angles_world:
        from_positions.append((px, py, z))
        to_positions.append((px + SENSOR_RANGE*np.cos(ang), py + SENSOR_RANGE*np.sin(ang), z))
    return from_positions, to_positions, angles_world

def vfh_using_rays(px, py, yaw):
    from_pos, to_pos, angles = lidar_beams_from_pose(px, py, yaw)
    results = p.rayTestBatch(from_pos, to_pos)
    distances = np.full(len(results), SENSOR_RANGE)
    for i, r in enumerate(results):
        hit_obj = r[0]
        hit_frac = r[2]  
        if hit_obj >= 0 and hit_frac >= 0:
            distances[i] = hit_frac * SENSOR_RANGE
    safe_idx = int(np.argmax(distances))
    safe_angle = angles[safe_idx]
    min_dist = float(np.min(distances))
    return safe_angle, min_dist, distances, angles

def set_wheel_velocities(robot, left_joints, right_joints, v_linear, w_ang):
    vl = (v_linear - w_ang * HALF_WHEELBASE) / WHEEL_RADIUS
    vr = (v_linear + w_ang * HALF_WHEELBASE) / WHEEL_RADIUS
    for j in left_joints:
        p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, targetVelocity=vl, force=200)
    for j in right_joints:
        p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, targetVelocity=vr, force=200)
    return vl, vr

def update_obstacles_positions(obs_pos, obs_dirs):
    obs_pos += obs_dirs * OBSTACLE_SPEED
    for i in range(len(obs_pos)):
        if obs_pos[i,0] < OBSTACLE_AREA_MIN + OBSTACLE_RADIUS or obs_pos[i,0] > OBSTACLE_AREA_MAX - OBSTACLE_RADIUS:
            obs_dirs[i,0] *= -1
        if obs_pos[i,1] < OBSTACLE_AREA_MIN + OBSTACLE_RADIUS or obs_pos[i,1] > OBSTACLE_AREA_MAX - OBSTACLE_RADIUS:
            obs_dirs[i,1] *= -1
        p.resetBasePositionAndOrientation(obstacle_ids[i], [float(obs_pos[i,0]), float(obs_pos[i,1]), OBSTACLE_RADIUS], [0,0,0,1])
    return obs_pos, obs_dirs

if not os.path.exists(robot_path):
    raise FileNotFoundError("robot_path tidak ditemukan. Ganti path URDF TurtleBot-mu di variabel robot_path.")

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0,0,-9.81)
p.setTimeStep(DT)

p.loadURDF("plane.urdf")

start_pos = [0, 0, 0.1]
start_ori = p.getQuaternionFromEuler([0,0,0])
robot = p.loadURDF(robot_path, start_pos, start_ori, useFixedBase=False, flags=p.URDF_USE_INERTIA_FROM_FILE)


left_joints, right_joints = find_wheel_joints(robot)
print("Detected left wheel joints:", left_joints)
print("Detected right wheel joints:", right_joints)
if not left_joints or not right_joints:
    print("Warning: wheel joints detection maybe wrong. Print joint list for manual check:")
    for i in range(p.getNumJoints(robot)):
        print(i, p.getJointInfo(robot, i)[1].decode())

tvis = p.createVisualShape(p.GEOM_SPHERE, radius=0.06, rgbaColor=[0.1,0.2,0.9,1])
p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=tvis, basePosition=[TARGET[0], TARGET[1], 0.06])

obstacle_ids = []
obs_pos_list = []
obs_dir_list = []
rng = np.random.default_rng(1234)
for i in range(NUM_OBSTACLES):
    x = rng.uniform(1+OBSTACLE_RADIUS, 9-OBSTACLE_RADIUS)
    y = rng.uniform(1+OBSTACLE_RADIUS, 9-OBSTACLE_RADIUS)
    col = p.createCollisionShape(p.GEOM_SPHERE, radius=OBSTACLE_RADIUS)
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=OBSTACLE_RADIUS, rgbaColor=[0.7,0.1,0.7,1])
    oid = p.createMultiBody(baseMass=0.2, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis, basePosition=[x,y,OBSTACLE_RADIUS])
    obstacle_ids.append(oid)
    obs_pos_list.append([x,y])
    d = rng.standard_normal(2); d /= np.linalg.norm(d)
    obs_dir_list.append(d)
obs_pos = np.array(obs_pos_list)
obs_dir = np.array(obs_dir_list)


debug_ids = []


p.resetDebugVisualizerCamera(cameraDistance=12, cameraYaw=40, cameraPitch=-35, cameraTargetPosition=[4,4,0])


for step in range(5000):
    
    pos, orn = p.getBasePositionAndOrientation(robot)
    yaw = p.getEulerFromQuaternion(orn)[2]
    px, py, pz = pos
    dist_to_target = np.hypot(TARGET[0]-px, TARGET[1]-py)


    safe_angle, min_dist, distances, angles = vfh_using_rays(px, py, yaw)


    alpha, desired_heading = pure_pursuit_angle(TARGET, px, py, yaw)


    if min_dist < AVOIDANCE_THRESHOLD:
        ref_heading = safe_angle
    else:
        ref_heading = desired_heading

    angle_err = np.arctan2(np.sin(ref_heading - yaw), np.cos(ref_heading - yaw))

    K_ang = 1.2
    w = np.clip(K_ang * angle_err, -2.0, 2.0)


    forward = LINEAR_SPEED * max(0.05, np.cos(angle_err))


    vl, vr = set_wheel_velocities(robot, left_joints, right_joints, forward, w)


    if step % 50 == 0:
        print(f"Step {step} | pos=({px:.2f},{py:.2f}) yaw={yaw:.2f} | dist_target={dist_to_target:.2f} min_obs={min_dist:.2f} | vl={vl:.2f} vr={vr:.2f}")


    obs_pos, obs_dir = update_obstacles_positions(obs_pos, obs_dir)

    for did in debug_ids:
        try:
            p.removeUserDebugItem(did)
        except Exception:
            pass
    debug_ids = []
    from_pos, to_pos, angs = lidar_beams_from_pose(px, py, yaw)
    for i, (f, t) in enumerate(zip(from_pos, to_pos)):
        end = t
        if distances[i] < SENSOR_RANGE:
            end = (f[0] + distances[i]*np.cos(angs[i]), f[1] + distances[i]*np.sin(angs[i]), f[2])
        color = [0,1,0] if distances[i] > 0.9*SENSOR_RANGE else [1,0,0]
        lid = p.addUserDebugLine(f, end, lineColorRGB=color, lifeTime=DT*1.5, lineWidth=1.2)
        debug_ids.append(lid)


    if dist_to_target < TOLERANCE:
        print("Target reached at step", step)

        set_wheel_velocities(robot, left_joints, right_joints, 0.0, 0.0)
        break

    p.stepSimulation()
    time.sleep(DT)

print("Done, disconnecting.")
p.disconnect()

