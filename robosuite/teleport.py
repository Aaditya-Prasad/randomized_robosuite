from robosuite.utils.transform_utils import mat2euler, quat2mat, euler2mat, get_orientation_error, mat2quat, quat2axisangle
import numpy as np

def sample_fn():
    """
    This is a sample function to show how to use tp
    """
    env = DummyEnv()

    delta_zrot = np.random.uniform(-0.1, 0.1)
    delta_rot = np.array([0, 0, delta_zrot]) 

    #tp use abs pos
    obs = env.reset()
    cur_ee_pos = obs['robot0_eef_pos']
    des_ee_pos = np.array([0.5, 0.5, 0.5]) #this can be drawn from a distribution
    obs = teleport(env, cur_ee_pos, des_ee_pos, delta_rot)

    #tp use delta pos
    dx = np.random.uniform(-0.1, 0.1)
    dy = np.random.uniform(-0.1, 0.1)
    dz = np.random.uniform(-0.1, 0.1)
    dpos = np.array([dx, dy, dz])

    obs = teleport(env, None, None, delta_rot, dpos=dpos)

    #go and run dome/acta/whatever
    eval()



def get_dpos(cur_ee_pos, des_ee_pos) -> np.ndarray:
    return des_ee_pos - cur_ee_pos



def teleport(env, cur_ee_pos, des_ee_pos, delta_rot, dpos = None):
    """
    Teleports the robot to a specific position. 
    Uses a delta rotation as well as either absolute current/desired pos or delta pos.
    This requires an IK controller!

    Args: 
        env: an instantiated env you want to change
        cur_ee_pos: current ee pose from obs.
        des_ee_pos: desired ee pose.
        delta_rot: delta rotation in euler angles
        dpos: delta position. If you pass this in, cur_ee_pos and des_ee_pos can be None, tp will just use dpos to move the robot
    """
    
    if dpos is None:
        dpos = get_dpos(cur_ee_pos, des_ee_pos)

    drot = euler2mat(delta_rot)

    
    robot = env.robots[0]
    controller = robot.controller
    controller.converge_steps=100

    jpos = controller.joint_positions_for_eef_command(dpos, drot, update_targets=True)
    robot.set_robot_joint_positions(jpos)

    observations = env._get_observations(force_update=True)

    return observations