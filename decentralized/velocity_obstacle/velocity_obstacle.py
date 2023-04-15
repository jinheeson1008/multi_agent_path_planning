"""
Collision avoidance using Velocity-obstacle method

author: Ashwin Bose (atb033@github.com)
"""

from utils.multi_robot_plot import plot_robot_and_obstacles,plot_robot_obstacles_and_vos
from utils.create_obstacles import create_obstacles
from utils.control import compute_desired_velocity
import numpy as np
from typing import Union

SIM_TIME = 5.
TIMESTEP = 0.1
NUMBER_OF_TIMESTEPS = int(SIM_TIME/TIMESTEP)
ROBOT_RADIUS = 0.5
VMAX = 2
VMIN = 0.2


def simulate(filename):
    obstacles = create_obstacles(SIM_TIME, NUMBER_OF_TIMESTEPS)

    start = np.array([5, 0, 0, 0])
    goal = np.array([5, 10, 0, 0])
    #start = np.array([5, 5, 0, 0])
    #goal = np.array([5, 5, 0, 0])

    robot_state = start
    robot_state_history = np.empty((4, NUMBER_OF_TIMESTEPS))
    vo_Amat_hist = np.empty((np.shape(obstacles)[2]*2 , 2, NUMBER_OF_TIMESTEPS))
    vo_bvec_hist = np.empty((np.shape(obstacles)[2]*2 , NUMBER_OF_TIMESTEPS))
    vo_pt_hist = np.empty((np.shape(obstacles)[2],2,NUMBER_OF_TIMESTEPS))
    vo_disp_hist = np.empty((np.shape(obstacles)[2],1,NUMBER_OF_TIMESTEPS))
    for i in range(NUMBER_OF_TIMESTEPS):
        v_desired = compute_desired_velocity(robot_state, goal, ROBOT_RADIUS, VMAX)
        control_vel,vo_Amat,vo_bvec,vo_pt,vo_disp = compute_velocity(
            robot_state, obstacles[:, i, :], v_desired)
        robot_state = update_state(robot_state, control_vel)
        robot_state_history[:4, i] = robot_state
        vo_Amat_hist[:,:,i] = vo_Amat
        vo_bvec_hist[:,i] = vo_bvec
        vo_pt_hist[:,:,i] = vo_pt
        vo_disp_hist[:,:,i] = vo_disp
    plot_robot_obstacles_and_vos(
        robot_state_history, obstacles, ROBOT_RADIUS, NUMBER_OF_TIMESTEPS, SIM_TIME, filename,vo_Amat_hist,vo_bvec_hist,vo_pt_hist,vo_disp_hist)
    


def compute_velocity(robot, obstacles, v_desired):
    pA = robot[:2]
    vA = robot[-2:]
    # Compute the constraints
    # for each velocity obstacles
    number_of_obstacles = np.shape(obstacles)[1]
    Amat = np.empty((number_of_obstacles * 2, 2))
    bvec = np.empty((number_of_obstacles * 2))
    vo_pt = np.empty((number_of_obstacles,2))
    vo_disp = np.empty((number_of_obstacles,1))
    for i in range(number_of_obstacles):
        obstacle = obstacles[:, i]
        pB = obstacle[:2]
        vB = obstacle[2:]
        #dispBA = pA - pB
        dispBA = pB- pA
        distBA = np.linalg.norm(dispBA)
        thetaBA = np.arctan2(dispBA[1], dispBA[0])
        if 2.2 * ROBOT_RADIUS > distBA:
            distBA = 2.2*ROBOT_RADIUS
        phi_obst = np.arcsin(2.2*ROBOT_RADIUS/distBA)
        phi_left = thetaBA + phi_obst
        phi_right = thetaBA - phi_obst

        # VO
        translation = vB
        Atemp, btemp = create_constraints(translation, phi_left, "left")
        Amat[i*2, :] = Atemp
        bvec[i*2] = btemp
        Atemp, btemp = create_constraints(translation, phi_right, "right")
        Amat[i*2 + 1, :] = Atemp
        bvec[i*2 + 1] = btemp
        vo_pt[i,:] = pA + vB
        vo_disp[i,:] = distBA

    # Create search-space
    th = np.linspace(0, 2*np.pi, 20)
    vel = np.linspace(0, VMAX, 5)

    vv, thth = np.meshgrid(vel, th)

    vx_sample = (vv * np.cos(thth)).flatten()
    vy_sample = (vv * np.sin(thth)).flatten()

    v_sample = np.stack((vx_sample, vy_sample))

    v_satisfying_constraints = check_constraints(v_sample, Amat, bvec)
    
    v_test = np.array([v_desired]).T

    check_result = check_constraints_single_vel(v_test,Amat,bvec)
    print("For velocity:{} -> danger result:{},safe result:{}".format(v_test.T,check_result['danger'],check_result['safe']))

    # Objective function
    size = np.shape(v_satisfying_constraints)[1]
    diffs = v_satisfying_constraints - \
        ((v_desired).reshape(2, 1) @ np.ones(size).reshape(1, size))
    norm = np.linalg.norm(diffs, axis=0)
    min_index = np.where(norm == np.amin(norm))[0][0]
    cmd_vel = (v_satisfying_constraints[:, min_index])

    return cmd_vel , Amat, bvec , vo_pt , vo_disp


def check_constraints(v_sample, Amat, bvec):
    length = np.shape(bvec)[0]
    
    for i in range(int(length/2)):
        v_sample = check_inside(v_sample, Amat[2*i:2*i+2, :], bvec[2*i:2*i+2])

    return v_sample

def check_constraints_single_vel(v_sample:np.ndarray, Amat, bvec):
    length = np.shape(bvec)[0]
    v_sample_save = v_sample.copy()
    safe_count,danger_count=0,0
    for i in range(int(length/2)):
            v_sample_result = check_inside(v_sample, Amat[2*i:2*i+2, :], bvec[2*i:2*i+2])
            if(v_sample_result.shape!=(0,)):
                safe_count+=1
            else:
                danger_count+=1
    return {'safe':safe_count,'danger':danger_count}


def check_inside(v, Amat, bvec):
    v_out = []
    for i in range(np.shape(v)[1]):
        if not ((Amat @ v[:, i] +bvec < 0).all()):
            v_out.append(v[:, i])
    return np.array(v_out).T

def check_inside_v2(v, Amat, bvec,return_code:Union[int,str]=0)->Union[tuple,dict]:
    #Return code 0:safe, 1:danger,2:both
    v_safe = []
    v_danger=[]
    v_safe_counts=[]
    v_danger_counts=[]
    for i in range(np.shape(v)[1]):
        if not ((Amat @ v[:, i] +bvec < 0).all()):
            v_safe.append(v[:, i])
            v_safe_counts.append(sum(Amat @ v[:, i] +bvec < 0))
        else:
            v_danger.append(v[:, i])
            v_danger_counts.append(sum(Amat @ v[:, i] +bvec < 0))
    if(return_code in [2,'both']):
        return {"danger":(np.array(v_danger).T, v_danger_counts),"safe":(np.array(v_safe).T, v_safe_counts)}
    elif(return_code in [1,'danger']):
        return np.array(v_danger).T, v_danger_counts
    else:
        return np.array(v_safe).T, v_safe_counts


def create_constraints(translation, angle, side):
    # create line
    origin = np.array([0, 0, 1])
    point = np.array([np.cos(angle), np.sin(angle)]) 
    line = np.cross(origin, point)#  각도와 origin 사이 normal vector 생성
    line = translate_line(line, translation)

    if side == "right": # left면 방향 조정
        line *= -1

    A = line[:2] # normal vector
    b = line[2] # 최소 거리(원점 간에) # => +bvec

    return A, b # A : line equation, b: 그 거리


def translate_line(line, translation):
    matrix = np.eye(3)
    matrix[2, :2] = -translation[:2] # 역변환
    return matrix @ line
    #-> -bvec

def update_state(x, v):
    new_state = np.empty((4))
    new_state[:2] = x[:2] + v * TIMESTEP
    new_state[-2:] = v
    return new_state

#From : https://github.com/MengGuo/RVO_Py_MAS
def in_between_angle(theta_right, theta_dif, theta_left):
    if abs(theta_right - theta_left) <= np.pi:
        if theta_right <= theta_dif <= theta_left:
            return True
        else:
            return False
    else:
        if (theta_left <0) and (theta_right >0):
            theta_left += 2*np.pi
            if theta_dif < 0:
                theta_dif += 2*np.pi
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        if (theta_left >0) and (theta_right <0):
            theta_right += 2*np.pi
            if theta_dif < 0:
                theta_dif += 2*np.pi
            if theta_left <= theta_dif <= theta_right:
                return True
            else:
                return False
