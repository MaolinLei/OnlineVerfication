#!/usr/bin/env python3

from modular.URDF_writer import *
import rospy
import sys
from FCL_TEST.srv import state_service, state_serviceRequest,  state_serviceResponse


class Grid(object):
    def __init__(
            self,
            x: int = None,  # 坐标x
            y: int = None,  # 坐标y
            grid_type: int = 0,  # 类别值（0：空；1：障碍或边界）
            enter_reward: float = 0.0):  # 进入该格子的即时奖励
        self.x = x
        self.y = y
        self.grid_type = grid_type
        self.enter_reward = enter_reward
        self.name = "X{0}-Y{1}".format(self.x, self.y)
 
    def __str__(self):
        return "Grid: {name:{3}, x:{0}, y:{1}, grid_type:{2}}".format(self.x, self.y, self.grid_type, self.name)


def get_reward(self):
    reward = 0
    return reward


def Table_Link(index):
    
  Table_ = ['module_joint_yaw_ORANGE.yaml','module_joint_double_elbow_ORANGE.yaml',
             'module_link_straight_140.yaml', 'module_link_elbow_45.yaml']
  Name = Table_[index]

  return Name

def add_two_ints_client(x, y, z):
    rospy.wait_for_service('IK')
        # 创建服务的处理句柄,可以像调用函数一样，调用句柄
    client = rospy.ServiceProxy('IK', state_service)
 
    client.wait_for_service()
    req = state_serviceRequest()
    req.x = x
    req.y = y    
    req.z = z

    resp = client.call(req)
    print(resp.x_)  
    print(resp.y_)  
    print(resp.z_)         
    return resp.x_, resp.y_, resp.z_


def award_index(index):
   Table_ = [-2, -2,
             -1, -1] 
   award = Table_[index]
   return award

def urdf_generate(index_sequence):

  urdf_writer.add_socket(0.0, 0.0, 0.0, 0.0)

  angle = 0
  J = 6
  for i in range(len(index_sequence)):
      index = index_sequence[i]
      homing_joint_map = robot_assemble(index, angle)

  urdf_writer.add_simple_ee(0.0, 0.0, 0.189, 0.0)


  # write_file_to_stdout(urdf_writer, homing_joint_map)
  urdf_writer.remove_connectors()

  urdf_writer.write_urdf()
  urdf_writer.write_lowlevel_config()
  urdf_writer.write_problem_description_multi()
  urdf_writer.write_srdf(homing_joint_map)
  urdf_writer.write_joint_map()

  urdf_writer.deploy_robot("modularbot" ,"/home/mlei/concert_ws/ros_src")

  return homing_joint_map


def robot_assemble(index, angle):

  modular =  Table_Link(index)
  data = urdf_writer.add_module(modular, 0, False)
  homing_joint_map[str(data['lastModule_name'])] = {'angle': angle}

  return homing_joint_map


# define the environment, generate the urdf
homing_joint_map = {}
urdf_writer = UrdfWriter(speedup=True)
urdf_generate([1, 0 ,1, 1, 2, 1])


x = 0.2
y = 0.1
z = 0.1
add_two_ints_client(x,y,z)

x = 0.2
y = 0.4
z = 0.1
add_two_ints_client(x,y,z)

x = 0.2
y = 0.6
z = 0.1
add_two_ints_client(x,y,z)
# calculating the inverse kinematicals
