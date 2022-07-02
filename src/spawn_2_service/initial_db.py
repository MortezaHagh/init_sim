#! /usr/bin/env python3

import tf
import json
from numpy import deg2rad
from geometry_msgs.msg import Pose, Point, Quaternion


class Initial(object):

    def __init__(self, method, stype):
        self.robots_data = []
        self.robots_count = 1
        self.id = []
        self.path_unit = 0.5

        if stype == 'sb':
            sysfunc = self.system_db()
        elif stype == 'sql':
            sysfunc = self.system_sql()
        else:
            pass

        # robots_initial
        if method == 'single_sys':
            sysfunc()
            self.single_sys()
        elif method == 'single_custome':
            self.single_custome()
        elif method == 'multi_sys':
            sysfunc()
            self.multi_sys()
        elif method == 'multi_custome':
            self.multi_custome()
        elif method == 'multi_json':
            self.multi_json()

    def single_custome(self):
        x = 1
        y = 1
        z = 0.0
        R = 0.0
        P = 0.0
        Y = deg2rad(0)
        pose = Point(x, y, z)
        theta = tf.transformations.quaternion_from_euler(R, P, Y)
        orient = Quaternion(*theta)
        self.robots_initial = Pose(pose, orient)
        self.id = 1

    def multi_custome(self):
        self.robots_count = 3
        xx = [1, 2, 3]
        yy = [1, 2, 3]
        YY = [0, 0, 0]

        robots_initial = [Pose() for n in range(self.robots_count)]
        for rd in range(self.robots_count):
            theta = tf.transformations.quaternion_from_euler(
                0, 0, deg2rad(YY[rd]))
            robots_initial[rd].position = Point(xx[rd], yy[rd], 0)
            orient = Quaternion(*theta)
            robots_initial[rd].orientation = orient

        self.robots_initial = robots_initial
        self.id = [i for i in range(self.robots_count)]

    def single_sys(self):
        robot_data = self.robots_data[0]
        self.id = robot_data['id']
        print('robot id: ', str(self.id))
        # robot_data = next(rd for rd in robots_data if rd['id']==id)

        # Pose data
        x = robot_data['position']['x']*self.path_unit/100
        y = robot_data['position']['y']*self.path_unit/100
        z = 0.0
        R = 0.0
        P = 0.0
        Y = robot_data['position']['theta']
        Y = deg2rad(float(Y))
        theta = tf.transformations.quaternion_from_euler(R, P, Y)
        pose = Point(x, y, z)
        orient = Quaternion(*theta)
        self.robots_initial = Pose(pose, orient)

    def multi_sys(self):
        robots_initial = [Pose() for n in range(self.robots_count)]
        robots_data = self.robots_data
        for rd in range(self.robots_count):
            x = robots_data[rd]['position']['x']*self.path_unit/100
            y = robots_data[rd]['position']['y']*self.path_unit/100
            Y = robots_data[rd]['position']['theta']
            Y = deg2rad(float(Y))
            theta = tf.transformations.quaternion_from_euler(0, 0, Y)
            robots_initial[rd].position = Point(x, y, 0)
            orient = Quaternion(*theta)
            robots_initial[rd].orientation = orient
            self.id.append(robots_data[rd]['id'])
        self.robots_initial = robots_initial

    def system_db(self):
        from roboware_db.initial_system import RosInitial
        sys_db = RosInitial()

        # warehouse
        warehouse_name = "DKN-Empty"
        warehouses = sys_db.warehouses
        warehouse = next(w for w in warehouses if w['name'] == warehouse_name)
        warehouse_id = warehouse['id']

        # get robots data
        robots_data = sys_db.robots
        robots_data = [
            rd for rd in robots_data if rd["warehouse_id"] == warehouse_id]
        robots_count = len(robots_data)

        self.robots_data = robots_data
        self.robots_count = robots_count

    def system_sql(self):
        from sql_handlers.services.robot_service import RobotService
        from sql_handlers.services.warehouse_service import WarehouseService

        # warehouse
        warehouse_name = "DKN"
        warehouses = WarehouseService.getAll()
        warehouse = next(w for w in warehouses if w.name == warehouse_name)
        warehouse_id = warehouse.id

        # get robots data
        robots_data = RobotService.getByWarehouseId(warehouse_id)
        robots_count = len(robots_data)

        self.robots_data = robots_data
        self.robots_count = robots_count

    def multi_json(self):
        with open('/home/morteza/catkin_ws/src/my_worlds/src/spawn_2_service/jsonObst.json', 'r') as f:
            obst = json.load(f)
        x = [ox*self.path_unit for ox in obst['x']]
        y = [oy*self.path_unit for oy in obst['y']]
        self.obst = {'x': x, 'y': y, 'count': obst['count']}

        with open('/home/morteza/catkin_ws/src/my_worlds/src/spawn_2_service/jsonRobots.json', 'r') as f:
            robots = json.load(f)

        self.robots_count = len(robots)
        xx = [r['xs']*self.path_unit for r in robots]
        yy = [r['ys']*self.path_unit for r in robots]
        YY = [r['dir'] for r in robots]

        robots_initial = [Pose() for n in range(self.robots_count)]
        for rd in range(self.robots_count):
            theta = tf.transformations.quaternion_from_euler(0, 0, YY[rd])
            robots_initial[rd].position = Point(xx[rd], yy[rd], 0)
            orient = Quaternion(*theta)
            robots_initial[rd].orientation = orient

        self.robots_initial = robots_initial
        self.id = [i for i in range(self.robots_count)]


if __name__ == '__main__':
    obj = Initial('multi_json', '')
    print(obj.obst)
