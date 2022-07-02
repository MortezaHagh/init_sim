from numpy import deg2rad


class Initial(object):

    def __init__(self, method, stype):
        self.path_unit = 1
        self.robots_data = []
        self.robots_count = 1
        self.id = []
        self.robots_initial = []

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

    def single_custome(self):
        x = 1
        y = 1
        z = 0.0
        R = 0.0
        P = 0.0
        Y = deg2rad(0)
        robot_initial = {'xyz': [x, y, z], 'rpy': [R, P, Y]}
        self.robots_initial = robot_initial
        self.id = 1

    def multi_custome(self):
        self.robots_count = 3
        xx = [1, 2, 3]
        yy = [1, 2, 3]
        YY = [0, 0, 0]
        ri = {'xyz': [], 'rpy': []}
        robots_initial = [ri.copy() for n in range(self.robots_count)]
        for rd in range(self.robots_count):
            robots_initial[rd]['xyz'] = [xx[rd], yy[rd], 0]
            robots_initial[rd]['rpy'] = [0, 0, deg2rad((YY[rd]))]

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
        robot_initial = {'xyz': [], 'rpy': []}
        robot_initial['xyz'] = [float(x), float(y), z]
        robot_initial['rpy'] = [R, P, deg2rad(float(Y))]
        self.robots_initial = robot_initial

    def multi_sys(self):
        z, R, P = 0, 0, 0
        ri = {'xyz': [], 'rpy': []}
        robots_initial = [ri.copy() for n in range(self.robots_count)]
        robots_data = self.robots_data
        for rd in range(self.robots_count):
            x = robots_data[rd]['position']['x']*self.path_unit/100
            y = robots_data[rd]['position']['y']*self.path_unit/100
            Y = robots_data[rd]['position']['theta']
            robots_initial[rd]['xyz'] = [float(x), float(y), z]
            robots_initial[rd]['rpy'] = [R, P, deg2rad(float(Y))]
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
