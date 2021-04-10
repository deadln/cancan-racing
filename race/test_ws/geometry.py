import math


class Point:
    '''3d Point (float)'''

    def __init__(self, x, y, z):
        '''Defines x and y variables'''
        self.x = x
        self.y = y
        self.z = z

    def move(self, dx, dy, dz):
        '''Determines where x and y move'''
        self.x = self.x + dx
        self.y = self.y + dy
        self.z = self.z + dz

    def __str__(self):
        return "Point({},{},{})".format(self.x, self.y, self.z)

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_z(self):
        return self.z

    def distance(self, other):
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

    def add_point(self, other):
        self.x += other.get_x
        self.y += other.get_y
        self.z += other.get_z

    def sub_point(self, other):
        self.x -= other.get_x
        self.y -= other.get_y
        self.z -= other.get_z

    def from_dict(self, a):
        self.x = a['x']
        self.y = a['y']
        self.z = a['z']

    def get_dict(self):
        return {'x': self.x, 'y': self.y, 'z': self.z}

