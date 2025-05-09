from math import copysign, sin, cos, pi

'''
Only will ever use (naive) 3-dimensional vectors,
so optimized for this use case.
'''
class Vector:
    def __init__(self, *components):
        self.v=components

    def get_v(self):
        return self.v
    def set_v(self, *new):
        self.v=new

    def multiply(self, f):
        return Vector(*[f*c for c in self.v])
        
    ## returns self-v1
    def subtract(self, v1):
        a=self.v
        b=v1.get_v()
        return Vector(a[0]-b[0], a[1]-b[1], a[2]-b[2])

    def add(self, v1):
        a=self.v
        b=v1.get_v()
        return Vector(a[0]+b[0], a[1]+b[1], a[2]+b[2])

    def component_add(self, v1):
        a=self.v
        b=v1.get_v()
        return (a[0]+b[0], a[1]+b[1], a[2]+b[2])
    
    def norm(self):
        return (self.v[0]**2 + self.v[1]**2 + self.v[2]**2)**0.5
    
    def dot_prod(self, v1):
        b=v1.get_v()
        return self.v[0]*b[0] + self.v[1]*b[1] + self.v[2]*b[2]

    def cross_prod(self, v1):
        b=v1.get_v()
        return Vector(self.v[1]*b[2] - self.v[2]*b[1], self.v[2]*b[0] - b[2]*self.v[0], self.v[0]*b[1] - b[0]*self.v[1])

    def normalize(self):
        norm = self.norm()
        return Vector(*[c/norm for c in self.v])

    def __str__(self):
        return ",".join([str(v) for v in self.v])

class Matrix:
    def __init__(self, *row_vectors):
        self.rv=row_vectors
        
    def get_rv(self):
        return self.rv

    def prod(self, v1):
        v1.set_v(*(v1.dot_prod(v) for v in self.rv))

    @staticmethod
    def get_rotation_matrix(degrees, axis):
        rads=degrees*pi/180
        if axis.lower() == "z":
            return Matrix( Vector(cos(rads), -sin(rads), 0),
              Vector(sin(rads), cos(rads), 0),
                Vector(0, 0, 1))
        elif axis.lower() == "y":
            return Matrix(Vector(cos(rads), 0, sin(rads)),
              Vector(0, 1, 0),
              Vector(-sin(rads), 0, cos(rads)))
        elif axis.lower() == "x":
            return Matrix(Vector(1, 0, 0),
              Vector(0, cos(rads), -sin(rads)),
              Vector(0, sin(rads), cos(rads)))

    @staticmethod
    def get_reflection_matrix(around):
        (x, y, z) = around.get_v()
        return Matrix(Vector(1-2*x**2, -2*x*y, -2*x*z),
                      Vector(-2*x*y, 1-2*y**2, -2*y*z),
                      Vector(-2*x*z, -2*y*z, 1-2*z**2))

    @staticmethod
    def get_scale_matrix(factor):
        return Matrix(Vector(factor, 0, 0),
                      Vector(0, factor, 0),
                      Vector(0, 0, factor))
