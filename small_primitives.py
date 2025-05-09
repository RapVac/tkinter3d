from small_linalg import *
from colorsys import rgb_to_hsv, hsv_to_rgb
import os
import copy

class Line:
    def __init__(self, v1, v2):
        self.start=v1
        self.end=v2
    def get_start(self):
        return self.start
    def get_end(self):
        return self.end

class Plane:
    ''' v1-v3 are three points that define a triangular plane
        Clockwise winding order, e.g.
        v2
        |  \
        v1--v3
    '''
    def __init__(self, v1, v2, v3, color):
        self.v1=v1
        self.c1=v2.subtract(self.v1)
        self.c2=v3.subtract(self.v1)
        #self.v2=v2
        #self.v3=v3

        self.normal=self.c1.cross_prod(self.c2)
        self.color=color
    def __string__(self):
        return self.color
    
    def get_color(self):
        return self.color
    def set_color(self, new_color):
        self.color=new_color
    def get_dist_color(self, distance_from_light, vec_cos):
        (r, g, b)=[int(self.color[n:n+2], 16)/255 for n in range(1, len(self.color), 2)]

        intensity=1000
        p=1/(distance_from_light/intensity+1)**2
        (h, s, v) = rgb_to_hsv(r, g, b)
        v*=p
        #s*=(1-vec_cos**2)**0.5

        (r, g, b) = hsv_to_rgb(h, s, v)
        (r, g, b) = [format(round(255*c), "02x") for c in (r, g, b)]
        return "#"+r+g+b

    def get_phong_color(self, point_to_light, point_to_camera):
        (r, g, b)=[int(self.color[n:n+2], 16)/255 for n in range(1, len(self.color), 2)]

        N=self.normal.normalize()
        R=point_to_light.normalize()
        point_to_light=copy.deepcopy(R)
        Matrix.get_reflection_matrix(N).prod(R)

        point_to_camera=point_to_camera.normalize()

        kd=1
        ks=1
        n=50
        
        di = max(kd*(N.dot_prod(point_to_light)), 0)
        sp = max(ks*(R.dot_prod(point_to_camera))**n, 0)

        #print(p, N.dot_prod(point_to_light), R.dot_prod(point_to_camera)**n)
        (h, s, v) = rgb_to_hsv(r, g, b)
        #v*=di
        (rd, gd, bd) = hsv_to_rgb(h, 1, di)
        #v/=di
        #s=1-s*sp
        (rs, gs, bs) = hsv_to_rgb(0, 0, sp)
        dim_factor=2
        (ra, ga, ba) = (r/dim_factor, g/dim_factor, b/dim_factor)
        
        
        #print((rd, gd, bd), (rs, gs, bs), (di, sp))
        (r, g, b) = [cd+cs+ca if cd+cs+ca <= 1 else 1 for cd, cs, ca in
                     zip((rd, gd, bd), (rs, gs, bs), (ra, ga, ba))]
        #r, g, b) = hsv_to_rgb(h, )

        #print(di, sp)
        #print(h, s, v)
        #(r, g, b) = hsv_to_rgb(h, s, v)
        (r, g, b) = [format(round(255*c), "02x") for c in (r, g, b)]
        return "#"+r+g+b
        
    
    def get_shadow_color(self, value):
        (r, g, b)=[int(self.color[n:n+2], 16) for n in range(1, len(self.color), 2)]
        factor=255/max(r, g, b) * value
        (r, g, b) = [format(round(c*factor), '02x') for c in (r, g, b)]
        return "#"+r+g+b

    def get_v1(self):
        return self.v1
    def get_c1(self):
        return self.c1
    def get_c2(self):
        return self.v2
    def get_normal(self):
        return self.normal
    def update_normal(self):
        self.normal=self.c1.cross_prod(self.c2)

    ## https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
    def is_hit(self, source, ray):
        ray=ray.multiply(-1)
        denom=ray.dot_prod(self.normal)
        sub=source.subtract(self.v1)

        u=self.c2.cross_prod(ray).dot_prod(sub)/denom
        v=ray.cross_prod(self.c1).dot_prod(sub)/denom

        if (u+v)<=1 and 0<=v<=1 and 0<=u<=1:
            t = self.get_normal().dot_prod(sub)/denom
            return t
        return float('inf')

class Bound:
    def __init__(self, points, planes):
        self.points=points
        self.planes=planes
        self.center=Vector(0, 0, 0)
        self.radius=0
        self.update()

    def get_planes(self):
        return self.planes

    def update(self):
        self.center.set_v(sum(point.get_v()[0] for point in self.points)/len(self.points),
                           sum(point.get_v()[1] for point in self.points)/len(self.points),
                           sum(point.get_v()[2] for point in self.points)/len(self.points))
        worst=self.center.subtract(self.points[0]).norm()
        
        for p in self.points[1:]:
            if self.center.subtract(p).norm() > worst:
                worst=self.center.subtract(p).norm()
        self.radius=worst
        
    def is_hit(self, source, ray):
        return self.center.subtract(source).cross_prod(ray).norm() / ray.norm() < self.radius

class BVH:
    '''
    Implements a simple bounding volume hierarchy.
    levels = [[topmost], [subordinate box, subordinate box], ...]
    '''
    def __init__(self, levels: list[list[Bound]]):
        self.levels=levels

    def get_level(self, index):
        return self.levels[index]

    def add_level(self, index, bounds):
        self.levels.insert(index, bounds)

    def update_all(self):
        for l in self.levels:
            for b in l:
                b.update()

class Object:
    def __init__(self, vectors, planes):
        self.verts=vectors
        self.planes=planes

    def transform(self, matrix):
        for v in self.verts:
            matrix.prod(v)
        for p in self.planes:
            matrix.prod(p.c1)
            matrix.prod(p.c2)
            p.update_normal()

    def translate(self, vec):
        for i, v in enumerate(self.verts):
            self.verts[i].set_v(*v.component_add(vec))

    def get_verts(self):
        return self.verts

    def get_planes(self):
        return self.planes

    def cull_planes(self, ray):
        out=[]
        for p in self.planes:
            if p.get_normal().dot_prod(ray) < 0:
                out.append(p)
        return out

class SmallObj:
    def __init__(self, name, obj_loc=""):
        self.main=os.getcwd()
        if obj_loc[:2] == "." + os.sep:
            obj_loc=obj_loc[1:]
        self.obj_loc=obj_loc.lstrip(os.sep)
        
        self.loc=os.path.join(self.main, self.obj_loc, name)
        
    def set_file(self, name):
        self.loc=os.path.join(self.main, self.obj_loc, name)

    def read(self):
        vectors=[]
        lines=[]
        planes=[]
        
        with open(self.loc, "r") as f:
            f=f.read().strip().split(sep="\n")
        for line in f:
            line=line.split()
            if line[0] == "#":
                continue

            try:
                ## Just verts
                components=[float(i) for i in line[1:]]
            except:
                ## Plane with color info
                components=[float(i) for i in line[1:-1]]

            ## print(line, components)
            if line[0] == "v":
                vectors.append(Vector(*components))
            else:
                components=[int(c) for c in components]
                if line[0] == "l":
                    lines.append(Line(vectors[components[0]-1], vectors[components[1]-1]))
                elif line[0] == "f":
                    if "#" not in line[-1]:
                        line.append("#c7c7c7")
                    planes.append( Plane(vectors[components[0]-1], vectors[components[1]-1], vectors[components[2]-1], line[-1]))

        ##return vectors, lines, planes
        return Object(vectors, planes)
