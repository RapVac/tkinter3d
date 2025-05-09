from tkinter import *
from time import sleep, time

from small_linalg import *
from small_primitives import *

tk=Tk()
size=500
if size % 2 != 0:
    raise ValueError("Size must be even")

tk.resizable(0, 0)
canvas=Canvas(tk, width=size, height=size, highlightthickness=0)
canvas.create_rectangle(-5, -5, size+5, size+5, fill="black")
canvas.pack()
tk.update()

class Canvas:
    def __init__(self, canvas):
        self.canvas=canvas
        self.w=self.canvas.winfo_width()
        self.h=self.canvas.winfo_height()

    def cx(self, x):
        return (self.w/2)-x
    def cy(self, y):
        return (self.h/2)-y
    def draw_point(self, x, y, c="black"):
        self.canvas.create_oval(self.cx(x-1), self.cy(y-1), self.cx(x+1), self.cy(y+1), fill=c, outline=c)
    def draw_line(self, x1, y1, x2, y2):
        self.canvas.create_line(self.cx(x1), self.cy(y1), self.cx(x2), self.cy(y2))
    def draw_pixel(self, x, y, color):
        self.canvas.create_rectangle(self.cx(x), self.cy(y), self.cx(x+1), self.cy(y+1), fill=color, outline="")
    def clear(self):
        self.canvas.delete("all")

## Checklist for creating object:
## - read from obj, store return Object instance
## - setup individual bounding box
## - add object to bounding hierarchy
## - add vectors to `all_vectors'
## - add planes to `all_planes'
## - add culled planes to `culled_planes'
####

## 1. Define standard objects
all_vectors=[]
all_planes=[]
culled_planes=[]

c=Canvas(canvas)

# Camera vector
c_ray=Vector(0, -1, 0)
# Origin of camera vector
o_ray=Vector(0, 500, 0)
light_ray=Vector(400, 400, 400)

m1=Matrix.get_rotation_matrix(25, "x")
m2=Matrix.get_rotation_matrix(25, "y")
m3=Matrix.get_rotation_matrix(25, "z")
m4=Matrix.get_scale_matrix(2)

## 2. Read and modify 3d figures
obj_reader=SmallObj("ico_sphere.obj", "./objects")
Ico1=obj_reader.read()

Ico1.transform(m1)
Ico1.transform(m2)
Ico1.transform(m3)
Ico1.translate(Vector(150, -200, 0))


obj_reader.set_file("cube2.obj")
Cube1=obj_reader.read()
Cube1.transform(m1)
Cube1.transform(m2)
Cube1.transform(m3)

obj_reader.set_file("sphere.obj")
Sphere=obj_reader.read()
Sphere.transform(m1)
Sphere.transform(m2)
Sphere.transform(m3)
Sphere.transform(m4)

#Sphere.transform(m4)

## 3. Set up bounding box(es) and document all verts/planes
AllBound = Bound(Cube1.get_verts() + Ico1.get_verts(), [])
CubeBound = Bound(Cube1.get_verts(), Cube1.get_planes())
IcoBound = Bound(Ico1.get_verts(), Ico1.get_planes())
SBound = Bound(Sphere.get_verts(), Sphere.get_planes())

B = BVH([[AllBound], [SBound]])
B.update_all()

all_vectors+=Sphere.get_verts()
all_planes+=Sphere.get_planes()

culled_planes+=Sphere.cull_planes(c_ray)

t1=time()

for i in range(1, size**2+1):
    x=i%size-size/2
    y=i//size-size/2
    o_ray.set_v(x, 500, y)

    min_dist=float('inf')
    min_plane=None

    if B.get_level(0)[0].is_hit(o_ray, c_ray):
        for b in B.get_level(1):
            
            if b.is_hit(o_ray, c_ray):
                for face in b.get_planes():
                    
                    if face in culled_planes:
                        t=face.is_hit(o_ray, c_ray)
                        if t<min_dist:
                            #print(t, face.get_color())
                            min_dist=t
                            min_plane=face

    if min_dist != float('inf'):        
        i = o_ray.add( c_ray.multiply(min_dist) )
        v_res = i.subtract(light_ray)
        c.draw_pixel(x, y, min_plane.get_phong_color(v_res.multiply(-1), i.multiply(-1)))

        ## c.draw_pixel(x, y, min_plane.get_color())

tk.update_idletasks()
print(time()-t1)
