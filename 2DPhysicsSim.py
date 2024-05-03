import multiprocessing,time,turtle,random,math,sys,copy



#Math Operations for 2D Vectors
def add(vec1,vec2):
  return [sum(pair) for pair in zip(vec1, vec2)]


def subtract(vec1,vec2):
  return [i - j for i, j in zip(vec1, vec2)]


def dot(vec1, vec2):
    return sum(i * j for i, j in zip(vec1, vec2))

def length(vec):
    return math.sqrt(sum(i ** 2 for i in vec))


class Point:
  def __init__(self, pos=[0, 0], v=[0, 0], a=[0, 0], color="black", radius=5):
    self.pos = pos
    self.v = v
    self.a = add(a,gravity)
    self.color = color
    self.radius = radius
    self.physics_processes = []
    points.append(self)
   
  def draw_point(self):
    t.penup()
    t.goto(self.pos[0],self.pos[1]-radius)
    t.pendown()
    t.color(self.color)
    t.begin_fill()
    t.circle(self.radius)
    t.end_fill()


  def detect_collisions(self,dt):
    collided_bodies = []
    for i in points:
        if math.sqrt((i.pos[0] - self.pos[0])**2 + (i.pos[1] - self.pos[1])**2) < self.radius+i.radius and i != self:
            collided_bodies.append(i)
   
    #Floor
    cn = [0,1]
    cd = -(self.pos[1]-radius)+floor_height
    if cd > 0:
        self.pos = add(self.pos,[cn[0]*cd,cn[1]*cd])
        vn = [cn[0]*dot(cn, self.v),cn[1]*dot(cn, self.v)]
        vt = subtract(self.v,vn)
        vn = [vn[0]*-elasticity,vn[1]*-elasticity]
        vt = [vt[0]*math.exp(-friction * dt),vt[1]*math.exp(-friction * dt)]
        self.v = add(vn,vt)


    #Right Wall
    cn = [1,0]
    cd = -(self.pos[0]+radius)+right_wall
    if cd < 0:
        self.pos = add(self.pos,[cn[0]*cd,cn[1]*cd])
        vn = [cn[0]*dot(cn, self.v),cn[1]*dot(cn, self.v)]
        vt = subtract(self.v,vn)
        vn = [vn[0]*-elasticity,vn[1]*-elasticity]
        vt = [vt[0]*math.exp(-friction * dt),vt[1]*math.exp(-friction * dt)]
        self.v = add(vn,vt)


    #Left Wall
    cn = [1,0]
    cd = -(self.pos[0]-radius)+left_wall
    if cd > 0:
        self.pos = add(self.pos,[cn[0]*cd,cn[1]*cd])
        vn = [cn[0]*dot(cn, self.v),cn[1]*dot(cn, self.v)]
        vt = subtract(self.v,vn)
        vn = [vn[0]*-elasticity,vn[1]*-elasticity]
        vt = [vt[0]*math.exp(-friction * dt),vt[1]*math.exp(-friction * dt)]
        self.v = add(vn,vt)


    #Roof
    cn = [0,1]
    cd = -(self.pos[1]+radius)+roof_height
    if cd < 0:
        self.pos = add(self.pos,[cn[0]*cd,cn[1]*cd])
        vn = [cn[0]*dot(cn, self.v),cn[1]*dot(cn, self.v)]
        vt = subtract(self.v,vn)
        vn = [vn[0]*-elasticity,vn[1]*-elasticity]
        vt = [vt[0]*math.exp(-friction * dt),vt[1]*math.exp(-friction * dt)]
        self.v = add(vn,vt)


    return collided_bodies

  #Keep Points from Intersecting
  def resolve_collisions(self,colliding_point1,colliding_point2,dt):
    distance = math.sqrt((colliding_point2.pos[0] - colliding_point1.pos[0])**2 + (colliding_point2.pos[1] - colliding_point1.pos[1])**2)
    cd = colliding_point1.radius+colliding_point2.radius-distance
    cn = [(colliding_point1.pos[0] - colliding_point2.pos[0]) / distance, (colliding_point1.pos[1] - colliding_point2.pos[1]) / distance]
    colliding_point1.pos = add(colliding_point1.pos,[cn[0]*cd,cn[1]*cd])
    vn = [cn[0]*dot(cn, colliding_point1.v),cn[1]*dot(cn, colliding_point1.v)]
    vt = subtract(colliding_point1.v,vn)
    vn = [vn[0]*-elasticity,vn[1]*-elasticity]
    vt = [vt[0]*math.exp(-friction * dt),vt[1]*math.exp(-friction * dt)]
    colliding_point1.v = add(vn,vt)

  #Apply Trasnformational Changes
  def update_physics(self,dt):
    self.v = add([self.a[0]*dt,self.a[1]*dt],self.v)
    self.pos = add([self.v[0]*dt,self.v[1]*dt],self.pos)
    self.physics_processes = []
    for i in self.detect_collisions(dt=dt):
       self.physics_processes.append(copy.deepcopy(i))


class Joint:
   def __init__(self, p0, p1, rd, color="black", damping = 10, spring_force = 100):
      self.p0 = p0
      self.p1 = p1
      self.rd = rd
      self.damping = damping
      self.spring_force = spring_force
      self.color = color
      self.pensize = radius
      constraints.append(self)
     
   #Keep Points Connected
   def enforce_constraints(self,dt):
      delta = subtract(self.p1.pos,self.p0.pos)
      distance = length(delta)
      direction = [delta[0]/distance,delta[1]/distance]
      target_delta = [direction[0]*self.rd,direction[1]*self.rd]
      force = [subtract(target_delta,delta)[0]*self.spring_force,subtract(target_delta,delta)[1]*self.spring_force]

      self.p0.v = subtract(self.p0.v,[force[0]*dt,force[1]*dt])
      self.p1.v = add(self.p1.v,[force[0]*dt,force[1]*dt])

      vrel = dot(subtract(self.p1.v,self.p0.v),direction)
      damping_factor = math.exp(-self.damping * dt)
      new_vrel = vrel*damping_factor
      vrel_delta = new_vrel-vrel

      self.p0.v = [self.p0.v[0]-(vrel_delta/2),self.p0.v[1]-(vrel_delta/2)]
      self.p1.v = [self.p1.v[0]+(vrel_delta/2),self.p1.v[1]+(vrel_delta/2)]

   def draw_joint(self):
      t.penup()
      t.goto(self.p0.pos)
      t.pensize(self.pensize)
      t.pendown()
      t.goto(self.p1.pos)
      t.pensize(1)
      

def update_frame(t):
  previous_time = time.time()
 
  while True:
    current_time = time.time()
    delta_time = current_time - previous_time
    if delta_time > 0:
        try:
            t.clear()
            for i in points:
                i.update_physics(delta_time)
            for j in points:
                for k in j.physics_processes:
                    j.resolve_collisions(colliding_point1=j,colliding_point2=k,dt=delta_time) 
            for l in constraints:
               l.enforce_constraints(delta_time)    
               l.draw_joint()
            for m in points:
               m.draw_point()
            previous_time = current_time
            sc.update()
        except:
            sys.exit()
    time.sleep(0.001)
   


if __name__ == "__main__":
  spring_force = 100
  damping = 10
  friction = 50
  elasticity = 0.5
  win_height = 420
  win_width = 420
  floor_height = -200
  right_wall = 200
  left_wall = -200
  roof_height = 200
  radius = 5
  color = "black"
  points = []
  constraints = []
  gravity = [0,-50]
  t = turtle.Turtle()
  t.hideturtle()
  sc = turtle.Screen()
  sc.title("2D Physics Simulations")
  sc.cv._rootwindow.resizable(False, False)
  sc.setup(win_height,win_width)
  sc.tracer(0)
  point1 = Point(pos=[30,0],v=[10,0],radius=radius)
  point2 = Point(pos=[-30,0],radius=radius)
  joint1 = Joint(point1,point2,100)

  physics_process = multiprocessing.Process(target=update_frame(t=t))
  physics_process.start()
  sc.mainloop()
