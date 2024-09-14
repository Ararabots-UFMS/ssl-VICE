import matplotlib.pyplot as plt
import numpy as np
from math import fabs
from math import sqrt
time_epsilon = 0.0000001
r_op = 0.09 #Raio robo
float_epsilon = 1.0E-200
theta = np.arange(0, 2*np.pi, 0.01)

def bang_bang_scaled(ix,iv,gx,gv,tf,umin=-1.0,umax=1.0):
    co = bang_bang_optimal(ix,iv,gx,gv,umin,umax)
    tfo = control_time(co)
    
    if tfo > tf:
        print("Error: Requested final time is less than time optimal")
        return []

    # Recently added divide by zero tests. Maybe it's beter to just test whether co is a singleton
    u1 = umax
    num = gx - ix - (iv + gv)*tf*0.5
    den = (iv - gv)*0.5 + tf*u1*0.5

    if fabs(den) < float_epsilon:
        return []
    t1 = num / den

    if t1 < 0:
        u1 = umin
        den = (iv - gv)*0.5 + tf*u1*0.5
        if fabs(den) < float_epsilon:
            return []
        t1 = num / den

    tden = (tf - t1)
    if fabs(tden) < float_epsilon:
        return []
    u2 = ((gv - iv) - u1*t1) / tden
    
    if u2 > umax + time_epsilon or u2 < umin - time_epsilon:
        #print("Failure: Not stretchable from time",tfo,"to",tf,"  u2",u2)
        return []
    
    return [[u1, t1], [u2, tf - t1]]
def bang_bang_hard_stop(ix,iv,gx,gv,umin=-1.0,umax=1.0):
    # Hard stopping time
    if iv > 0:
        ux = umin
        s = -iv / ux
        sx = ix + iv*s + 0.5 * ux*s*s
    else:
        ux = umax
        s = -iv / ux
        sx = ix + iv*s + 0.5 * ux*s*s
    d = bang_bang_optimal(sx,0.0,gx,gv,umin,umax)
   
    c = []
    
    # Make the hard stop (if needed)
    if s > 0.0:
        c.append([ux,s])

    c += d
    
    return c
def bang_bang_hard_stop_wait(ix,iv,gx,gv,tf,umin=-1.0,umax=1.0):
    c = bang_bang_hard_stop(ix,iv,gx,gv,umin,umax)
    tt = control_time(c)
    
    if tt > tf:
        #print("No enough time for hard stop")
        return []
    
    # Make a wait (if needed)
    if tf > tt:
        if iv == 0.0:
            c.insert(0,[0.0,tf-tt])
        else:
            c.insert(1,[0.0,tf-tt])
    
    return c
#Função para otimização bang bang
def bang_bang_optimal(ix,iv,gx,gv,umin=-1.0,umax=1.0):

    # Checks for zero-time solution.  Remove if you dare!
    if ix==gx and iv==gv:
        return []
    
    invmin = 1 / umin
    invmax = 1 / umax
    c1 = ix - gx - 0.5*(invmin*iv*iv - invmax*gv*gv)
    a1 = 0.5*(invmin - invmax)
    s1 = -4.0*a1*c1

    c2 = ix - gx - 0.5*(invmax*iv*iv - invmin*gv*gv)
    s2 = 4.0*a1*c2 # Saving computation by noting that a2 = -a1

    t1 = t1b = 1.0E20
    t2 = t2b = 1.0E20
    u1 = u1b = umin
    u2 = u2b = umax
    
    if s1 >= 0:
        xdot = sqrt(s1) / (2.0*a1)
        t1 = invmin*(xdot-iv)
        t2 = invmax*(gv-xdot)
        u1 = umin
        u2 = umax
    
    if s2 >= 0:
        xdot = -sqrt(s2) / (2.0*a1)
        t1b = invmax*(xdot-iv)
        t2b = invmin*(gv-xdot)
        u1b = umax
        u2b = umin

    # Fix numerical problems! Only occurs in case where init and goal are already along an optimal parabolic curve (single bang case)
    if fabs(t1) < time_epsilon:
        t1 = 0.0
    if fabs(t2) < time_epsilon:
        t2 = 0.0
    if fabs(t1b) < time_epsilon:
        t1b = 0.0
    if fabs(t2b) < time_epsilon:
        t2b = 0.0 
    
    if (t1b+t2b < t1+t2 and t1b >= 0.0 and t2b >= 0.0) or t1 < 0.0 or t2 < 0.0:
        t1 = t1b; t2 = t2b; u1 = u1b; u2 = u2b

    '''
    # Test whether answer reaches the goal
    xc = ix + iv*t1 + 0.5*u1*t1*t1
    vc = iv + u1*t1
    # (xv,vc) should be the parabola intersection point
    xf = xc + vc*t2 + 0.5*u2*t2*t2
    vf = vc + u2*t2
    print("Integrated crossing: (",xc,",",vc,")  Final state: (",xf,",",vf,")")
    err = abs(xf-gx)+abs(vf-gv)
    if err > 0.001 or t1 < -0.001 or t2 < -0.001:
        print("Error! ",err,t1,t2)
    '''

    # No need to include zero-time control segments
    if t1 == 0.0:
        return [[u2,t2]]
    if t2 == 0.0:
        return [[u1,t1]]
    
    return [[u1,t1],[u2,t2]]

def integrate_control_2d(xi,con):
    x = xi[0]
    y = xi[1]
    xdot = xi[2]
    ydot = xi[3]
    #    n_op = [vector_x/sqrt(vector_x*vector_x+vector_y*vector_y), vector_y/sqrt(vector_x*vector_x+vector_y*vector_y)]
    #    c = [n_op, time]
    x += xdot*con[1] + 0.5*(con[0][0])*(con[1]**2)
    xdot += (con[0][0])*con[1]
    y += ydot*con[1] + 0.5*(con[0][1])*(con[1]**2)
    ydot += (con[0][1])*con[1]
    #xinit = (robot_x, robot_y, vector_x, vector_y)
    return (x,y,xdot,ydot)

def merge_vector_scalar_controls(c1,c2):
    tt = control_time(c1) # c2 should have same time
    if tt == 0.0:
        return []
    # Calculate and store terminal time for each stage
    ts1 = []
    t = 0.0
    for cs in c1:
        t += cs[1]
        ts1.append(t)
    ts2 = []
    t = 0.0
    for cs in c2:
        t += cs[1]
        ts2.append(t)
    
    merged = False
    c = []
    i = 0
    j = 0
    t = 0.0
    dt = 0.0
    while not merged:
        if ts1[i]-t < ts2[j]-t:
            dt = ts1[i] - t
            c.append([c1[i][0]+[c2[j][0]], dt])
            t += dt
            i += 1
            if (t == ts2[j]):
                j += 1
        else:
            dt = ts2[j] - t
            c.append([c1[i][0]+[c2[j][0]], dt])
            t += dt
            j += 1
            if (t == ts1[i]):
                i += 1
        if t >= tt - time_epsilon:
            merged = True
    if abs(tt - control_time(c)) > 0.00001:
        print("control merge error.  Times don't match")
        
    return c

def merge_scalar_controls(cvec):
    cv = []
    for c in cvec[0]:
        cv.append([[c[0]],c[1]])
    cm = merge_vector_scalar_controls(cv,cvec[1])
    i = 2
    n = len(cvec) # Dimension of input vector
    while i < n:
        cm = merge_vector_scalar_controls(cm,cvec[i])
        i += 1
    return cm
def time_optimal_steer_2d(xinit,xgoal,umin=(-1,-1),umax=(1,1)):
    c1 = bang_bang_optimal(xinit[0],xinit[2],xgoal[0],xgoal[2],umin[0],umax[0])
    c2 = bang_bang_optimal(xinit[1],xinit[3],xgoal[1],xgoal[3],umin[1],umax[1])
    t1 = control_time(c1)
    t2 = control_time(c2)

    if fabs(t1-t2) > time_epsilon:
        if t1 < t2:
            c1 = bang_bang_scaled(xinit[0],xinit[2],xgoal[0],xgoal[2],t2,umin[0],umax[0])
            if c1 == []:
                c1 = bang_bang_hard_stop(xinit[0],xinit[2],xgoal[0],xgoal[2],umin[0],umax[0])
                tt1 = control_time(c1)
                if tt1 > t2:
                    c2 = bang_bang_scaled(xinit[1],xinit[3],xgoal[1],xgoal[3],tt1,umin[1],umax[1])
                    if c2 == []:
                        c2 = bang_bang_hard_stop(xinit[1],xinit[3],xgoal[1],xgoal[3],umin[1],umax[1])
                        tt2 = control_time(c2)
                        if tt2 < tt1:
                            c2 = bang_bang_hard_stop_wait(xinit[1],xinit[3],xgoal[1],xgoal[3],tt1,umin[1],umax[1])
                        else:
                            c1 = bang_bang_hard_stop_wait(xinit[0],xinit[2],xgoal[0],xgoal[2],tt2,umin[0],umax[0])
                else: # tt1 <= t2
                    c1 = bang_bang_hard_stop_wait(xinit[0],xinit[2],xgoal[0],xgoal[2],t2,umin[0],umax[0])                        
        else:
            c2 = bang_bang_scaled(xinit[1],xinit[3],xgoal[1],xgoal[3],t1,umin[1],umax[1])
            if c2 == []:
                c2 = bang_bang_hard_stop(xinit[1],xinit[3],xgoal[1],xgoal[3],umin[1],umax[1])
                tt2 = control_time(c2)
                if tt2 > t1:
                    c1 = bang_bang_scaled(xinit[0],xinit[2],xgoal[0],xgoal[2],tt2,umin[0],umax[0])
                    if c1 == []:
                        c1 = bang_bang_hard_stop(xinit[0],xinit[2],xgoal[0],xgoal[2],umin[0],umax[0])
                        tt1 = control_time(c1)
                        if tt1 < tt2:
                            c1 = bang_bang_hard_stop_wait(xinit[0],xinit[2],xgoal[0],xgoal[2],tt2,umin[0],umax[0])
                        else:
                            c2 = bang_bang_hard_stop_wait(xinit[1],xinit[3],xgoal[1],xgoal[3],tt1,umin[1],umax[1])
                else: # tt2 <= t1 
                    c2 = bang_bang_hard_stop_wait(xinit[1],xinit[3],xgoal[1],xgoal[3],t1,umin[1],umax[1])                        

    c = merge_scalar_controls([c1,c2])
    
    return c
#Para que serve esse control_time?
def control_time(con):
    t = 0.0
    for c in con:
        t += c[1]
    return t

def tc(t, t_max):
    if t < 0:
        return 0
    elif t > t_max:
        return t_max
    else:
        return t

#Calcula o caminho mínimo do robo
def min_path(xinit, t_max):
    u1 = 0
    u2 = 0
    if x < 0:
        u1 = 1
    elif x > 0:
        u1 = -1
    if y < 0:
        u2 = 1
    elif y > 0:
        u2 = -1
    u = [[u1, u2], t_max]
    return u

#Função para calcular obstaculos para movimentações ofensivas
def agressive_obstacles_calc(time, robot_x, robot_y, vector_x, vector_y, aceleration):
    xinit = (robot_x, robot_y, vector_x, vector_y)
    vector_x_unaceleration = aceleration
    if vector_x > 0:
        vector_x_unaceleration = vector_x_unaceleration*-1
    elif vector_x == 0:
        vector_x_unaceleration = 0
    vector_y_unaceleration = aceleration
    if vector_y > 0:
        vector_y_unaceleration = vector_y_unaceleration*-1
    elif vector_y == 0:
        vector_y_unaceleration = 0
    f_min = integrate_control_2d(xinit, [[vector_x_unaceleration, vector_y_unaceleration], time])
    f_min = sqrt(f_min[0]*f_min[0]+f_min[1]*f_min[1])
    f_max = integrate_control_2d(xinit, [[aceleration, aceleration], time])
    f_max = sqrt(f_max[0]*f_max[0]+f_max[1]*f_max[1])
    r_dyn = abs(f_max-f_min)/2
    p_op = [robot_x, robot_y]
    if vector_x == 0 and vector_y == 0:
        n_op = [0, 0]
    else:
        n_op = [vector_x/sqrt(vector_x*vector_x+vector_y*vector_y), vector_y/sqrt(vector_x*vector_x+vector_y*vector_y)]

    p_obstacle = [p_op[0]+(n_op[0]*(f_min+r_dyn)), p_op[1]+(n_op[1]*(f_min+r_dyn))] #Centro do obstáculo
    r_obstacle = r_op+r_dyn #Raio total do obstáculo
    
    # #Printa raios que serão plotados
    # print(r_obstacle)
    # print(r_op)

    #Geração de array que desenha o circulo para plot da área que o robo pode estar
    x_external_circle = r_obstacle*np.cos(theta)
    y_external_circle = r_obstacle*np.sin(theta)

    #Geração do array que desenha o circulo para plot da área que robo ocupa
    x_robot_position = r_op*np.cos(theta)
    y_robot_position = r_op*np.sin(theta)

    #Plot dos dados
    plt.plot(x_robot_position+robot_x, y_robot_position+robot_y) #Plot do robo
    plt.plot(x_external_circle+p_obstacle[0], y_external_circle+p_obstacle[1]) #Plot da área que o robo pode estar
    plt.show()


#def non_agressive_obstacles_calc(time, robot_x, robot_y, vector_x, vector_y):
time = float(input('Digite quanto tempo deseja simular: '))
x = float(input(f'Digite a posição em X do robo: '))    
y = float(input(f'Digite a posição em y do robo: '))
max_robot_velocity = float(input(f"Digite a velocidade máxima do robo: "))
max_robot_aceleration = float(input("Digite a aceleração máxima do robo: "))
agressive_obstacles_calc(time, x, y, max_robot_velocity, max_robot_velocity, max_robot_aceleration)



























