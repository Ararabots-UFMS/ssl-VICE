#!/usr/bin/env python

'''
Bang-Bang Steering for a Vector of Double-Integrators

This method was published in "Bang-bang boosting of RRTs", A. J. LaValle,
B. Sakcak, and S. M. LaValle. IEEE/RSJ International Conference on Intelligent
Robots and Systems, Oct 2023.

BSD 2-Clause License

Copyright (c) 2023, Alexander J. LaValle
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''


from math import sqrt, fabs
import random

time_epsilon = 0.0000001
float_epsilon = 1.0E-200


'''
Given a vector of n double integrators, this function calculates a time-optimal control that steers the system from state xinit to state xgoal. 
Each state is formatted as a 2n-dimensional vector with n positions listed first, followed by n velocities. The acceleration limits are set
by umin and umax, which are n-dimensional vectors. 

The output is a list of constant controls, each of which is formatted as [u, t], in which u is an n-dimensional vector of acceleration inputs
and t is the length of time for which they are applied. 

Note: This does not use the O(n lg n) time sweeping method described in the IROS paper. It is O(n^2) in the worst case, which is significant for very large n. 
'''
def time_optimal_steer(xinit,xgoal,umin=0,umax=0):
    n = int(0.5*len(xinit))
    if umin == 0:
        umin = [-1.0]*n
    if umax == 0:
        umax = [1.0]*n
    cvec = []
    tvec = []
    tmax = 0.0
    imax = 0
    for i in range(n):
        
        cvec.append(bang_bang_optimal(xinit[i],xinit[n+i],xgoal[i],xgoal[n+i],umin[i],umax[i]))
        tvec.append(control_time(cvec[i]))
        
        if tvec[i] > tmax:
            tmax = tvec[i]
            imax = i
    i = 0
    restart = False
    
    while i < n:
        if i != imax and fabs(tvec[i] - tmax) > time_epsilon:
            c = bang_bang_scaled(xinit[i],xinit[n+i],xgoal[i],xgoal[n+i],tmax,umin[i],umax[i])
            if c == []:
                c = bang_bang_hard_stop_wait(xinit[i],xinit[n+i],xgoal[i],xgoal[n+i],tmax,umin[i],umax[i])
                #print("Bang failure",i)
            if c == []:
                c = bang_bang_hard_stop(xinit[i],xinit[n+i],xgoal[i],xgoal[n+i],umin[i],umax[i])
                tmax = control_time(c)
                imax = i
                restart = True
                #print("Bang double failure",i,"new max time",tmax)
            cvec[i] = c
            tvec[i] = control_time(c)
            if restart == True:
                i = -1
        i += 1

    cv = merge_scalar_controls(cvec)
    return cv

'''
This is a faster version of time_optimal_steer for the special case of two double integrators. 
'''
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


# Calculate total time of a control
def control_time(con):
    t = 0.0
    for c in con:
        t += c[1]
    return t


# Assumes 4d state space: x, y, xdot, ydot
def integrate_control_2d(xi,con):
    x = xi[0]
    y = xi[1]
    xdot = xi[2]
    ydot = xi[3]
    for c in con:
        x += xdot*c[1] + 0.5*c[0][0]*c[1]**2
        xdot += c[0][0]*c[1]
        y += ydot*c[1] + 0.5*c[0][1]*c[1]**2
        ydot += c[0][1]*c[1]
    return (x,y,xdot,ydot)


def integrate_control_nd(xinit,con):
    n = int(0.5*len(xinit))
    xn = []
    for i in range(len(xinit)):
        xn.append(xinit[i])
    #print("xn",xn,"\n\ncon",con)
    for c in con:
        for i in range(n):
            xn[i] += xn[n+i]*c[1] + 0.5*c[0][i]*c[1]**2
            xn[n+i] += c[0][i]*c[1]
    return xn


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


# Returns time-optimal control to get from (ix,iv) to (gx,gv) in the phase plane
# Control bounds are umin and umax
# Output is [[t1, u1], [t2, u2]]
# We need a fast version that assumes nondegeneracy (both bangs are nonzero)
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


def test_bang_bang_optimal():
    for i in range(1):
        # X = 0, Y = 0, Vel X = random, Vel Y = random.
        xinit = (0.0,0.0,random.random()*20.0-10.0,random.random()*20.0-10.0)
        c = []
        for j in range(random.randint(1,10)):
            c.append([[random.random()*2.0-1.0,random.random()*2.0-1.0],random.random()*5.0])
        print(c)
        ttc = control_time(c)
        xgoal = integrate_control_2d(xinit,c)

        print(f'xinit: {xinit} | xgoal: {xgoal}')

        copt = time_optimal_steer_2d(xinit,xgoal)

        print(copt)
        ttcopt = control_time(copt)
        
        print(ttc,ttcopt)
        if ttcopt > ttc:
            print("Bad optimization",ttc,ttcopt)


# Returns empty control if it fails
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
