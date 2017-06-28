#!/usr/bin/env python

#############################################################################
# IK Calculations Module
#############################################################################

from mpmath import *
from sympy import *
import numpy as np
import sys

verbosity = 1   # sets how much debug info to print

#############################################################################
# Helper functions
#############################################################################

rtd = 180./np.pi # radians to degrees
dtr = np.pi/180. # degrees to radians

# check if a is within 1% of b
def within_one_percent(a, b):
    if a == b:
        return True
    else:
        return np.abs(a-b)/b*100. < 0.01

# make sure the angle (given in radians) is between min and max (given in degrees)
def inrange(prad, mindegrees, maxdegrees):
    plim = prad
    pdegrees = prad * rtd
    if pdegrees < mindegrees:
        plim = mindegrees * dtr
    if pdegrees > maxdegrees:
        plim = maxdegrees * dtr
    return plim

#############################################################################
#
# Do the IK Calculations here. Module separated so it can be run
# independently of the whole simulation.
#
# px, py, pz - the requested location
# lx, ly, lz - the current location
# roll, pitch, yaw - the current orientation
#
#############################################################################

# Define DH param symbols

q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')    # theta_i
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')    # link offsets
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')    # link length

# Joint angle symbols

alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# Joint min max angle symbols

jmin0, jmin1, jmin2, jmin3, jmin4, jmin5, jmin6 = symbols('jmin0:7')
jmax0, jmax1, jmax2, jmax3, jmax4, jmax5, jmax6 = symbols('jmax0:7')

# Modified DH params

"""
    This gave invalid output for rest  position...
s = { alpha0: 0.,     a0: 0.,  d1: 0.75,
      alpha1: -pi/2., a1: 0.35,   d2: 0.,   q2: q2-pi/2.,
      alpha2: 0.,     a2: 1.25,   d3: 0.,
      alpha3: -pi/2., a3: -0.054, d4: 1.50,
      alpha4:  pi/2., a4: 0.,     d5: 0.,
      alpha5: -pi/2., a5: 0.,     d6: 0.,
      alpha6: 0.,     a6: 0.,     d7: 0.303, q7: 0. }
"""
s = { alpha0: 0.,     a0: 0.,     d1: 0.75,
      alpha1: -pi/2., a1: 0.35,   d2: 0.,   q2: q2-pi/2.,
      alpha2: 0.,     a2: 1.25,   d3: 0.,
      alpha3: -pi/2., a3: -0.054, d4: 1.50,
      alpha4: 0.,     a4: 0.,     d5: 0.,
      alpha5: 0.,     a5: 0.,     d6: 0.,
      alpha6: 0.,     a6: 0.,     d7: 0.303, q7: 0. }

# Joint min max values
m = { jmin0: -185.,  jmax0: 185.,
      jmin1: -140.,  jmax1: 5.,
      jmin2: -120.,  jmax2: 155.,
      jmin3: -350.,  jmax3: 350.,
      jmin4: -122.5, jmax4: 122.5,
      jmin5: -350.,  jmax5: 350. }

#############################################################################
#
# Initialize the Forward Kinematics Transformation Matrices
#
# Note that the forward kinematics matrix need only be calculated once
# at startup. After initial calcuation the resultant matrix is evalf'd
# during refinement of the kinematics.
#
#############################################################################

def IK_mathinit():
    
    #############################################################################
    # Part 1: Forward Kinematics
    #############################################################################

    print("Initializing Transformation matrix, please wait...")
    
    # Define Modified DH Transformation matrix

    T0_1 = Matrix([[cos(q1), -sin(q1), 0, a0],
                   [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                   [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                   [ 0, 0, 0, 1]])
    T0_1 = T0_1.subs(s)

    T1_2 = Matrix([[cos(q2), -sin(q2), 0, a1],
                   [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                   [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                   [ 0, 0, 0, 1]])
    T1_2 = T1_2.subs(s)

    T2_3 = Matrix([[cos(q3), -sin(q3), 0, a2],
                   [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                   [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                   [ 0, 0, 0, 1]])
    T2_3 = T2_3.subs(s)

    T3_4 = Matrix([[cos(q4), -sin(q4), 0, a3],
                   [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                   [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                   [ 0, 0, 0, 1]])
    T3_4 = T3_4.subs(s)

    T4_5 = Matrix([[cos(q5), -sin(q5), 0, a4],
                   [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                   [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                   [ 0, 0, 0, 1]])
    T4_5 = T4_5.subs(s)

    T5_6 = Matrix([[cos(q6), -sin(q6), 0, a5],
                   [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                   [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                   [ 0, 0, 0, 1]])
    T5_6 = T5_6.subs(s)

    T6_G = Matrix([[cos(q7), -sin(q7), 0, a6],
                   [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                   [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                   [ 0, 0, 0, 1]])
    T6_G = T6_G.subs(s)

    # Create individual transformation matrices

    T0_2 = simplify(T0_1*T1_2)
    T0_3 = simplify(T0_2*T2_3)
    T0_4 = simplify(T0_3*T3_4)
    T0_5 = simplify(T0_4*T4_5)
    T0_6 = simplify(T0_5*T5_6)
    T0_G = simplify(T0_6*T6_G)

    if verbosity >= 4:
        print "T0_1=", T0_1.evalf(subs={q1:0., q2:0., q3:0., q4:0., q5:0., q6:0., q7:0.})
        print "T1_2=", T1_2.evalf(subs={q1:0., q2:0., q3:0., q4:0., q5:0., q6:0., q7:0.})
        print "T2_3=", T2_3.evalf(subs={q1:0., q2:0., q3:0., q4:0., q5:0., q6:0., q7:0.})
        print "T3_4=", T3_4.evalf(subs={q1:0., q2:0., q3:0., q4:0., q5:0., q6:0., q7:0.})
        print "T4_5=", T4_5.evalf(subs={q1:0., q2:0., q3:0., q4:0., q5:0., q6:0., q7:0.})
        print "T5_6=", T5_6.evalf(subs={q1:0., q2:0., q3:0., q4:0., q5:0., q6:0., q7:0.})
        print "T6_G=", T6_G.evalf(subs={q1:0., q2:0., q3:0., q4:0., q5:0., q6:0., q7:0.})

    if verbosity >= 4:
        print "T0_1=", T0_1
        print "T0_2=", T0_2
        print "T0_3=", T0_3
        print "T0_4=", T0_4
        print "T0_5=", T0_5
        print "T0_6=", T0_6
        print "T0_G=", T0_G

    if verbosity >= 3:
        print "T0_1=", T0_1.evalf(subs={q1:0., q2:0., q3:0., q4:0., q5:0., q6:0., q7:0.})
        print "T0_2=", T0_2.evalf(subs={q1:0., q2:0., q3:0., q4:0., q5:0., q6:0., q7:0.})
        print "T0_3=", T0_3.evalf(subs={q1:0., q2:0., q3:0., q4:0., q5:0., q6:0., q7:0.})
        print "T0_4=", T0_4.evalf(subs={q1:0., q2:0., q3:0., q4:0., q5:0., q6:0., q7:0.})
        print "T0_5=", T0_5.evalf(subs={q1:0., q2:0., q3:0., q4:0., q5:0., q6:0., q7:0.})
        print "T0_6=", T0_6.evalf(subs={q1:0., q2:0., q3:0., q4:0., q5:0., q6:0., q7:0.})
        print "T0_G=", T0_G.evalf(subs={q1:0., q2:0., q3:0., q4:0., q5:0., q6:0., q7:0.})

    #############################################################################
    # Rotation Correction, general solution in 3D
    # ... not actually used in the computation, shown here for reference...
    #############################################################################
    
    R_x = Matrix([[ 1,              0,        0],
                  [ 0,        cos(q1), -sin(q1)],
                  [ 0,        sin(q1),  cos(q1)]])

    R_y = Matrix([[ cos(q2),        0,  sin(q2)],
                  [       0,        1,        0],
                  [-sin(q2),        0,  cos(q2)]])

    R_z = Matrix([[ cos(q3), -sin(q3),        0],
                  [ sin(q3),  cos(q3),        0],
                  [ 0,              0,        1]])

    if verbosity >= 4:
        print("General Rotation about the X-axis by 45-degrees")
        print(R_x.evalf(subs={q1: 45*dtr}))
        print("General Rotation about the y-axis by 45-degrees")
        print(R_y.evalf(subs={q2: 45*dtr}))
        print("General Rotation about the Z-axis by 30-degrees")
        print(R_z.evalf(subs={q3: 30*dtr}))
    
    #############################################################################
    # Correction, in y, z for gripper
    #############################################################################

    R_y = Matrix([[ cos(-pi/2.),  sin(-pi/2.), 0, 0],
                  [ 0,            1,           0, 0],
                  [ -sin(-pi/2.), cos(-pi/2.), 0, 0],
                  [ 0,            0,           0, 1]])
    
    R_z = Matrix([[ cos(pi), -sin(pi), 0, 0],
                  [ sin(pi),  cos(pi), 0, 0],
                  [ 0,              0, 1, 0],
                  [ 0,              0, 0, 1]])

    R_corr = simplify(R_z*R_y)
    T_total = simplify(T0_G * R_corr)
    if verbosity >= 3:
        print("R_corr=", R_corr.evalf(subs={q1:0., q2:0., q3:0., q4:0., q5:0., q6:0., q7:0.}))
        print("T_total=", T_total)

    print("Transformation matrix ready...")

    return T_total

#############################################################################
# Reverse Geometry Calculations
#############################################################################

# Note that theta2 and theta3 are co-dependent, but to get a good guess, I use
# theta2 as the prinicpal contributor to movement in the x axis and theta3 as
# the principal contributor in the z axis.

def ReverseGeometryCalcs(px, py, pz):
    
    theta1 = inrange(np.arctan2(py, px), m[jmin0], m[jmax0])
    # The total x and z distances from geometry
    xdist  = s[a1]+s[d4]+s[d7]    # from joint 3 to gripper
    zdist  = s[d1]+s[a2]+0.058    # from base to gripper fingers
    # now calculate theta2 and theta3
    # Note that limits of movement for each joint are applied
    theta2 = inrange(-np.arctan2(px-xdist, np.sqrt(px**2+py**2))/3., m[jmin2], m[jmax2])
    theta3 = inrange(-np.arctan2(pz-zdist, np.sqrt(px**2+py**2))-theta2, m[jmin3], m[jmax3])
    # all these are rotations and do not contribute to x,y,z displacements
    theta4 = 0.
    theta5 = 0.
    theta6 = 0.
   
    return [theta1, theta2, theta3, theta4, theta5, theta6]


#############################################################################
# Kinematics Calculations
#############################################################################

def IK_calcs(px, py, pz, lx=0., ly=0., lz=0., roll=0., pitch=0., yaw=0.):
    
    # set up initial values at rest

    theta1 = 0.
    theta2 = 0.
    theta3 = 0.
    theta4 = 0.
    theta5 = 0.
    theta6 = 0.
    theta7 = 0.

    T_rest = T_total.evalf(subs={q1:theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6, q7:theta7})
    
    T_np_rest = np.array(T_rest).astype(np.float64)
    
    rx = T_np_rest[0,3]
    ry = T_np_rest[1,3]
    rz = T_np_rest[2,3]

    if verbosity >= 3:
        print "T_rest=", T_rest

    if verbosity >= 1:
        print "Rest position:x,y,z= %.3f %.3f %.3f" % (rx, ry, rz)

    #############################################################################
    # Part 2: Reverse Kinematics
    #############################################################################
    
    # Calculate first three joint angles using Geometric IK method

    theta1, theta2, theta3, theta4, theta5, theta6 = ReverseGeometryCalcs(px, py, pz)

    # Calculate how far we are from the requested position

    T_new = T_total.evalf(subs={q1:theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6, q7:theta7})
    T_np_new = np.array(T_new).astype(np.float64)

    if verbosity >= 2:
        print "T_new=", T_new

    nx = T_np_new[0,3]
    ny = T_np_new[1,3]
    nz = T_np_new[2,3]

    dx = px-nx
    dy = py-ny
    dz = pz-nz

    dist = sqrt(dx**2+dy**2+dz**2)
    
    if verbosity >= 1:
        print "End Effector position:x,y,z   = %.3f %.3f %.3f" % (lx, ly, lz)
        print "Request position:x,y,z        = %.3f %.3f %.3f" % (px, py, pz)
        print "Calculated position:x,y,z     = %.3f %.3f %.3f" % (nx, ny, nz)
        print "Delta dx, dy, dz              = %.3f %.3f %.3f" % (dx, dy, dz), " dist=%.3f"%(dist)
        print "Roll, pitch, yaw:             = %.3f %.3f %.3f" % (roll, pitch, yaw)
        print "Joint angles (degrees)        = %.3f %.3f %.3f %.3f %.3f %.3f %.3f" % (theta1*rtd, theta2*rtd, theta3*rtd, theta4*rtd, theta5*rtd, theta6*rtd, theta7*rtd)

    return [theta1, theta2, theta3, theta4, theta5, theta6]


#############################################################################
# Main: When running standalone. Read x, y, z from the argument list.
#############################################################################

# Initialize the matrix

T_total = IK_mathinit()

# Do offline testing if called from command line

if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "-test":
        for i in Range(20):
            for j in Range(20):
                for k in Range(20):
                    px = float(i-10)/10. * 2.5
                    py = float(j-10)/10. * 2.
                    pz = float(k-10)/10. * 2.
                    theta1, theta2, theta3, theta4, theta5, theta6 = IK_calcs(px, py, pz)
    elif len(sys.argv) >= 4:
        px = float(sys.argv[1])
        py = float(sys.argv[2])
        pz = float(sys.argv[3])
        theta1, theta2, theta3, theta4, theta5, theta6 = IK_calcs(px, py, pz)
    else:
        print "Usage: ", sys.argv[0], " -test | px py pz"
