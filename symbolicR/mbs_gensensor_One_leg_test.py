#
#	MBsysTran - Release 8.1
#
#	Copyright 
#	Universite catholique de Louvain (UCLouvain) 
#	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
#	2, Place du Levant
#	1348 Louvain-la-Neuve 
#	Belgium 
#
#	http://www.robotran.be 
#
#	==> Generation Date: Tue Mar 28 15:30:59 2023
#
#	==> Project name: One_leg_test
#
#	==> Number of joints: 10
#
#	==> Function: F6 - Sensors Kinematics
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos, sqrt

def sensor(sens, s, isens):
  q = s.q
  qd = s.qd
  qdd = s.qdd

  dpt = s.dpt
 
# Trigonometric functions

  S4 = sin(q[4])
  C4 = cos(q[4])
  S5 = sin(q[5])
  C5 = cos(q[5])
  S6 = sin(q[6])
  C6 = cos(q[6])
  S7 = sin(q[7])
  C7 = cos(q[7])
  S9 = sin(q[9])
  C9 = cos(q[9])
  S10 = sin(q[10])
  C10 = cos(q[10])
 
# Augmented Joint Position Vectors

  Dz83 = q[8]+s.dpt[3,2]
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics


  if (isens == 1): 

    sens.P[1] = q[1]
    sens.P[2] = 0
    sens.P[3] = 0
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = qd[1]
    sens.V[2] = 0
    sens.V[3] = 0
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[1,1] = (1.0)
    sens.A[1] = qdd[1]
    sens.A[2] = 0
    sens.A[3] = 0
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 2): 

    sens.P[1] = q[1]
    sens.P[2] = q[2]
    sens.P[3] = 0
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = qd[1]
    sens.V[2] = qd[2]
    sens.V[3] = 0
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[1,1] = (1.0)
    sens.J[2,2] = (1.0)
    sens.A[1] = qdd[1]
    sens.A[2] = qdd[2]
    sens.A[3] = 0
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 3): 

    sens.P[1] = q[1]
    sens.P[2] = q[2]
    sens.P[3] = q[3]
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = qd[1]
    sens.V[2] = qd[2]
    sens.V[3] = qd[3]
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[1,1] = (1.0)
    sens.J[2,2] = (1.0)
    sens.J[3,3] = (1.0)
    sens.A[1] = qdd[1]
    sens.A[2] = qdd[2]
    sens.A[3] = qdd[3]
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 4): 

    sens.P[1] = q[1]
    sens.P[2] = q[2]
    sens.P[3] = q[3]
    sens.R[1,1] = (1.0)
    sens.R[2,2] = C4
    sens.R[2,3] = S4
    sens.R[3,2] = -S4
    sens.R[3,3] = C4
    sens.V[1] = qd[1]
    sens.V[2] = qd[2]
    sens.V[3] = qd[3]
    sens.OM[1] = qd[4]
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[1,1] = (1.0)
    sens.J[2,2] = (1.0)
    sens.J[3,3] = (1.0)
    sens.J[4,4] = (1.0)
    sens.A[1] = qdd[1]
    sens.A[2] = qdd[2]
    sens.A[3] = qdd[3]
    sens.OMP[1] = qdd[4]
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 5): 

    ROcp5_25 = S4*S5
    ROcp5_35 = -C4*S5
    ROcp5_85 = -S4*C5
    ROcp5_95 = C4*C5
    OMcp5_25 = qd[5]*C4
    OMcp5_35 = qd[5]*S4
    OPcp5_25 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp5_35 = qdd[5]*S4+qd[4]*qd[5]*C4
    sens.P[1] = q[1]
    sens.P[2] = q[2]
    sens.P[3] = q[3]
    sens.R[1,1] = C5
    sens.R[1,2] = ROcp5_25
    sens.R[1,3] = ROcp5_35
    sens.R[2,2] = C4
    sens.R[2,3] = S4
    sens.R[3,1] = S5
    sens.R[3,2] = ROcp5_85
    sens.R[3,3] = ROcp5_95
    sens.V[1] = qd[1]
    sens.V[2] = qd[2]
    sens.V[3] = qd[3]
    sens.OM[1] = qd[4]
    sens.OM[2] = OMcp5_25
    sens.OM[3] = OMcp5_35
    sens.J[1,1] = (1.0)
    sens.J[2,2] = (1.0)
    sens.J[3,3] = (1.0)
    sens.J[4,4] = (1.0)
    sens.J[5,5] = C4
    sens.J[6,5] = S4
    sens.A[1] = qdd[1]
    sens.A[2] = qdd[2]
    sens.A[3] = qdd[3]
    sens.OMP[1] = qdd[4]
    sens.OMP[2] = OPcp5_25
    sens.OMP[3] = OPcp5_35

  if (isens == 6): 

    ROcp6_25 = S4*S5
    ROcp6_35 = -C4*S5
    ROcp6_85 = -S4*C5
    ROcp6_95 = C4*C5
    ROcp6_16 = C5*C6
    ROcp6_26 = ROcp6_25*C6+C4*S6
    ROcp6_36 = ROcp6_35*C6+S4*S6
    ROcp6_46 = -C5*S6
    ROcp6_56 = -ROcp6_25*S6+C4*C6
    ROcp6_66 = -ROcp6_35*S6+S4*C6
    OMcp6_25 = qd[5]*C4
    OMcp6_35 = qd[5]*S4
    OPcp6_25 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp6_35 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp6_16 = qd[4]+qd[6]*S5
    OMcp6_26 = OMcp6_25+ROcp6_85*qd[6]
    OMcp6_36 = OMcp6_35+ROcp6_95*qd[6]
    OPcp6_16 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp6_25*ROcp6_95-OMcp6_35*ROcp6_85)
    OPcp6_26 = OPcp6_25+ROcp6_85*qdd[6]+qd[6]*(OMcp6_35*S5-ROcp6_95*qd[4])
    OPcp6_36 = OPcp6_35+ROcp6_95*qdd[6]+qd[6]*(-OMcp6_25*S5+ROcp6_85*qd[4])
    sens.P[1] = q[1]
    sens.P[2] = q[2]
    sens.P[3] = q[3]
    sens.R[1,1] = ROcp6_16
    sens.R[1,2] = ROcp6_26
    sens.R[1,3] = ROcp6_36
    sens.R[2,1] = ROcp6_46
    sens.R[2,2] = ROcp6_56
    sens.R[2,3] = ROcp6_66
    sens.R[3,1] = S5
    sens.R[3,2] = ROcp6_85
    sens.R[3,3] = ROcp6_95
    sens.V[1] = qd[1]
    sens.V[2] = qd[2]
    sens.V[3] = qd[3]
    sens.OM[1] = OMcp6_16
    sens.OM[2] = OMcp6_26
    sens.OM[3] = OMcp6_36
    sens.J[1,1] = (1.0)
    sens.J[2,2] = (1.0)
    sens.J[3,3] = (1.0)
    sens.J[4,4] = (1.0)
    sens.J[4,6] = S5
    sens.J[5,5] = C4
    sens.J[5,6] = ROcp6_85
    sens.J[6,5] = S4
    sens.J[6,6] = ROcp6_95
    sens.A[1] = qdd[1]
    sens.A[2] = qdd[2]
    sens.A[3] = qdd[3]
    sens.OMP[1] = OPcp6_16
    sens.OMP[2] = OPcp6_26
    sens.OMP[3] = OPcp6_36

  if (isens == 7): 

    ROcp7_25 = S4*S5
    ROcp7_35 = -C4*S5
    ROcp7_85 = -S4*C5
    ROcp7_95 = C4*C5
    ROcp7_16 = C5*C6
    ROcp7_26 = ROcp7_25*C6+C4*S6
    ROcp7_36 = ROcp7_35*C6+S4*S6
    ROcp7_46 = -C5*S6
    ROcp7_56 = -ROcp7_25*S6+C4*C6
    ROcp7_66 = -ROcp7_35*S6+S4*C6
    ROcp7_17 = ROcp7_16*C7-S5*S7
    ROcp7_27 = ROcp7_26*C7-ROcp7_85*S7
    ROcp7_37 = ROcp7_36*C7-ROcp7_95*S7
    ROcp7_77 = ROcp7_16*S7+S5*C7
    ROcp7_87 = ROcp7_26*S7+ROcp7_85*C7
    ROcp7_97 = ROcp7_36*S7+ROcp7_95*C7
    OMcp7_25 = qd[5]*C4
    OMcp7_35 = qd[5]*S4
    OPcp7_25 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp7_35 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp7_16 = qd[4]+qd[6]*S5
    OMcp7_26 = OMcp7_25+ROcp7_85*qd[6]
    OMcp7_36 = OMcp7_35+ROcp7_95*qd[6]
    OPcp7_16 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp7_25*ROcp7_95-OMcp7_35*ROcp7_85)
    OPcp7_26 = OPcp7_25+ROcp7_85*qdd[6]+qd[6]*(OMcp7_35*S5-ROcp7_95*qd[4])
    OPcp7_36 = OPcp7_35+ROcp7_95*qdd[6]+qd[6]*(-OMcp7_25*S5+ROcp7_85*qd[4])
    RLcp7_17 = s.dpt[3,1]*S5
    RLcp7_27 = ROcp7_85*s.dpt[3,1]
    RLcp7_37 = ROcp7_95*s.dpt[3,1]
    POcp7_17 = RLcp7_17+q[1]
    POcp7_27 = RLcp7_27+q[2]
    POcp7_37 = RLcp7_37+q[3]
    JTcp7_17_5 = -RLcp7_27*S4+RLcp7_37*C4
    JTcp7_27_5 = RLcp7_17*S4
    JTcp7_37_5 = -RLcp7_17*C4
    JTcp7_17_6 = -RLcp7_27*ROcp7_95+RLcp7_37*ROcp7_85
    JTcp7_27_6 = RLcp7_17*ROcp7_95-RLcp7_37*S5
    JTcp7_37_6 = -RLcp7_17*ROcp7_85+RLcp7_27*S5
    OMcp7_17 = OMcp7_16+ROcp7_46*qd[7]
    OMcp7_27 = OMcp7_26+ROcp7_56*qd[7]
    OMcp7_37 = OMcp7_36+ROcp7_66*qd[7]
    ORcp7_17 = OMcp7_26*RLcp7_37-OMcp7_36*RLcp7_27
    ORcp7_27 = -OMcp7_16*RLcp7_37+OMcp7_36*RLcp7_17
    ORcp7_37 = OMcp7_16*RLcp7_27-OMcp7_26*RLcp7_17
    VIcp7_17 = ORcp7_17+qd[1]
    VIcp7_27 = ORcp7_27+qd[2]
    VIcp7_37 = ORcp7_37+qd[3]
    OPcp7_17 = OPcp7_16+ROcp7_46*qdd[7]+qd[7]*(OMcp7_26*ROcp7_66-OMcp7_36*ROcp7_56)
    OPcp7_27 = OPcp7_26+ROcp7_56*qdd[7]+qd[7]*(-OMcp7_16*ROcp7_66+OMcp7_36*ROcp7_46)
    OPcp7_37 = OPcp7_36+ROcp7_66*qdd[7]+qd[7]*(OMcp7_16*ROcp7_56-OMcp7_26*ROcp7_46)
    ACcp7_17 = qdd[1]+OMcp7_26*ORcp7_37-OMcp7_36*ORcp7_27+OPcp7_26*RLcp7_37-OPcp7_36*RLcp7_27
    ACcp7_27 = qdd[2]-OMcp7_16*ORcp7_37+OMcp7_36*ORcp7_17-OPcp7_16*RLcp7_37+OPcp7_36*RLcp7_17
    ACcp7_37 = qdd[3]+OMcp7_16*ORcp7_27-OMcp7_26*ORcp7_17+OPcp7_16*RLcp7_27-OPcp7_26*RLcp7_17
    sens.P[1] = POcp7_17
    sens.P[2] = POcp7_27
    sens.P[3] = POcp7_37
    sens.R[1,1] = ROcp7_17
    sens.R[1,2] = ROcp7_27
    sens.R[1,3] = ROcp7_37
    sens.R[2,1] = ROcp7_46
    sens.R[2,2] = ROcp7_56
    sens.R[2,3] = ROcp7_66
    sens.R[3,1] = ROcp7_77
    sens.R[3,2] = ROcp7_87
    sens.R[3,3] = ROcp7_97
    sens.V[1] = VIcp7_17
    sens.V[2] = VIcp7_27
    sens.V[3] = VIcp7_37
    sens.OM[1] = OMcp7_17
    sens.OM[2] = OMcp7_27
    sens.OM[3] = OMcp7_37
    sens.J[1,1] = (1.0)
    sens.J[1,5] = JTcp7_17_5
    sens.J[1,6] = JTcp7_17_6
    sens.J[2,2] = (1.0)
    sens.J[2,4] = -RLcp7_37
    sens.J[2,5] = JTcp7_27_5
    sens.J[2,6] = JTcp7_27_6
    sens.J[3,3] = (1.0)
    sens.J[3,4] = RLcp7_27
    sens.J[3,5] = JTcp7_37_5
    sens.J[3,6] = JTcp7_37_6
    sens.J[4,4] = (1.0)
    sens.J[4,6] = S5
    sens.J[4,7] = ROcp7_46
    sens.J[5,5] = C4
    sens.J[5,6] = ROcp7_85
    sens.J[5,7] = ROcp7_56
    sens.J[6,5] = S4
    sens.J[6,6] = ROcp7_95
    sens.J[6,7] = ROcp7_66
    sens.A[1] = ACcp7_17
    sens.A[2] = ACcp7_27
    sens.A[3] = ACcp7_37
    sens.OMP[1] = OPcp7_17
    sens.OMP[2] = OPcp7_27
    sens.OMP[3] = OPcp7_37

  if (isens == 8): 

    ROcp8_25 = S4*S5
    ROcp8_35 = -C4*S5
    ROcp8_85 = -S4*C5
    ROcp8_95 = C4*C5
    ROcp8_16 = C5*C6
    ROcp8_26 = ROcp8_25*C6+C4*S6
    ROcp8_36 = ROcp8_35*C6+S4*S6
    ROcp8_46 = -C5*S6
    ROcp8_56 = -ROcp8_25*S6+C4*C6
    ROcp8_66 = -ROcp8_35*S6+S4*C6
    ROcp8_17 = ROcp8_16*C7-S5*S7
    ROcp8_27 = ROcp8_26*C7-ROcp8_85*S7
    ROcp8_37 = ROcp8_36*C7-ROcp8_95*S7
    ROcp8_77 = ROcp8_16*S7+S5*C7
    ROcp8_87 = ROcp8_26*S7+ROcp8_85*C7
    ROcp8_97 = ROcp8_36*S7+ROcp8_95*C7
    OMcp8_25 = qd[5]*C4
    OMcp8_35 = qd[5]*S4
    OPcp8_25 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp8_35 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp8_16 = qd[4]+qd[6]*S5
    OMcp8_26 = OMcp8_25+ROcp8_85*qd[6]
    OMcp8_36 = OMcp8_35+ROcp8_95*qd[6]
    OPcp8_16 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp8_25*ROcp8_95-OMcp8_35*ROcp8_85)
    OPcp8_26 = OPcp8_25+ROcp8_85*qdd[6]+qd[6]*(OMcp8_35*S5-ROcp8_95*qd[4])
    OPcp8_36 = OPcp8_35+ROcp8_95*qdd[6]+qd[6]*(-OMcp8_25*S5+ROcp8_85*qd[4])
    RLcp8_17 = s.dpt[3,1]*S5
    RLcp8_27 = ROcp8_85*s.dpt[3,1]
    RLcp8_37 = ROcp8_95*s.dpt[3,1]
    POcp8_17 = RLcp8_17+q[1]
    POcp8_27 = RLcp8_27+q[2]
    POcp8_37 = RLcp8_37+q[3]
    JTcp8_17_5 = -RLcp8_27*S4+RLcp8_37*C4
    JTcp8_27_5 = RLcp8_17*S4
    JTcp8_37_5 = -RLcp8_17*C4
    JTcp8_17_6 = -RLcp8_27*ROcp8_95+RLcp8_37*ROcp8_85
    JTcp8_27_6 = RLcp8_17*ROcp8_95-RLcp8_37*S5
    JTcp8_37_6 = -RLcp8_17*ROcp8_85+RLcp8_27*S5
    OMcp8_17 = OMcp8_16+ROcp8_46*qd[7]
    OMcp8_27 = OMcp8_26+ROcp8_56*qd[7]
    OMcp8_37 = OMcp8_36+ROcp8_66*qd[7]
    ORcp8_17 = OMcp8_26*RLcp8_37-OMcp8_36*RLcp8_27
    ORcp8_27 = -OMcp8_16*RLcp8_37+OMcp8_36*RLcp8_17
    ORcp8_37 = OMcp8_16*RLcp8_27-OMcp8_26*RLcp8_17
    VIcp8_17 = ORcp8_17+qd[1]
    VIcp8_27 = ORcp8_27+qd[2]
    VIcp8_37 = ORcp8_37+qd[3]
    OPcp8_17 = OPcp8_16+ROcp8_46*qdd[7]+qd[7]*(OMcp8_26*ROcp8_66-OMcp8_36*ROcp8_56)
    OPcp8_27 = OPcp8_26+ROcp8_56*qdd[7]+qd[7]*(-OMcp8_16*ROcp8_66+OMcp8_36*ROcp8_46)
    OPcp8_37 = OPcp8_36+ROcp8_66*qdd[7]+qd[7]*(OMcp8_16*ROcp8_56-OMcp8_26*ROcp8_46)
    ACcp8_17 = qdd[1]+OMcp8_26*ORcp8_37-OMcp8_36*ORcp8_27+OPcp8_26*RLcp8_37-OPcp8_36*RLcp8_27
    ACcp8_27 = qdd[2]-OMcp8_16*ORcp8_37+OMcp8_36*ORcp8_17-OPcp8_16*RLcp8_37+OPcp8_36*RLcp8_17
    ACcp8_37 = qdd[3]+OMcp8_16*ORcp8_27-OMcp8_26*ORcp8_17+OPcp8_16*RLcp8_27-OPcp8_26*RLcp8_17
    RLcp8_18 = ROcp8_77*Dz83
    RLcp8_28 = ROcp8_87*Dz83
    RLcp8_38 = ROcp8_97*Dz83
    POcp8_18 = POcp8_17+RLcp8_18
    POcp8_28 = POcp8_27+RLcp8_28
    POcp8_38 = POcp8_37+RLcp8_38
    JTcp8_28_4 = -RLcp8_37-RLcp8_38
    JTcp8_38_4 = RLcp8_27+RLcp8_28
    JTcp8_18_5 = JTcp8_17_5-RLcp8_28*S4+RLcp8_38*C4
    JTcp8_28_5 = JTcp8_27_5+RLcp8_18*S4
    JTcp8_38_5 = JTcp8_37_5-RLcp8_18*C4
    JTcp8_18_6 = JTcp8_17_6-RLcp8_28*ROcp8_95+RLcp8_38*ROcp8_85
    JTcp8_28_6 = JTcp8_27_6+RLcp8_18*ROcp8_95-RLcp8_38*S5
    JTcp8_38_6 = JTcp8_37_6-RLcp8_18*ROcp8_85+RLcp8_28*S5
    JTcp8_18_7 = -RLcp8_28*ROcp8_66+RLcp8_38*ROcp8_56
    JTcp8_28_7 = RLcp8_18*ROcp8_66-RLcp8_38*ROcp8_46
    JTcp8_38_7 = -RLcp8_18*ROcp8_56+RLcp8_28*ROcp8_46
    ORcp8_18 = OMcp8_27*RLcp8_38-OMcp8_37*RLcp8_28
    ORcp8_28 = -OMcp8_17*RLcp8_38+OMcp8_37*RLcp8_18
    ORcp8_38 = OMcp8_17*RLcp8_28-OMcp8_27*RLcp8_18
    VIcp8_18 = ORcp8_18+VIcp8_17+ROcp8_77*qd[8]
    VIcp8_28 = ORcp8_28+VIcp8_27+ROcp8_87*qd[8]
    VIcp8_38 = ORcp8_38+VIcp8_37+ROcp8_97*qd[8]
    ACcp8_18 = ACcp8_17+OMcp8_27*ORcp8_38-OMcp8_37*ORcp8_28+OPcp8_27*RLcp8_38-OPcp8_37*RLcp8_28+ROcp8_77*qdd[8]+(2.0)*qd[8]*(OMcp8_27*ROcp8_97-OMcp8_37*ROcp8_87)
    ACcp8_28 = ACcp8_27-OMcp8_17*ORcp8_38+OMcp8_37*ORcp8_18-OPcp8_17*RLcp8_38+OPcp8_37*RLcp8_18+ROcp8_87*qdd[8]+(2.0)*qd[8]*(-OMcp8_17*ROcp8_97+OMcp8_37*ROcp8_77)
    ACcp8_38 = ACcp8_37+OMcp8_17*ORcp8_28-OMcp8_27*ORcp8_18+OPcp8_17*RLcp8_28-OPcp8_27*RLcp8_18+ROcp8_97*qdd[8]+(2.0)*qd[8]*(OMcp8_17*ROcp8_87-OMcp8_27*ROcp8_77)
    sens.P[1] = POcp8_18
    sens.P[2] = POcp8_28
    sens.P[3] = POcp8_38
    sens.R[1,1] = ROcp8_17
    sens.R[1,2] = ROcp8_27
    sens.R[1,3] = ROcp8_37
    sens.R[2,1] = ROcp8_46
    sens.R[2,2] = ROcp8_56
    sens.R[2,3] = ROcp8_66
    sens.R[3,1] = ROcp8_77
    sens.R[3,2] = ROcp8_87
    sens.R[3,3] = ROcp8_97
    sens.V[1] = VIcp8_18
    sens.V[2] = VIcp8_28
    sens.V[3] = VIcp8_38
    sens.OM[1] = OMcp8_17
    sens.OM[2] = OMcp8_27
    sens.OM[3] = OMcp8_37
    sens.J[1,1] = (1.0)
    sens.J[1,5] = JTcp8_18_5
    sens.J[1,6] = JTcp8_18_6
    sens.J[1,7] = JTcp8_18_7
    sens.J[1,8] = ROcp8_77
    sens.J[2,2] = (1.0)
    sens.J[2,4] = JTcp8_28_4
    sens.J[2,5] = JTcp8_28_5
    sens.J[2,6] = JTcp8_28_6
    sens.J[2,7] = JTcp8_28_7
    sens.J[2,8] = ROcp8_87
    sens.J[3,3] = (1.0)
    sens.J[3,4] = JTcp8_38_4
    sens.J[3,5] = JTcp8_38_5
    sens.J[3,6] = JTcp8_38_6
    sens.J[3,7] = JTcp8_38_7
    sens.J[3,8] = ROcp8_97
    sens.J[4,4] = (1.0)
    sens.J[4,6] = S5
    sens.J[4,7] = ROcp8_46
    sens.J[5,5] = C4
    sens.J[5,6] = ROcp8_85
    sens.J[5,7] = ROcp8_56
    sens.J[6,5] = S4
    sens.J[6,6] = ROcp8_95
    sens.J[6,7] = ROcp8_66
    sens.A[1] = ACcp8_18
    sens.A[2] = ACcp8_28
    sens.A[3] = ACcp8_38
    sens.OMP[1] = OPcp8_17
    sens.OMP[2] = OPcp8_27
    sens.OMP[3] = OPcp8_37

  if (isens == 9): 

    ROcp9_25 = S4*S5
    ROcp9_35 = -C4*S5
    ROcp9_85 = -S4*C5
    ROcp9_95 = C4*C5
    ROcp9_16 = C5*C6
    ROcp9_26 = ROcp9_25*C6+C4*S6
    ROcp9_36 = ROcp9_35*C6+S4*S6
    ROcp9_46 = -C5*S6
    ROcp9_56 = -ROcp9_25*S6+C4*C6
    ROcp9_66 = -ROcp9_35*S6+S4*C6
    ROcp9_17 = ROcp9_16*C7-S5*S7
    ROcp9_27 = ROcp9_26*C7-ROcp9_85*S7
    ROcp9_37 = ROcp9_36*C7-ROcp9_95*S7
    ROcp9_77 = ROcp9_16*S7+S5*C7
    ROcp9_87 = ROcp9_26*S7+ROcp9_85*C7
    ROcp9_97 = ROcp9_36*S7+ROcp9_95*C7
    ROcp9_19 = ROcp9_17*C9-ROcp9_77*S9
    ROcp9_29 = ROcp9_27*C9-ROcp9_87*S9
    ROcp9_39 = ROcp9_37*C9-ROcp9_97*S9
    ROcp9_79 = ROcp9_17*S9+ROcp9_77*C9
    ROcp9_89 = ROcp9_27*S9+ROcp9_87*C9
    ROcp9_99 = ROcp9_37*S9+ROcp9_97*C9
    OMcp9_25 = qd[5]*C4
    OMcp9_35 = qd[5]*S4
    OPcp9_25 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp9_35 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp9_16 = qd[4]+qd[6]*S5
    OMcp9_26 = OMcp9_25+ROcp9_85*qd[6]
    OMcp9_36 = OMcp9_35+ROcp9_95*qd[6]
    OPcp9_16 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp9_25*ROcp9_95-OMcp9_35*ROcp9_85)
    OPcp9_26 = OPcp9_25+ROcp9_85*qdd[6]+qd[6]*(OMcp9_35*S5-ROcp9_95*qd[4])
    OPcp9_36 = OPcp9_35+ROcp9_95*qdd[6]+qd[6]*(-OMcp9_25*S5+ROcp9_85*qd[4])
    RLcp9_17 = s.dpt[3,1]*S5
    RLcp9_27 = ROcp9_85*s.dpt[3,1]
    RLcp9_37 = ROcp9_95*s.dpt[3,1]
    POcp9_17 = RLcp9_17+q[1]
    POcp9_27 = RLcp9_27+q[2]
    POcp9_37 = RLcp9_37+q[3]
    JTcp9_17_5 = -RLcp9_27*S4+RLcp9_37*C4
    JTcp9_27_5 = RLcp9_17*S4
    JTcp9_37_5 = -RLcp9_17*C4
    JTcp9_17_6 = -RLcp9_27*ROcp9_95+RLcp9_37*ROcp9_85
    JTcp9_27_6 = RLcp9_17*ROcp9_95-RLcp9_37*S5
    JTcp9_37_6 = -RLcp9_17*ROcp9_85+RLcp9_27*S5
    OMcp9_17 = OMcp9_16+ROcp9_46*qd[7]
    OMcp9_27 = OMcp9_26+ROcp9_56*qd[7]
    OMcp9_37 = OMcp9_36+ROcp9_66*qd[7]
    ORcp9_17 = OMcp9_26*RLcp9_37-OMcp9_36*RLcp9_27
    ORcp9_27 = -OMcp9_16*RLcp9_37+OMcp9_36*RLcp9_17
    ORcp9_37 = OMcp9_16*RLcp9_27-OMcp9_26*RLcp9_17
    VIcp9_17 = ORcp9_17+qd[1]
    VIcp9_27 = ORcp9_27+qd[2]
    VIcp9_37 = ORcp9_37+qd[3]
    OPcp9_17 = OPcp9_16+ROcp9_46*qdd[7]+qd[7]*(OMcp9_26*ROcp9_66-OMcp9_36*ROcp9_56)
    OPcp9_27 = OPcp9_26+ROcp9_56*qdd[7]+qd[7]*(-OMcp9_16*ROcp9_66+OMcp9_36*ROcp9_46)
    OPcp9_37 = OPcp9_36+ROcp9_66*qdd[7]+qd[7]*(OMcp9_16*ROcp9_56-OMcp9_26*ROcp9_46)
    ACcp9_17 = qdd[1]+OMcp9_26*ORcp9_37-OMcp9_36*ORcp9_27+OPcp9_26*RLcp9_37-OPcp9_36*RLcp9_27
    ACcp9_27 = qdd[2]-OMcp9_16*ORcp9_37+OMcp9_36*ORcp9_17-OPcp9_16*RLcp9_37+OPcp9_36*RLcp9_17
    ACcp9_37 = qdd[3]+OMcp9_16*ORcp9_27-OMcp9_26*ORcp9_17+OPcp9_16*RLcp9_27-OPcp9_26*RLcp9_17
    RLcp9_18 = ROcp9_77*Dz83
    RLcp9_28 = ROcp9_87*Dz83
    RLcp9_38 = ROcp9_97*Dz83
    POcp9_18 = POcp9_17+RLcp9_18
    POcp9_28 = POcp9_27+RLcp9_28
    POcp9_38 = POcp9_37+RLcp9_38
    JTcp9_28_4 = -RLcp9_37-RLcp9_38
    JTcp9_38_4 = RLcp9_27+RLcp9_28
    JTcp9_18_5 = JTcp9_17_5-RLcp9_28*S4+RLcp9_38*C4
    JTcp9_28_5 = JTcp9_27_5+RLcp9_18*S4
    JTcp9_38_5 = JTcp9_37_5-RLcp9_18*C4
    JTcp9_18_6 = JTcp9_17_6-RLcp9_28*ROcp9_95+RLcp9_38*ROcp9_85
    JTcp9_28_6 = JTcp9_27_6+RLcp9_18*ROcp9_95-RLcp9_38*S5
    JTcp9_38_6 = JTcp9_37_6-RLcp9_18*ROcp9_85+RLcp9_28*S5
    JTcp9_18_7 = -RLcp9_28*ROcp9_66+RLcp9_38*ROcp9_56
    JTcp9_28_7 = RLcp9_18*ROcp9_66-RLcp9_38*ROcp9_46
    JTcp9_38_7 = -RLcp9_18*ROcp9_56+RLcp9_28*ROcp9_46
    ORcp9_18 = OMcp9_27*RLcp9_38-OMcp9_37*RLcp9_28
    ORcp9_28 = -OMcp9_17*RLcp9_38+OMcp9_37*RLcp9_18
    ORcp9_38 = OMcp9_17*RLcp9_28-OMcp9_27*RLcp9_18
    VIcp9_18 = ORcp9_18+VIcp9_17+ROcp9_77*qd[8]
    VIcp9_28 = ORcp9_28+VIcp9_27+ROcp9_87*qd[8]
    VIcp9_38 = ORcp9_38+VIcp9_37+ROcp9_97*qd[8]
    ACcp9_18 = ACcp9_17+OMcp9_27*ORcp9_38-OMcp9_37*ORcp9_28+OPcp9_27*RLcp9_38-OPcp9_37*RLcp9_28+ROcp9_77*qdd[8]+(2.0)*qd[8]*(OMcp9_27*ROcp9_97-OMcp9_37*ROcp9_87)
    ACcp9_28 = ACcp9_27-OMcp9_17*ORcp9_38+OMcp9_37*ORcp9_18-OPcp9_17*RLcp9_38+OPcp9_37*RLcp9_18+ROcp9_87*qdd[8]+(2.0)*qd[8]*(-OMcp9_17*ROcp9_97+OMcp9_37*ROcp9_77)
    ACcp9_38 = ACcp9_37+OMcp9_17*ORcp9_28-OMcp9_27*ORcp9_18+OPcp9_17*RLcp9_28-OPcp9_27*RLcp9_18+ROcp9_97*qdd[8]+(2.0)*qd[8]*(OMcp9_17*ROcp9_87-OMcp9_27*ROcp9_77)
    RLcp9_19 = ROcp9_77*s.dpt[3,3]
    RLcp9_29 = ROcp9_87*s.dpt[3,3]
    RLcp9_39 = ROcp9_97*s.dpt[3,3]
    POcp9_19 = POcp9_18+RLcp9_19
    POcp9_29 = POcp9_28+RLcp9_29
    POcp9_39 = POcp9_38+RLcp9_39
    JTcp9_29_4 = JTcp9_28_4-RLcp9_39
    JTcp9_39_4 = JTcp9_38_4+RLcp9_29
    JTcp9_19_5 = JTcp9_18_5-RLcp9_29*S4+RLcp9_39*C4
    JTcp9_29_5 = JTcp9_28_5+RLcp9_19*S4
    JTcp9_39_5 = JTcp9_38_5-RLcp9_19*C4
    JTcp9_19_6 = JTcp9_18_6-RLcp9_29*ROcp9_95+RLcp9_39*ROcp9_85
    JTcp9_29_6 = JTcp9_28_6+RLcp9_19*ROcp9_95-RLcp9_39*S5
    JTcp9_39_6 = JTcp9_38_6-RLcp9_19*ROcp9_85+RLcp9_29*S5
    JTcp9_19_7 = JTcp9_18_7-RLcp9_29*ROcp9_66+RLcp9_39*ROcp9_56
    JTcp9_29_7 = JTcp9_28_7+RLcp9_19*ROcp9_66-RLcp9_39*ROcp9_46
    JTcp9_39_7 = JTcp9_38_7-RLcp9_19*ROcp9_56+RLcp9_29*ROcp9_46
    OMcp9_19 = OMcp9_17+ROcp9_46*qd[9]
    OMcp9_29 = OMcp9_27+ROcp9_56*qd[9]
    OMcp9_39 = OMcp9_37+ROcp9_66*qd[9]
    ORcp9_19 = OMcp9_27*RLcp9_39-OMcp9_37*RLcp9_29
    ORcp9_29 = -OMcp9_17*RLcp9_39+OMcp9_37*RLcp9_19
    ORcp9_39 = OMcp9_17*RLcp9_29-OMcp9_27*RLcp9_19
    VIcp9_19 = ORcp9_19+VIcp9_18
    VIcp9_29 = ORcp9_29+VIcp9_28
    VIcp9_39 = ORcp9_39+VIcp9_38
    OPcp9_19 = OPcp9_17+ROcp9_46*qdd[9]+qd[9]*(OMcp9_27*ROcp9_66-OMcp9_37*ROcp9_56)
    OPcp9_29 = OPcp9_27+ROcp9_56*qdd[9]+qd[9]*(-OMcp9_17*ROcp9_66+OMcp9_37*ROcp9_46)
    OPcp9_39 = OPcp9_37+ROcp9_66*qdd[9]+qd[9]*(OMcp9_17*ROcp9_56-OMcp9_27*ROcp9_46)
    ACcp9_19 = ACcp9_18+OMcp9_27*ORcp9_39-OMcp9_37*ORcp9_29+OPcp9_27*RLcp9_39-OPcp9_37*RLcp9_29
    ACcp9_29 = ACcp9_28-OMcp9_17*ORcp9_39+OMcp9_37*ORcp9_19-OPcp9_17*RLcp9_39+OPcp9_37*RLcp9_19
    ACcp9_39 = ACcp9_38+OMcp9_17*ORcp9_29-OMcp9_27*ORcp9_19+OPcp9_17*RLcp9_29-OPcp9_27*RLcp9_19
    sens.P[1] = POcp9_19
    sens.P[2] = POcp9_29
    sens.P[3] = POcp9_39
    sens.R[1,1] = ROcp9_19
    sens.R[1,2] = ROcp9_29
    sens.R[1,3] = ROcp9_39
    sens.R[2,1] = ROcp9_46
    sens.R[2,2] = ROcp9_56
    sens.R[2,3] = ROcp9_66
    sens.R[3,1] = ROcp9_79
    sens.R[3,2] = ROcp9_89
    sens.R[3,3] = ROcp9_99
    sens.V[1] = VIcp9_19
    sens.V[2] = VIcp9_29
    sens.V[3] = VIcp9_39
    sens.OM[1] = OMcp9_19
    sens.OM[2] = OMcp9_29
    sens.OM[3] = OMcp9_39
    sens.J[1,1] = (1.0)
    sens.J[1,5] = JTcp9_19_5
    sens.J[1,6] = JTcp9_19_6
    sens.J[1,7] = JTcp9_19_7
    sens.J[1,8] = ROcp9_77
    sens.J[2,2] = (1.0)
    sens.J[2,4] = JTcp9_29_4
    sens.J[2,5] = JTcp9_29_5
    sens.J[2,6] = JTcp9_29_6
    sens.J[2,7] = JTcp9_29_7
    sens.J[2,8] = ROcp9_87
    sens.J[3,3] = (1.0)
    sens.J[3,4] = JTcp9_39_4
    sens.J[3,5] = JTcp9_39_5
    sens.J[3,6] = JTcp9_39_6
    sens.J[3,7] = JTcp9_39_7
    sens.J[3,8] = ROcp9_97
    sens.J[4,4] = (1.0)
    sens.J[4,6] = S5
    sens.J[4,7] = ROcp9_46
    sens.J[4,9] = ROcp9_46
    sens.J[5,5] = C4
    sens.J[5,6] = ROcp9_85
    sens.J[5,7] = ROcp9_56
    sens.J[5,9] = ROcp9_56
    sens.J[6,5] = S4
    sens.J[6,6] = ROcp9_95
    sens.J[6,7] = ROcp9_66
    sens.J[6,9] = ROcp9_66
    sens.A[1] = ACcp9_19
    sens.A[2] = ACcp9_29
    sens.A[3] = ACcp9_39
    sens.OMP[1] = OPcp9_19
    sens.OMP[2] = OPcp9_29
    sens.OMP[3] = OPcp9_39

  if (isens == 10): 

    ROcp10_25 = S4*S5
    ROcp10_35 = -C4*S5
    ROcp10_85 = -S4*C5
    ROcp10_95 = C4*C5
    ROcp10_16 = C5*C6
    ROcp10_26 = ROcp10_25*C6+C4*S6
    ROcp10_36 = ROcp10_35*C6+S4*S6
    ROcp10_46 = -C5*S6
    ROcp10_56 = -ROcp10_25*S6+C4*C6
    ROcp10_66 = -ROcp10_35*S6+S4*C6
    ROcp10_17 = ROcp10_16*C7-S5*S7
    ROcp10_27 = ROcp10_26*C7-ROcp10_85*S7
    ROcp10_37 = ROcp10_36*C7-ROcp10_95*S7
    ROcp10_77 = ROcp10_16*S7+S5*C7
    ROcp10_87 = ROcp10_26*S7+ROcp10_85*C7
    ROcp10_97 = ROcp10_36*S7+ROcp10_95*C7
    ROcp10_19 = ROcp10_17*C9-ROcp10_77*S9
    ROcp10_29 = ROcp10_27*C9-ROcp10_87*S9
    ROcp10_39 = ROcp10_37*C9-ROcp10_97*S9
    ROcp10_79 = ROcp10_17*S9+ROcp10_77*C9
    ROcp10_89 = ROcp10_27*S9+ROcp10_87*C9
    ROcp10_99 = ROcp10_37*S9+ROcp10_97*C9
    ROcp10_110 = ROcp10_19*C10-ROcp10_79*S10
    ROcp10_210 = ROcp10_29*C10-ROcp10_89*S10
    ROcp10_310 = ROcp10_39*C10-ROcp10_99*S10
    ROcp10_710 = ROcp10_19*S10+ROcp10_79*C10
    ROcp10_810 = ROcp10_29*S10+ROcp10_89*C10
    ROcp10_910 = ROcp10_39*S10+ROcp10_99*C10
    OMcp10_25 = qd[5]*C4
    OMcp10_35 = qd[5]*S4
    OPcp10_25 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp10_35 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp10_16 = qd[4]+qd[6]*S5
    OMcp10_26 = OMcp10_25+ROcp10_85*qd[6]
    OMcp10_36 = OMcp10_35+ROcp10_95*qd[6]
    OPcp10_16 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp10_25*ROcp10_95-OMcp10_35*ROcp10_85)
    OPcp10_26 = OPcp10_25+ROcp10_85*qdd[6]+qd[6]*(OMcp10_35*S5-ROcp10_95*qd[4])
    OPcp10_36 = OPcp10_35+ROcp10_95*qdd[6]+qd[6]*(-OMcp10_25*S5+ROcp10_85*qd[4])
    RLcp10_17 = s.dpt[3,1]*S5
    RLcp10_27 = ROcp10_85*s.dpt[3,1]
    RLcp10_37 = ROcp10_95*s.dpt[3,1]
    POcp10_17 = RLcp10_17+q[1]
    POcp10_27 = RLcp10_27+q[2]
    POcp10_37 = RLcp10_37+q[3]
    JTcp10_17_5 = -RLcp10_27*S4+RLcp10_37*C4
    JTcp10_27_5 = RLcp10_17*S4
    JTcp10_37_5 = -RLcp10_17*C4
    JTcp10_17_6 = -RLcp10_27*ROcp10_95+RLcp10_37*ROcp10_85
    JTcp10_27_6 = RLcp10_17*ROcp10_95-RLcp10_37*S5
    JTcp10_37_6 = -RLcp10_17*ROcp10_85+RLcp10_27*S5
    OMcp10_17 = OMcp10_16+ROcp10_46*qd[7]
    OMcp10_27 = OMcp10_26+ROcp10_56*qd[7]
    OMcp10_37 = OMcp10_36+ROcp10_66*qd[7]
    ORcp10_17 = OMcp10_26*RLcp10_37-OMcp10_36*RLcp10_27
    ORcp10_27 = -OMcp10_16*RLcp10_37+OMcp10_36*RLcp10_17
    ORcp10_37 = OMcp10_16*RLcp10_27-OMcp10_26*RLcp10_17
    VIcp10_17 = ORcp10_17+qd[1]
    VIcp10_27 = ORcp10_27+qd[2]
    VIcp10_37 = ORcp10_37+qd[3]
    OPcp10_17 = OPcp10_16+ROcp10_46*qdd[7]+qd[7]*(OMcp10_26*ROcp10_66-OMcp10_36*ROcp10_56)
    OPcp10_27 = OPcp10_26+ROcp10_56*qdd[7]+qd[7]*(-OMcp10_16*ROcp10_66+OMcp10_36*ROcp10_46)
    OPcp10_37 = OPcp10_36+ROcp10_66*qdd[7]+qd[7]*(OMcp10_16*ROcp10_56-OMcp10_26*ROcp10_46)
    ACcp10_17 = qdd[1]+OMcp10_26*ORcp10_37-OMcp10_36*ORcp10_27+OPcp10_26*RLcp10_37-OPcp10_36*RLcp10_27
    ACcp10_27 = qdd[2]-OMcp10_16*ORcp10_37+OMcp10_36*ORcp10_17-OPcp10_16*RLcp10_37+OPcp10_36*RLcp10_17
    ACcp10_37 = qdd[3]+OMcp10_16*ORcp10_27-OMcp10_26*ORcp10_17+OPcp10_16*RLcp10_27-OPcp10_26*RLcp10_17
    RLcp10_18 = ROcp10_77*Dz83
    RLcp10_28 = ROcp10_87*Dz83
    RLcp10_38 = ROcp10_97*Dz83
    POcp10_18 = POcp10_17+RLcp10_18
    POcp10_28 = POcp10_27+RLcp10_28
    POcp10_38 = POcp10_37+RLcp10_38
    JTcp10_28_4 = -RLcp10_37-RLcp10_38
    JTcp10_38_4 = RLcp10_27+RLcp10_28
    JTcp10_18_5 = JTcp10_17_5-RLcp10_28*S4+RLcp10_38*C4
    JTcp10_28_5 = JTcp10_27_5+RLcp10_18*S4
    JTcp10_38_5 = JTcp10_37_5-RLcp10_18*C4
    JTcp10_18_6 = JTcp10_17_6-RLcp10_28*ROcp10_95+RLcp10_38*ROcp10_85
    JTcp10_28_6 = JTcp10_27_6+RLcp10_18*ROcp10_95-RLcp10_38*S5
    JTcp10_38_6 = JTcp10_37_6-RLcp10_18*ROcp10_85+RLcp10_28*S5
    JTcp10_18_7 = -RLcp10_28*ROcp10_66+RLcp10_38*ROcp10_56
    JTcp10_28_7 = RLcp10_18*ROcp10_66-RLcp10_38*ROcp10_46
    JTcp10_38_7 = -RLcp10_18*ROcp10_56+RLcp10_28*ROcp10_46
    ORcp10_18 = OMcp10_27*RLcp10_38-OMcp10_37*RLcp10_28
    ORcp10_28 = -OMcp10_17*RLcp10_38+OMcp10_37*RLcp10_18
    ORcp10_38 = OMcp10_17*RLcp10_28-OMcp10_27*RLcp10_18
    VIcp10_18 = ORcp10_18+VIcp10_17+ROcp10_77*qd[8]
    VIcp10_28 = ORcp10_28+VIcp10_27+ROcp10_87*qd[8]
    VIcp10_38 = ORcp10_38+VIcp10_37+ROcp10_97*qd[8]
    ACcp10_18 = ACcp10_17+OMcp10_27*ORcp10_38-OMcp10_37*ORcp10_28+OPcp10_27*RLcp10_38-OPcp10_37*RLcp10_28+ROcp10_77*qdd[8]+(2.0)*qd[8]*(OMcp10_27*ROcp10_97-OMcp10_37*ROcp10_87)
    ACcp10_28 = ACcp10_27-OMcp10_17*ORcp10_38+OMcp10_37*ORcp10_18-OPcp10_17*RLcp10_38+OPcp10_37*RLcp10_18+ROcp10_87*qdd[8]+(2.0)*qd[8]*(-OMcp10_17*ROcp10_97+OMcp10_37*ROcp10_77)
    ACcp10_38 = ACcp10_37+OMcp10_17*ORcp10_28-OMcp10_27*ORcp10_18+OPcp10_17*RLcp10_28-OPcp10_27*RLcp10_18+ROcp10_97*qdd[8]+(2.0)*qd[8]*(OMcp10_17*ROcp10_87-OMcp10_27*ROcp10_77)
    RLcp10_19 = ROcp10_77*s.dpt[3,3]
    RLcp10_29 = ROcp10_87*s.dpt[3,3]
    RLcp10_39 = ROcp10_97*s.dpt[3,3]
    POcp10_19 = POcp10_18+RLcp10_19
    POcp10_29 = POcp10_28+RLcp10_29
    POcp10_39 = POcp10_38+RLcp10_39
    JTcp10_29_4 = JTcp10_28_4-RLcp10_39
    JTcp10_39_4 = JTcp10_38_4+RLcp10_29
    JTcp10_19_5 = JTcp10_18_5-RLcp10_29*S4+RLcp10_39*C4
    JTcp10_29_5 = JTcp10_28_5+RLcp10_19*S4
    JTcp10_39_5 = JTcp10_38_5-RLcp10_19*C4
    JTcp10_19_6 = JTcp10_18_6-RLcp10_29*ROcp10_95+RLcp10_39*ROcp10_85
    JTcp10_29_6 = JTcp10_28_6+RLcp10_19*ROcp10_95-RLcp10_39*S5
    JTcp10_39_6 = JTcp10_38_6-RLcp10_19*ROcp10_85+RLcp10_29*S5
    JTcp10_19_7 = JTcp10_18_7-RLcp10_29*ROcp10_66+RLcp10_39*ROcp10_56
    JTcp10_29_7 = JTcp10_28_7+RLcp10_19*ROcp10_66-RLcp10_39*ROcp10_46
    JTcp10_39_7 = JTcp10_38_7-RLcp10_19*ROcp10_56+RLcp10_29*ROcp10_46
    OMcp10_19 = OMcp10_17+ROcp10_46*qd[9]
    OMcp10_29 = OMcp10_27+ROcp10_56*qd[9]
    OMcp10_39 = OMcp10_37+ROcp10_66*qd[9]
    ORcp10_19 = OMcp10_27*RLcp10_39-OMcp10_37*RLcp10_29
    ORcp10_29 = -OMcp10_17*RLcp10_39+OMcp10_37*RLcp10_19
    ORcp10_39 = OMcp10_17*RLcp10_29-OMcp10_27*RLcp10_19
    VIcp10_19 = ORcp10_19+VIcp10_18
    VIcp10_29 = ORcp10_29+VIcp10_28
    VIcp10_39 = ORcp10_39+VIcp10_38
    OPcp10_19 = OPcp10_17+ROcp10_46*qdd[9]+qd[9]*(OMcp10_27*ROcp10_66-OMcp10_37*ROcp10_56)
    OPcp10_29 = OPcp10_27+ROcp10_56*qdd[9]+qd[9]*(-OMcp10_17*ROcp10_66+OMcp10_37*ROcp10_46)
    OPcp10_39 = OPcp10_37+ROcp10_66*qdd[9]+qd[9]*(OMcp10_17*ROcp10_56-OMcp10_27*ROcp10_46)
    ACcp10_19 = ACcp10_18+OMcp10_27*ORcp10_39-OMcp10_37*ORcp10_29+OPcp10_27*RLcp10_39-OPcp10_37*RLcp10_29
    ACcp10_29 = ACcp10_28-OMcp10_17*ORcp10_39+OMcp10_37*ORcp10_19-OPcp10_17*RLcp10_39+OPcp10_37*RLcp10_19
    ACcp10_39 = ACcp10_38+OMcp10_17*ORcp10_29-OMcp10_27*ORcp10_19+OPcp10_17*RLcp10_29-OPcp10_27*RLcp10_19
    RLcp10_110 = ROcp10_79*s.dpt[3,4]
    RLcp10_210 = ROcp10_89*s.dpt[3,4]
    RLcp10_310 = ROcp10_99*s.dpt[3,4]
    POcp10_110 = POcp10_19+RLcp10_110
    POcp10_210 = POcp10_29+RLcp10_210
    POcp10_310 = POcp10_39+RLcp10_310
    JTcp10_210_4 = JTcp10_29_4-RLcp10_310
    JTcp10_310_4 = JTcp10_39_4+RLcp10_210
    JTcp10_110_5 = JTcp10_19_5-RLcp10_210*S4+RLcp10_310*C4
    JTcp10_210_5 = JTcp10_29_5+RLcp10_110*S4
    JTcp10_310_5 = JTcp10_39_5-RLcp10_110*C4
    JTcp10_110_6 = JTcp10_19_6-RLcp10_210*ROcp10_95+RLcp10_310*ROcp10_85
    JTcp10_210_6 = JTcp10_29_6+RLcp10_110*ROcp10_95-RLcp10_310*S5
    JTcp10_310_6 = JTcp10_39_6-RLcp10_110*ROcp10_85+RLcp10_210*S5
    JTcp10_110_7 = JTcp10_19_7-RLcp10_210*ROcp10_66+RLcp10_310*ROcp10_56
    JTcp10_210_7 = JTcp10_29_7+RLcp10_110*ROcp10_66-RLcp10_310*ROcp10_46
    JTcp10_310_7 = JTcp10_39_7-RLcp10_110*ROcp10_56+RLcp10_210*ROcp10_46
    JTcp10_110_9 = -RLcp10_210*ROcp10_66+RLcp10_310*ROcp10_56
    JTcp10_210_9 = RLcp10_110*ROcp10_66-RLcp10_310*ROcp10_46
    JTcp10_310_9 = -RLcp10_110*ROcp10_56+RLcp10_210*ROcp10_46
    OMcp10_110 = OMcp10_19+ROcp10_46*qd[10]
    OMcp10_210 = OMcp10_29+ROcp10_56*qd[10]
    OMcp10_310 = OMcp10_39+ROcp10_66*qd[10]
    ORcp10_110 = OMcp10_29*RLcp10_310-OMcp10_39*RLcp10_210
    ORcp10_210 = -OMcp10_19*RLcp10_310+OMcp10_39*RLcp10_110
    ORcp10_310 = OMcp10_19*RLcp10_210-OMcp10_29*RLcp10_110
    VIcp10_110 = ORcp10_110+VIcp10_19
    VIcp10_210 = ORcp10_210+VIcp10_29
    VIcp10_310 = ORcp10_310+VIcp10_39
    OPcp10_110 = OPcp10_19+ROcp10_46*qdd[10]+qd[10]*(OMcp10_29*ROcp10_66-OMcp10_39*ROcp10_56)
    OPcp10_210 = OPcp10_29+ROcp10_56*qdd[10]+qd[10]*(-OMcp10_19*ROcp10_66+OMcp10_39*ROcp10_46)
    OPcp10_310 = OPcp10_39+ROcp10_66*qdd[10]+qd[10]*(OMcp10_19*ROcp10_56-OMcp10_29*ROcp10_46)
    ACcp10_110 = ACcp10_19+OMcp10_29*ORcp10_310-OMcp10_39*ORcp10_210+OPcp10_29*RLcp10_310-OPcp10_39*RLcp10_210
    ACcp10_210 = ACcp10_29-OMcp10_19*ORcp10_310+OMcp10_39*ORcp10_110-OPcp10_19*RLcp10_310+OPcp10_39*RLcp10_110
    ACcp10_310 = ACcp10_39+OMcp10_19*ORcp10_210-OMcp10_29*ORcp10_110+OPcp10_19*RLcp10_210-OPcp10_29*RLcp10_110
    sens.P[1] = POcp10_110
    sens.P[2] = POcp10_210
    sens.P[3] = POcp10_310
    sens.R[1,1] = ROcp10_110
    sens.R[1,2] = ROcp10_210
    sens.R[1,3] = ROcp10_310
    sens.R[2,1] = ROcp10_46
    sens.R[2,2] = ROcp10_56
    sens.R[2,3] = ROcp10_66
    sens.R[3,1] = ROcp10_710
    sens.R[3,2] = ROcp10_810
    sens.R[3,3] = ROcp10_910
    sens.V[1] = VIcp10_110
    sens.V[2] = VIcp10_210
    sens.V[3] = VIcp10_310
    sens.OM[1] = OMcp10_110
    sens.OM[2] = OMcp10_210
    sens.OM[3] = OMcp10_310
    sens.J[1,1] = (1.0)
    sens.J[1,5] = JTcp10_110_5
    sens.J[1,6] = JTcp10_110_6
    sens.J[1,7] = JTcp10_110_7
    sens.J[1,8] = ROcp10_77
    sens.J[1,9] = JTcp10_110_9
    sens.J[2,2] = (1.0)
    sens.J[2,4] = JTcp10_210_4
    sens.J[2,5] = JTcp10_210_5
    sens.J[2,6] = JTcp10_210_6
    sens.J[2,7] = JTcp10_210_7
    sens.J[2,8] = ROcp10_87
    sens.J[2,9] = JTcp10_210_9
    sens.J[3,3] = (1.0)
    sens.J[3,4] = JTcp10_310_4
    sens.J[3,5] = JTcp10_310_5
    sens.J[3,6] = JTcp10_310_6
    sens.J[3,7] = JTcp10_310_7
    sens.J[3,8] = ROcp10_97
    sens.J[3,9] = JTcp10_310_9
    sens.J[4,4] = (1.0)
    sens.J[4,6] = S5
    sens.J[4,7] = ROcp10_46
    sens.J[4,9] = ROcp10_46
    sens.J[4,10] = ROcp10_46
    sens.J[5,5] = C4
    sens.J[5,6] = ROcp10_85
    sens.J[5,7] = ROcp10_56
    sens.J[5,9] = ROcp10_56
    sens.J[5,10] = ROcp10_56
    sens.J[6,5] = S4
    sens.J[6,6] = ROcp10_95
    sens.J[6,7] = ROcp10_66
    sens.J[6,9] = ROcp10_66
    sens.J[6,10] = ROcp10_66
    sens.A[1] = ACcp10_110
    sens.A[2] = ACcp10_210
    sens.A[3] = ACcp10_310
    sens.OMP[1] = OPcp10_110
    sens.OMP[2] = OPcp10_210
    sens.OMP[3] = OPcp10_310

 


# Number of continuation lines = 0


