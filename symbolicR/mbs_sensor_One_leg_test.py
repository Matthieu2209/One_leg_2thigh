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

    ROcp1_25 = S4*S5
    ROcp1_35 = -C4*S5
    ROcp1_85 = -S4*C5
    ROcp1_95 = C4*C5
    ROcp1_16 = C5*C6
    ROcp1_26 = ROcp1_25*C6+C4*S6
    ROcp1_36 = ROcp1_35*C6+S4*S6
    ROcp1_46 = -C5*S6
    ROcp1_56 = -ROcp1_25*S6+C4*C6
    ROcp1_66 = -ROcp1_35*S6+S4*C6
    ROcp1_17 = ROcp1_16*C7-S5*S7
    ROcp1_27 = ROcp1_26*C7-ROcp1_85*S7
    ROcp1_37 = ROcp1_36*C7-ROcp1_95*S7
    ROcp1_77 = ROcp1_16*S7+S5*C7
    ROcp1_87 = ROcp1_26*S7+ROcp1_85*C7
    ROcp1_97 = ROcp1_36*S7+ROcp1_95*C7
    ROcp1_19 = ROcp1_17*C9-ROcp1_77*S9
    ROcp1_29 = ROcp1_27*C9-ROcp1_87*S9
    ROcp1_39 = ROcp1_37*C9-ROcp1_97*S9
    ROcp1_79 = ROcp1_17*S9+ROcp1_77*C9
    ROcp1_89 = ROcp1_27*S9+ROcp1_87*C9
    ROcp1_99 = ROcp1_37*S9+ROcp1_97*C9
    ROcp1_110 = ROcp1_19*C10-ROcp1_79*S10
    ROcp1_210 = ROcp1_29*C10-ROcp1_89*S10
    ROcp1_310 = ROcp1_39*C10-ROcp1_99*S10
    ROcp1_710 = ROcp1_19*S10+ROcp1_79*C10
    ROcp1_810 = ROcp1_29*S10+ROcp1_89*C10
    ROcp1_910 = ROcp1_39*S10+ROcp1_99*C10
    OMcp1_25 = qd[5]*C4
    OMcp1_35 = qd[5]*S4
    OPcp1_25 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp1_35 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp1_16 = qd[4]+qd[6]*S5
    OMcp1_26 = OMcp1_25+ROcp1_85*qd[6]
    OMcp1_36 = OMcp1_35+ROcp1_95*qd[6]
    OPcp1_16 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp1_25*ROcp1_95-OMcp1_35*ROcp1_85)
    OPcp1_26 = OPcp1_25+ROcp1_85*qdd[6]+qd[6]*(OMcp1_35*S5-ROcp1_95*qd[4])
    OPcp1_36 = OPcp1_35+ROcp1_95*qdd[6]+qd[6]*(-OMcp1_25*S5+ROcp1_85*qd[4])
    RLcp1_17 = s.dpt[3,1]*S5
    RLcp1_27 = ROcp1_85*s.dpt[3,1]
    RLcp1_37 = ROcp1_95*s.dpt[3,1]
    POcp1_17 = RLcp1_17+q[1]
    POcp1_27 = RLcp1_27+q[2]
    POcp1_37 = RLcp1_37+q[3]
    OMcp1_17 = OMcp1_16+ROcp1_46*qd[7]
    OMcp1_27 = OMcp1_26+ROcp1_56*qd[7]
    OMcp1_37 = OMcp1_36+ROcp1_66*qd[7]
    ORcp1_17 = OMcp1_26*RLcp1_37-OMcp1_36*RLcp1_27
    ORcp1_27 = -OMcp1_16*RLcp1_37+OMcp1_36*RLcp1_17
    ORcp1_37 = OMcp1_16*RLcp1_27-OMcp1_26*RLcp1_17
    VIcp1_17 = ORcp1_17+qd[1]
    VIcp1_27 = ORcp1_27+qd[2]
    VIcp1_37 = ORcp1_37+qd[3]
    OPcp1_17 = OPcp1_16+ROcp1_46*qdd[7]+qd[7]*(OMcp1_26*ROcp1_66-OMcp1_36*ROcp1_56)
    OPcp1_27 = OPcp1_26+ROcp1_56*qdd[7]+qd[7]*(-OMcp1_16*ROcp1_66+OMcp1_36*ROcp1_46)
    OPcp1_37 = OPcp1_36+ROcp1_66*qdd[7]+qd[7]*(OMcp1_16*ROcp1_56-OMcp1_26*ROcp1_46)
    ACcp1_17 = qdd[1]+OMcp1_26*ORcp1_37-OMcp1_36*ORcp1_27+OPcp1_26*RLcp1_37-OPcp1_36*RLcp1_27
    ACcp1_27 = qdd[2]-OMcp1_16*ORcp1_37+OMcp1_36*ORcp1_17-OPcp1_16*RLcp1_37+OPcp1_36*RLcp1_17
    ACcp1_37 = qdd[3]+OMcp1_16*ORcp1_27-OMcp1_26*ORcp1_17+OPcp1_16*RLcp1_27-OPcp1_26*RLcp1_17
    RLcp1_18 = ROcp1_77*Dz83
    RLcp1_28 = ROcp1_87*Dz83
    RLcp1_38 = ROcp1_97*Dz83
    POcp1_18 = POcp1_17+RLcp1_18
    POcp1_28 = POcp1_27+RLcp1_28
    POcp1_38 = POcp1_37+RLcp1_38
    ORcp1_18 = OMcp1_27*RLcp1_38-OMcp1_37*RLcp1_28
    ORcp1_28 = -OMcp1_17*RLcp1_38+OMcp1_37*RLcp1_18
    ORcp1_38 = OMcp1_17*RLcp1_28-OMcp1_27*RLcp1_18
    VIcp1_18 = ORcp1_18+VIcp1_17+ROcp1_77*qd[8]
    VIcp1_28 = ORcp1_28+VIcp1_27+ROcp1_87*qd[8]
    VIcp1_38 = ORcp1_38+VIcp1_37+ROcp1_97*qd[8]
    ACcp1_18 = ACcp1_17+OMcp1_27*ORcp1_38-OMcp1_37*ORcp1_28+OPcp1_27*RLcp1_38-OPcp1_37*RLcp1_28+ROcp1_77*qdd[8]+(2.0)*qd[8]*(OMcp1_27*ROcp1_97-OMcp1_37*ROcp1_87)
    ACcp1_28 = ACcp1_27-OMcp1_17*ORcp1_38+OMcp1_37*ORcp1_18-OPcp1_17*RLcp1_38+OPcp1_37*RLcp1_18+ROcp1_87*qdd[8]+(2.0)*qd[8]*(-OMcp1_17*ROcp1_97+OMcp1_37*ROcp1_77)
    ACcp1_38 = ACcp1_37+OMcp1_17*ORcp1_28-OMcp1_27*ORcp1_18+OPcp1_17*RLcp1_28-OPcp1_27*RLcp1_18+ROcp1_97*qdd[8]+(2.0)*qd[8]*(OMcp1_17*ROcp1_87-OMcp1_27*ROcp1_77)
    RLcp1_19 = ROcp1_77*s.dpt[3,3]
    RLcp1_29 = ROcp1_87*s.dpt[3,3]
    RLcp1_39 = ROcp1_97*s.dpt[3,3]
    POcp1_19 = POcp1_18+RLcp1_19
    POcp1_29 = POcp1_28+RLcp1_29
    POcp1_39 = POcp1_38+RLcp1_39
    OMcp1_19 = OMcp1_17+ROcp1_46*qd[9]
    OMcp1_29 = OMcp1_27+ROcp1_56*qd[9]
    OMcp1_39 = OMcp1_37+ROcp1_66*qd[9]
    ORcp1_19 = OMcp1_27*RLcp1_39-OMcp1_37*RLcp1_29
    ORcp1_29 = -OMcp1_17*RLcp1_39+OMcp1_37*RLcp1_19
    ORcp1_39 = OMcp1_17*RLcp1_29-OMcp1_27*RLcp1_19
    VIcp1_19 = ORcp1_19+VIcp1_18
    VIcp1_29 = ORcp1_29+VIcp1_28
    VIcp1_39 = ORcp1_39+VIcp1_38
    OPcp1_19 = OPcp1_17+ROcp1_46*qdd[9]+qd[9]*(OMcp1_27*ROcp1_66-OMcp1_37*ROcp1_56)
    OPcp1_29 = OPcp1_27+ROcp1_56*qdd[9]+qd[9]*(-OMcp1_17*ROcp1_66+OMcp1_37*ROcp1_46)
    OPcp1_39 = OPcp1_37+ROcp1_66*qdd[9]+qd[9]*(OMcp1_17*ROcp1_56-OMcp1_27*ROcp1_46)
    ACcp1_19 = ACcp1_18+OMcp1_27*ORcp1_39-OMcp1_37*ORcp1_29+OPcp1_27*RLcp1_39-OPcp1_37*RLcp1_29
    ACcp1_29 = ACcp1_28-OMcp1_17*ORcp1_39+OMcp1_37*ORcp1_19-OPcp1_17*RLcp1_39+OPcp1_37*RLcp1_19
    ACcp1_39 = ACcp1_38+OMcp1_17*ORcp1_29-OMcp1_27*ORcp1_19+OPcp1_17*RLcp1_29-OPcp1_27*RLcp1_19
    RLcp1_110 = ROcp1_79*s.dpt[3,4]
    RLcp1_210 = ROcp1_89*s.dpt[3,4]
    RLcp1_310 = ROcp1_99*s.dpt[3,4]
    POcp1_110 = POcp1_19+RLcp1_110
    POcp1_210 = POcp1_29+RLcp1_210
    POcp1_310 = POcp1_39+RLcp1_310
    OMcp1_110 = OMcp1_19+ROcp1_46*qd[10]
    OMcp1_210 = OMcp1_29+ROcp1_56*qd[10]
    OMcp1_310 = OMcp1_39+ROcp1_66*qd[10]
    ORcp1_110 = OMcp1_29*RLcp1_310-OMcp1_39*RLcp1_210
    ORcp1_210 = -OMcp1_19*RLcp1_310+OMcp1_39*RLcp1_110
    ORcp1_310 = OMcp1_19*RLcp1_210-OMcp1_29*RLcp1_110
    VIcp1_110 = ORcp1_110+VIcp1_19
    VIcp1_210 = ORcp1_210+VIcp1_29
    VIcp1_310 = ORcp1_310+VIcp1_39
    OPcp1_110 = OPcp1_19+ROcp1_46*qdd[10]+qd[10]*(OMcp1_29*ROcp1_66-OMcp1_39*ROcp1_56)
    OPcp1_210 = OPcp1_29+ROcp1_56*qdd[10]+qd[10]*(-OMcp1_19*ROcp1_66+OMcp1_39*ROcp1_46)
    OPcp1_310 = OPcp1_39+ROcp1_66*qdd[10]+qd[10]*(OMcp1_19*ROcp1_56-OMcp1_29*ROcp1_46)
    ACcp1_110 = ACcp1_19+OMcp1_29*ORcp1_310-OMcp1_39*ORcp1_210+OPcp1_29*RLcp1_310-OPcp1_39*RLcp1_210
    ACcp1_210 = ACcp1_29-OMcp1_19*ORcp1_310+OMcp1_39*ORcp1_110-OPcp1_19*RLcp1_310+OPcp1_39*RLcp1_110
    ACcp1_310 = ACcp1_39+OMcp1_19*ORcp1_210-OMcp1_29*ORcp1_110+OPcp1_19*RLcp1_210-OPcp1_29*RLcp1_110
    RLcp1_111 = ROcp1_110*s.dpt[1,5]
    RLcp1_211 = ROcp1_210*s.dpt[1,5]
    RLcp1_311 = ROcp1_310*s.dpt[1,5]
    POcp1_111 = POcp1_110+RLcp1_111
    POcp1_211 = POcp1_210+RLcp1_211
    POcp1_311 = POcp1_310+RLcp1_311
    ORcp1_111 = OMcp1_210*RLcp1_311-OMcp1_310*RLcp1_211
    ORcp1_211 = -OMcp1_110*RLcp1_311+OMcp1_310*RLcp1_111
    ORcp1_311 = OMcp1_110*RLcp1_211-OMcp1_210*RLcp1_111
    VIcp1_111 = ORcp1_111+VIcp1_110
    VIcp1_211 = ORcp1_211+VIcp1_210
    VIcp1_311 = ORcp1_311+VIcp1_310
    ACcp1_111 = ACcp1_110+OMcp1_210*ORcp1_311-OMcp1_310*ORcp1_211+OPcp1_210*RLcp1_311-OPcp1_310*RLcp1_211
    ACcp1_211 = ACcp1_210-OMcp1_110*ORcp1_311+OMcp1_310*ORcp1_111-OPcp1_110*RLcp1_311+OPcp1_310*RLcp1_111
    ACcp1_311 = ACcp1_310+OMcp1_110*ORcp1_211-OMcp1_210*ORcp1_111+OPcp1_110*RLcp1_211-OPcp1_210*RLcp1_111
    sens.P[1] = POcp1_111
    sens.P[2] = POcp1_211
    sens.P[3] = POcp1_311
    sens.R[1,1] = ROcp1_110
    sens.R[1,2] = ROcp1_210
    sens.R[1,3] = ROcp1_310
    sens.R[2,1] = ROcp1_46
    sens.R[2,2] = ROcp1_56
    sens.R[2,3] = ROcp1_66
    sens.R[3,1] = ROcp1_710
    sens.R[3,2] = ROcp1_810
    sens.R[3,3] = ROcp1_910
    sens.V[1] = VIcp1_111
    sens.V[2] = VIcp1_211
    sens.V[3] = VIcp1_311
    sens.OM[1] = OMcp1_110
    sens.OM[2] = OMcp1_210
    sens.OM[3] = OMcp1_310
    sens.A[1] = ACcp1_111
    sens.A[2] = ACcp1_211
    sens.A[3] = ACcp1_311
    sens.OMP[1] = OPcp1_110
    sens.OMP[2] = OPcp1_210
    sens.OMP[3] = OPcp1_310

  if (isens == 2): 

    ROcp2_25 = S4*S5
    ROcp2_35 = -C4*S5
    ROcp2_85 = -S4*C5
    ROcp2_95 = C4*C5
    ROcp2_16 = C5*C6
    ROcp2_26 = ROcp2_25*C6+C4*S6
    ROcp2_36 = ROcp2_35*C6+S4*S6
    ROcp2_46 = -C5*S6
    ROcp2_56 = -ROcp2_25*S6+C4*C6
    ROcp2_66 = -ROcp2_35*S6+S4*C6
    ROcp2_17 = ROcp2_16*C7-S5*S7
    ROcp2_27 = ROcp2_26*C7-ROcp2_85*S7
    ROcp2_37 = ROcp2_36*C7-ROcp2_95*S7
    ROcp2_77 = ROcp2_16*S7+S5*C7
    ROcp2_87 = ROcp2_26*S7+ROcp2_85*C7
    ROcp2_97 = ROcp2_36*S7+ROcp2_95*C7
    ROcp2_19 = ROcp2_17*C9-ROcp2_77*S9
    ROcp2_29 = ROcp2_27*C9-ROcp2_87*S9
    ROcp2_39 = ROcp2_37*C9-ROcp2_97*S9
    ROcp2_79 = ROcp2_17*S9+ROcp2_77*C9
    ROcp2_89 = ROcp2_27*S9+ROcp2_87*C9
    ROcp2_99 = ROcp2_37*S9+ROcp2_97*C9
    ROcp2_110 = ROcp2_19*C10-ROcp2_79*S10
    ROcp2_210 = ROcp2_29*C10-ROcp2_89*S10
    ROcp2_310 = ROcp2_39*C10-ROcp2_99*S10
    ROcp2_710 = ROcp2_19*S10+ROcp2_79*C10
    ROcp2_810 = ROcp2_29*S10+ROcp2_89*C10
    ROcp2_910 = ROcp2_39*S10+ROcp2_99*C10
    OMcp2_25 = qd[5]*C4
    OMcp2_35 = qd[5]*S4
    OPcp2_25 = qdd[5]*C4-qd[4]*qd[5]*S4
    OPcp2_35 = qdd[5]*S4+qd[4]*qd[5]*C4
    OMcp2_16 = qd[4]+qd[6]*S5
    OMcp2_26 = OMcp2_25+ROcp2_85*qd[6]
    OMcp2_36 = OMcp2_35+ROcp2_95*qd[6]
    OPcp2_16 = qdd[4]+qdd[6]*S5+qd[6]*(OMcp2_25*ROcp2_95-OMcp2_35*ROcp2_85)
    OPcp2_26 = OPcp2_25+ROcp2_85*qdd[6]+qd[6]*(OMcp2_35*S5-ROcp2_95*qd[4])
    OPcp2_36 = OPcp2_35+ROcp2_95*qdd[6]+qd[6]*(-OMcp2_25*S5+ROcp2_85*qd[4])
    RLcp2_17 = s.dpt[3,1]*S5
    RLcp2_27 = ROcp2_85*s.dpt[3,1]
    RLcp2_37 = ROcp2_95*s.dpt[3,1]
    POcp2_17 = RLcp2_17+q[1]
    POcp2_27 = RLcp2_27+q[2]
    POcp2_37 = RLcp2_37+q[3]
    OMcp2_17 = OMcp2_16+ROcp2_46*qd[7]
    OMcp2_27 = OMcp2_26+ROcp2_56*qd[7]
    OMcp2_37 = OMcp2_36+ROcp2_66*qd[7]
    ORcp2_17 = OMcp2_26*RLcp2_37-OMcp2_36*RLcp2_27
    ORcp2_27 = -OMcp2_16*RLcp2_37+OMcp2_36*RLcp2_17
    ORcp2_37 = OMcp2_16*RLcp2_27-OMcp2_26*RLcp2_17
    VIcp2_17 = ORcp2_17+qd[1]
    VIcp2_27 = ORcp2_27+qd[2]
    VIcp2_37 = ORcp2_37+qd[3]
    OPcp2_17 = OPcp2_16+ROcp2_46*qdd[7]+qd[7]*(OMcp2_26*ROcp2_66-OMcp2_36*ROcp2_56)
    OPcp2_27 = OPcp2_26+ROcp2_56*qdd[7]+qd[7]*(-OMcp2_16*ROcp2_66+OMcp2_36*ROcp2_46)
    OPcp2_37 = OPcp2_36+ROcp2_66*qdd[7]+qd[7]*(OMcp2_16*ROcp2_56-OMcp2_26*ROcp2_46)
    ACcp2_17 = qdd[1]+OMcp2_26*ORcp2_37-OMcp2_36*ORcp2_27+OPcp2_26*RLcp2_37-OPcp2_36*RLcp2_27
    ACcp2_27 = qdd[2]-OMcp2_16*ORcp2_37+OMcp2_36*ORcp2_17-OPcp2_16*RLcp2_37+OPcp2_36*RLcp2_17
    ACcp2_37 = qdd[3]+OMcp2_16*ORcp2_27-OMcp2_26*ORcp2_17+OPcp2_16*RLcp2_27-OPcp2_26*RLcp2_17
    RLcp2_18 = ROcp2_77*Dz83
    RLcp2_28 = ROcp2_87*Dz83
    RLcp2_38 = ROcp2_97*Dz83
    POcp2_18 = POcp2_17+RLcp2_18
    POcp2_28 = POcp2_27+RLcp2_28
    POcp2_38 = POcp2_37+RLcp2_38
    ORcp2_18 = OMcp2_27*RLcp2_38-OMcp2_37*RLcp2_28
    ORcp2_28 = -OMcp2_17*RLcp2_38+OMcp2_37*RLcp2_18
    ORcp2_38 = OMcp2_17*RLcp2_28-OMcp2_27*RLcp2_18
    VIcp2_18 = ORcp2_18+VIcp2_17+ROcp2_77*qd[8]
    VIcp2_28 = ORcp2_28+VIcp2_27+ROcp2_87*qd[8]
    VIcp2_38 = ORcp2_38+VIcp2_37+ROcp2_97*qd[8]
    ACcp2_18 = ACcp2_17+OMcp2_27*ORcp2_38-OMcp2_37*ORcp2_28+OPcp2_27*RLcp2_38-OPcp2_37*RLcp2_28+ROcp2_77*qdd[8]+(2.0)*qd[8]*(OMcp2_27*ROcp2_97-OMcp2_37*ROcp2_87)
    ACcp2_28 = ACcp2_27-OMcp2_17*ORcp2_38+OMcp2_37*ORcp2_18-OPcp2_17*RLcp2_38+OPcp2_37*RLcp2_18+ROcp2_87*qdd[8]+(2.0)*qd[8]*(-OMcp2_17*ROcp2_97+OMcp2_37*ROcp2_77)
    ACcp2_38 = ACcp2_37+OMcp2_17*ORcp2_28-OMcp2_27*ORcp2_18+OPcp2_17*RLcp2_28-OPcp2_27*RLcp2_18+ROcp2_97*qdd[8]+(2.0)*qd[8]*(OMcp2_17*ROcp2_87-OMcp2_27*ROcp2_77)
    RLcp2_19 = ROcp2_77*s.dpt[3,3]
    RLcp2_29 = ROcp2_87*s.dpt[3,3]
    RLcp2_39 = ROcp2_97*s.dpt[3,3]
    POcp2_19 = POcp2_18+RLcp2_19
    POcp2_29 = POcp2_28+RLcp2_29
    POcp2_39 = POcp2_38+RLcp2_39
    OMcp2_19 = OMcp2_17+ROcp2_46*qd[9]
    OMcp2_29 = OMcp2_27+ROcp2_56*qd[9]
    OMcp2_39 = OMcp2_37+ROcp2_66*qd[9]
    ORcp2_19 = OMcp2_27*RLcp2_39-OMcp2_37*RLcp2_29
    ORcp2_29 = -OMcp2_17*RLcp2_39+OMcp2_37*RLcp2_19
    ORcp2_39 = OMcp2_17*RLcp2_29-OMcp2_27*RLcp2_19
    VIcp2_19 = ORcp2_19+VIcp2_18
    VIcp2_29 = ORcp2_29+VIcp2_28
    VIcp2_39 = ORcp2_39+VIcp2_38
    OPcp2_19 = OPcp2_17+ROcp2_46*qdd[9]+qd[9]*(OMcp2_27*ROcp2_66-OMcp2_37*ROcp2_56)
    OPcp2_29 = OPcp2_27+ROcp2_56*qdd[9]+qd[9]*(-OMcp2_17*ROcp2_66+OMcp2_37*ROcp2_46)
    OPcp2_39 = OPcp2_37+ROcp2_66*qdd[9]+qd[9]*(OMcp2_17*ROcp2_56-OMcp2_27*ROcp2_46)
    ACcp2_19 = ACcp2_18+OMcp2_27*ORcp2_39-OMcp2_37*ORcp2_29+OPcp2_27*RLcp2_39-OPcp2_37*RLcp2_29
    ACcp2_29 = ACcp2_28-OMcp2_17*ORcp2_39+OMcp2_37*ORcp2_19-OPcp2_17*RLcp2_39+OPcp2_37*RLcp2_19
    ACcp2_39 = ACcp2_38+OMcp2_17*ORcp2_29-OMcp2_27*ORcp2_19+OPcp2_17*RLcp2_29-OPcp2_27*RLcp2_19
    RLcp2_110 = ROcp2_79*s.dpt[3,4]
    RLcp2_210 = ROcp2_89*s.dpt[3,4]
    RLcp2_310 = ROcp2_99*s.dpt[3,4]
    POcp2_110 = POcp2_19+RLcp2_110
    POcp2_210 = POcp2_29+RLcp2_210
    POcp2_310 = POcp2_39+RLcp2_310
    OMcp2_110 = OMcp2_19+ROcp2_46*qd[10]
    OMcp2_210 = OMcp2_29+ROcp2_56*qd[10]
    OMcp2_310 = OMcp2_39+ROcp2_66*qd[10]
    ORcp2_110 = OMcp2_29*RLcp2_310-OMcp2_39*RLcp2_210
    ORcp2_210 = -OMcp2_19*RLcp2_310+OMcp2_39*RLcp2_110
    ORcp2_310 = OMcp2_19*RLcp2_210-OMcp2_29*RLcp2_110
    VIcp2_110 = ORcp2_110+VIcp2_19
    VIcp2_210 = ORcp2_210+VIcp2_29
    VIcp2_310 = ORcp2_310+VIcp2_39
    OPcp2_110 = OPcp2_19+ROcp2_46*qdd[10]+qd[10]*(OMcp2_29*ROcp2_66-OMcp2_39*ROcp2_56)
    OPcp2_210 = OPcp2_29+ROcp2_56*qdd[10]+qd[10]*(-OMcp2_19*ROcp2_66+OMcp2_39*ROcp2_46)
    OPcp2_310 = OPcp2_39+ROcp2_66*qdd[10]+qd[10]*(OMcp2_19*ROcp2_56-OMcp2_29*ROcp2_46)
    ACcp2_110 = ACcp2_19+OMcp2_29*ORcp2_310-OMcp2_39*ORcp2_210+OPcp2_29*RLcp2_310-OPcp2_39*RLcp2_210
    ACcp2_210 = ACcp2_29-OMcp2_19*ORcp2_310+OMcp2_39*ORcp2_110-OPcp2_19*RLcp2_310+OPcp2_39*RLcp2_110
    ACcp2_310 = ACcp2_39+OMcp2_19*ORcp2_210-OMcp2_29*ORcp2_110+OPcp2_19*RLcp2_210-OPcp2_29*RLcp2_110
    RLcp2_111 = ROcp2_110*s.dpt[1,6]
    RLcp2_211 = ROcp2_210*s.dpt[1,6]
    RLcp2_311 = ROcp2_310*s.dpt[1,6]
    POcp2_111 = POcp2_110+RLcp2_111
    POcp2_211 = POcp2_210+RLcp2_211
    POcp2_311 = POcp2_310+RLcp2_311
    ORcp2_111 = OMcp2_210*RLcp2_311-OMcp2_310*RLcp2_211
    ORcp2_211 = -OMcp2_110*RLcp2_311+OMcp2_310*RLcp2_111
    ORcp2_311 = OMcp2_110*RLcp2_211-OMcp2_210*RLcp2_111
    VIcp2_111 = ORcp2_111+VIcp2_110
    VIcp2_211 = ORcp2_211+VIcp2_210
    VIcp2_311 = ORcp2_311+VIcp2_310
    ACcp2_111 = ACcp2_110+OMcp2_210*ORcp2_311-OMcp2_310*ORcp2_211+OPcp2_210*RLcp2_311-OPcp2_310*RLcp2_211
    ACcp2_211 = ACcp2_210-OMcp2_110*ORcp2_311+OMcp2_310*ORcp2_111-OPcp2_110*RLcp2_311+OPcp2_310*RLcp2_111
    ACcp2_311 = ACcp2_310+OMcp2_110*ORcp2_211-OMcp2_210*ORcp2_111+OPcp2_110*RLcp2_211-OPcp2_210*RLcp2_111
    sens.P[1] = POcp2_111
    sens.P[2] = POcp2_211
    sens.P[3] = POcp2_311
    sens.R[1,1] = ROcp2_110
    sens.R[1,2] = ROcp2_210
    sens.R[1,3] = ROcp2_310
    sens.R[2,1] = ROcp2_46
    sens.R[2,2] = ROcp2_56
    sens.R[2,3] = ROcp2_66
    sens.R[3,1] = ROcp2_710
    sens.R[3,2] = ROcp2_810
    sens.R[3,3] = ROcp2_910
    sens.V[1] = VIcp2_111
    sens.V[2] = VIcp2_211
    sens.V[3] = VIcp2_311
    sens.OM[1] = OMcp2_110
    sens.OM[2] = OMcp2_210
    sens.OM[3] = OMcp2_310
    sens.A[1] = ACcp2_111
    sens.A[2] = ACcp2_211
    sens.A[3] = ACcp2_311
    sens.OMP[1] = OPcp2_110
    sens.OMP[2] = OPcp2_210
    sens.OMP[3] = OPcp2_310

 


# Number of continuation lines = 0


