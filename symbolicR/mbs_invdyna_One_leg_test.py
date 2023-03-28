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
#	==> Function: F2 - Recursive Inverse Dynamics of tree-like MBS
#
#	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
#

from math import sin, cos

def invdyna(phi,s,tsim):
    Qq = phi  # compatibility with symbolic generation
    q = s.q
    qd = s.qd
    qdd = s.qdd
 
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

 
# Forward Kinematics

    ALPHA33 = qdd[3]-s.g[3]
    ALPHA24 = qdd[2]*C4+ALPHA33*S4
    ALPHA34 = -qdd[2]*S4+ALPHA33*C4
    OM15 = qd[4]*C5
    OM35 = qd[4]*S5
    OMp15 = -qd[4]*qd[5]*S5+qdd[4]*C5
    OMp35 = qd[4]*qd[5]*C5+qdd[4]*S5
    ALPHA15 = qdd[1]*C5-ALPHA34*S5
    ALPHA35 = qdd[1]*S5+ALPHA34*C5
    OM16 = qd[5]*S6+OM15*C6
    OM26 = qd[5]*C6-OM15*S6
    OM36 = qd[6]+OM35
    OMp16 = C6*(OMp15+qd[5]*qd[6])+S6*(qdd[5]-qd[6]*OM15)
    OMp26 = C6*(qdd[5]-qd[6]*OM15)-S6*(OMp15+qd[5]*qd[6])
    OMp36 = qdd[6]+OMp35
    BS36 = OM16*OM36
    BS66 = OM26*OM36
    BS96 = -OM16*OM16-OM26*OM26
    BETA36 = BS36+OMp26
    BETA66 = BS66-OMp16
    ALPHA16 = ALPHA15*C6+ALPHA24*S6
    ALPHA26 = -ALPHA15*S6+ALPHA24*C6
    OM17 = OM16*C7-OM36*S7
    OM27 = qd[7]+OM26
    OM37 = OM16*S7+OM36*C7
    OMp17 = C7*(OMp16-qd[7]*OM36)-S7*(OMp36+qd[7]*OM16)
    OMp27 = qdd[7]+OMp26
    OMp37 = C7*(OMp36+qd[7]*OM16)+S7*(OMp16-qd[7]*OM36)
    BS37 = OM17*OM37
    BS67 = OM27*OM37
    BS97 = -OM17*OM17-OM27*OM27
    BETA37 = BS37+OMp27
    BETA67 = BS67-OMp17
    ALPHA17 = C7*(ALPHA16+BETA36*s.dpt[3,1])-S7*(ALPHA35+BS96*s.dpt[3,1])
    ALPHA27 = ALPHA26+BETA66*s.dpt[3,1]
    ALPHA37 = C7*(ALPHA35+BS96*s.dpt[3,1])+S7*(ALPHA16+BETA36*s.dpt[3,1])
    BS38 = OM17*OM37
    BS68 = OM27*OM37
    BS98 = -OM17*OM17-OM27*OM27
    BETA38 = BS38+OMp27
    BETA68 = BS68-OMp17
    ALPHA18 = ALPHA17+(2.0)*qd[8]*OM27+BETA37*Dz83
    ALPHA28 = ALPHA27-(2.0)*qd[8]*OM17+BETA67*Dz83
    ALPHA38 = qdd[8]+ALPHA37+BS97*Dz83
    OM19 = OM17*C9-OM37*S9
    OM29 = qd[9]+OM27
    OM39 = OM17*S9+OM37*C9
    OMp19 = C9*(OMp17-qd[9]*OM37)-S9*(OMp37+qd[9]*OM17)
    OMp29 = qdd[9]+OMp27
    OMp39 = C9*(OMp37+qd[9]*OM17)+S9*(OMp17-qd[9]*OM37)
    BS39 = OM19*OM39
    BS69 = OM29*OM39
    BS99 = -OM19*OM19-OM29*OM29
    BETA39 = BS39+OMp29
    BETA69 = BS69-OMp19
    ALPHA19 = C9*(ALPHA18+BETA38*s.dpt[3,3])-S9*(ALPHA38+BS98*s.dpt[3,3])
    ALPHA29 = ALPHA28+BETA68*s.dpt[3,3]
    ALPHA39 = C9*(ALPHA38+BS98*s.dpt[3,3])+S9*(ALPHA18+BETA38*s.dpt[3,3])
    OM110 = OM19*C10-OM39*S10
    OM210 = qd[10]+OM29
    OM310 = OM19*S10+OM39*C10
    OMp210 = qdd[10]+OMp29
    OMp310 = C10*(OMp39+qd[10]*OM19)+S10*(OMp19-qd[10]*OM39)
    BS110 = -OM210*OM210-OM310*OM310
    BS210 = OM110*OM210
    BS310 = OM110*OM310
    BETA410 = BS210+OMp310
    BETA710 = BS310-OMp210
    ALPHA110 = C10*(ALPHA19+BETA39*s.dpt[3,4])-S10*(ALPHA39+BS99*s.dpt[3,4])
    ALPHA210 = ALPHA29+BETA69*s.dpt[3,4]
    ALPHA310 = C10*(ALPHA39+BS99*s.dpt[3,4])+S10*(ALPHA19+BETA39*s.dpt[3,4])
 
# Backward Dynamics

    Fs110 = -s.frc[1,10]+s.m[10]*(ALPHA110+BS110*s.l[1,10])
    Fs210 = -s.frc[2,10]+s.m[10]*(ALPHA210+BETA410*s.l[1,10])
    Fs310 = -s.frc[3,10]+s.m[10]*(ALPHA310+BETA710*s.l[1,10])
    Cq110 = -s.trq[1,10]-s.In[5,10]*OM210*OM310
    Cq210 = -s.trq[2,10]+s.In[5,10]*OMp210-Fs310*s.l[1,10]
    Cq310 = -s.trq[3,10]+s.In[5,10]*OM110*OM210+Fs210*s.l[1,10]
    Fs19 = -s.frc[1,9]+s.m[9]*(ALPHA19+BETA39*s.l[3,9])
    Fs29 = -s.frc[2,9]+s.m[9]*(ALPHA29+BETA69*s.l[3,9])
    Fs39 = -s.frc[3,9]+s.m[9]*(ALPHA39+BS99*s.l[3,9])
    Fq19 = Fs19+Fs110*C10+Fs310*S10
    Fq29 = Fs210+Fs29
    Fq39 = Fs39-Fs110*S10+Fs310*C10
    Cq19 = -s.trq[1,9]-s.In[5,9]*OM29*OM39+Cq110*C10+Cq310*S10-Fs210*s.dpt[3,4]-Fs29*s.l[3,9]
    Cq29 = -s.trq[2,9]+Cq210+s.In[5,9]*OMp29+Fs19*s.l[3,9]+s.dpt[3,4]*(Fs110*C10+Fs310*S10)
    Cq39 = -s.trq[3,9]+s.In[5,9]*OM19*OM29-Cq110*S10+Cq310*C10
    Fs18 = -s.frc[1,8]+s.m[8]*(ALPHA18+BETA38*s.l[3,8])
    Fs28 = -s.frc[2,8]+s.m[8]*(ALPHA28+BETA68*s.l[3,8])
    Fs38 = -s.frc[3,8]+s.m[8]*(ALPHA38+BS98*s.l[3,8])
    Fq18 = Fs18+Fq19*C9+Fq39*S9
    Fq28 = Fq29+Fs28
    Fq38 = Fs38-Fq19*S9+Fq39*C9
    Cq18 = -s.trq[1,8]-s.In[5,8]*OM27*OM37+Cq19*C9+Cq39*S9-Fq29*s.dpt[3,3]-Fs28*s.l[3,8]
    Cq28 = -s.trq[2,8]+Cq29+s.In[5,8]*OMp27+Fs18*s.l[3,8]+s.dpt[3,3]*(Fq19*C9+Fq39*S9)
    Cq38 = -s.trq[3,8]+s.In[5,8]*OM17*OM27-Cq19*S9+Cq39*C9
    Fs17 = -s.frc[1,7]+s.m[7]*(ALPHA17+BETA37*s.l[3,7])
    Fs27 = -s.frc[2,7]+s.m[7]*(ALPHA27+BETA67*s.l[3,7])
    Fs37 = -s.frc[3,7]+s.m[7]*(ALPHA37+BS97*s.l[3,7])
    Fq17 = Fq18+Fs17
    Fq27 = Fq28+Fs27
    Fq37 = Fq38+Fs37
    Cq17 = -s.trq[1,7]+Cq18-s.In[5,7]*OM27*OM37-Fq28*Dz83-Fs27*s.l[3,7]
    Cq27 = -s.trq[2,7]+Cq28+s.In[5,7]*OMp27+Fq18*Dz83+Fs17*s.l[3,7]
    Cq37 = -s.trq[3,7]+Cq38+s.In[5,7]*OM17*OM27
    Fs16 = -s.frc[1,6]+s.m[6]*(ALPHA16+BETA36*s.l[3,6])
    Fs26 = -s.frc[2,6]+s.m[6]*(ALPHA26+BETA66*s.l[3,6])
    Fs36 = -s.frc[3,6]+s.m[6]*(ALPHA35+BS96*s.l[3,6])
    Fq16 = Fs16+Fq17*C7+Fq37*S7
    Fq26 = Fq27+Fs26
    Fq36 = Fs36-Fq17*S7+Fq37*C7
    Cq16 = -s.trq[1,6]-s.In[5,6]*OM26*OM36+Cq17*C7+Cq37*S7-Fq27*s.dpt[3,1]-Fs26*s.l[3,6]
    Cq26 = -s.trq[2,6]+Cq27+s.In[5,6]*OMp26+Fs16*s.l[3,6]+s.dpt[3,1]*(Fq17*C7+Fq37*S7)
    Cq36 = -s.trq[3,6]+s.In[5,6]*OM16*OM26-Cq17*S7+Cq37*C7
    Fq15 = Fq16*C6-Fq26*S6
    Fq25 = Fq16*S6+Fq26*C6
    Cq15 = Cq16*C6-Cq26*S6
    Cq25 = Cq16*S6+Cq26*C6
    Fq14 = Fq15*C5+Fq36*S5
    Fq34 = -Fq15*S5+Fq36*C5
    Cq14 = Cq15*C5+Cq36*S5
    Fq23 = Fq25*C4-Fq34*S4
    Fq33 = Fq25*S4+Fq34*C4
    Fq11 = -s.frc[1,1]+Fq14
 
# Symbolic model output

    Qq[1] = Fq11
    Qq[2] = Fq23
    Qq[3] = Fq33
    Qq[4] = Cq14
    Qq[5] = Cq25
    Qq[6] = Cq36
    Qq[7] = Cq27
    Qq[8] = Fq38
    Qq[9] = Cq29
    Qq[10] = Cq210

# Number of continuation lines = 0


