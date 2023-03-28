# -*- coding: utf-8 -*-
"""Module for defining user function required to compute external forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2019

import sys
sys.path.insert(0, "/Users/matthieuxaussems/Documents/MBProjects/One_leg_test/User_function")

import numpy as np
import math
import useful_functions as u_f
import pandas as pd

#### exportation des donnees excel en numpy array
# no torque with inertia
Contact_force = pd.read_excel ("/Users/matthieuxaussems/Documents/MBProjects/One_leg_test/User_function/one_leg_gf.xlsx")
# no torque without inertia of the trunk
#Contact_force = pd.read_excel ("/Users/matthieuxaussems/Documents/MBProjects/One_leg_test/User_function/no_trunk_inertia_one_leg.xlsx")
# no torque without inertia of the trunk and without inertia of the tight
#Contact_force = pd.read_excel ("/Users/matthieuxaussems/Documents/MBProjects/One_leg_test/User_function/no_thighinertia.xlsx")
# no torque without inertia of the trunk and without inertia of the tight and without inertia of the shank
#Contact_force = pd.read_excel ("/Users/matthieuxaussems/Documents/MBProjects/One_leg_test/User_function/noshank_inertia.xlsx")
# no torque and no inertia except for foot and no heel force
#Contact_force = pd.read_excel ("/Users/matthieuxaussems/Documents/MBProjects/One_leg_test/User_function/No_heelforces.xlsx")
# no torque and no inertia except for foot and no ball force
#Contact_force = pd.read_excel ("/Users/matthieuxaussems/Documents/MBProjects/One_leg_test/User_function/No_ballforces.xlsx")
#no torque and inertia Rosenbrock
#Contact_force = pd.read_excel ("/Users/matthieuxaussems/Documents/MBProjects/One_leg_test/User_function/One_leg_rosenbrock.xlsx")

current_t = Contact_force["time"].to_numpy()
fz_ball = Contact_force["fy_balll_one_leg"].to_numpy()
fx_ball = Contact_force["fx_ball_one_leg"].to_numpy()
fz_heel = Contact_force["fy_heel_one_leg"].to_numpy()
fx_heel = Contact_force["fx_heel_one_leg"].to_numpy()

#mémoire

fz_ball_m = np.column_stack((fz_ball , current_t))
fx_ball_m = np.column_stack((fx_ball , current_t))
fz_heel_m = np.column_stack((fz_heel , current_t))
fx_heel_m = np.column_stack((fx_heel , current_t))

def user_ExtForces(PxF, RxF, VxF, OMxF, AxF, OMPxF, mbs_data, tsim, ixF):
    """Compute an user-specified external force.

    Parameters
    ----------
    PxF : numpy.ndarray
        Position vector (index starting at 1) of the force sensor expressed in
        the inertial frame: PxF[1:4] = [P_x, P_y, P_z]
    RxF : numpy.ndarray
        Rotation matrix (index starting at 1) from the inertial frame to the
        force sensor frame: Frame_sensor = RxF[1:4,1:4] * Frame_inertial
    VxF : numpy.ndarray
        Velocity vector (index starting at 1) of the force sensor expressed in
        the inertial frame: VxF[1:4] = [V_x, V_y, V_z]
    OMxF : numpy.ndarray
        Angular velocity vector (index starting at 1) of the force sensor
        expressed in the inertial frame: OMxF[1:4] = [OM_x, OM_y, OM_z]
    AxF : numpy.ndarray
        Acceleration vector (index starting at 1) of the force sensor expressed
        in the inertial frame: AxF[1:4] = [A_x, A_y, A_z]
    OMPxF : numpy.ndarray
        Angular acceleration vector (index starting at 1) of the force sensor
        expressed in the inertial frame: OMPxF[1:4] = [OMP_x, OMP_y, OMP_z]
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.
    ixF : int
        The ID identifying the computed force sensor.

    Notes
    -----
    For 1D numpy.ndarray with index starting at 1, the first index (array[0])
    must not be modified. The first index to be filled is array[1].

    For 2D numpy.ndarray with index starting at 1, the first row (mat[0, :]) and
    line (mat[:,0]) must not be modified. The subarray to be filled is mat[1:, 1:].

    Returns
    -------
    Swr : numpy.ndarray
        An array of length 10 equal to [0., Fx, Fy, Fz, Mx, My, Mz, dxF].
        F_# are the forces components expressed in inertial frame.
        M_# are the torques components expressed in inertial frame.
        dxF is an array of length 3 containing the component of the forces/torque
        application point expressed in the BODY-FIXED frame.
        This array is a specific line of MbsData.SWr.
    """
    #initialisation des parametres :
        
    Fx = 0.0
    Fy = 0.0
    Fz = 0.0
    Mx = 0.0
    My = 0.0
    Mz = 0.0
    
    idpt = mbs_data.xfidpt[ixF]
    dxF = mbs_data.dpt[1:, idpt]

   #initialisation des sensors de forces externes:
   
    Force_BallL = mbs_data.extforce_id["Force_BallL"]
    Force_HeelL = mbs_data.extforce_id["Force_HeelL"]
    
    if ixF == Force_BallL:
        
        Fz = - u_f.interpolation_memory(fz_ball_m, tsim)
        Fx = u_f.interpolation_memory(fx_ball_m, tsim)
    
    if ixF == Force_HeelL:
        
        Fz = - u_f.interpolation_memory(fz_heel_m, tsim)
        Fx = u_f.interpolation_memory(fx_heel_m, tsim)

    # Concatenating force, torque and force application point to returned array.
    # This must not be modified.
    Swr = mbs_data.SWr[ixF]
    Swr[1:] = [Fx, Fy, Fz, Mx, My, Mz, dxF[0], dxF[1], dxF[2]]

    return Swr
