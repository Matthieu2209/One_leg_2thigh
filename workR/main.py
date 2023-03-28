#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Script to run a direct dynamic analysis on a multibody system.

Summary
-------
This template loads the data file *.mbs and execute:
 - the coordinate partitioning module
 - the direct dynamic module (time integration of equations of motion).
 - if available, plot the time evolution of the first generalized coordinate.

It may have to be adapted and completed by the user.


Universite catholique de Louvain
CEREM : Centre for research in mechatronics

http://www.robotran.eu
Contact : info@robotran.be

(c) Universite catholique de Louvain
"""

# %%============================================================================
# Packages loading
# =============================================================================
try:
    import MBsysPy as Robotran
except:
    raise ImportError("MBsysPy not found/installed."
                      "See: https://www.robotran.eu/download/how-to-install/"
                      )

import numpy as np

# %%===========================================================================
# Project loading
# =============================================================================
mbs_data = Robotran.MbsData('../dataR/One_leg_test.mbs')

# %%===========================================================================
# Partitionning
# =============================================================================
mbs_data.process = 1
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)
mbs_part.run()

# %%===========================================================================
# Direct Dynamics
# =============================================================================
mbs_data.process = 3
mbs_dirdyn = Robotran.MbsDirdyn(mbs_data)
#print(Robotran.MbsDirdyn.set_options.__doc__)
mbs_dirdyn.set_options(dt0=125e-6, tf=0.245, save2file=1)
#mbs_dirdyn.set_options(integrator="Dopri5",verbose=1,rtoler=1.0e-3,atoler=1.0e-4,dt_max=0.1,dt0=1e-2, tf=0.245
#                    , save2file=1)
#mbs_dirdyn.set_options(integrator="Rosenbrock",verbose=1,rtoler=1.0e-3,atoler=1.0e-4,dt_max=0.1,dt0=1e-2, tf=0.25
#                       , save2file=1)
results = mbs_dirdyn.run()

# %%===========================================================================
# Plotting results
# =============================================================================
try:
    import matplotlib.pyplot as plt
except Exception:
    raise RuntimeError('Unable to load matplotlib, plotting results unavailable.')

# Figure creation
fig = plt.figure(num='Example of plot')
axis = fig.gca()

# Plotting data's
#print(results.q[1900, 0],np.pi/2 - results.q[1900, 10])
axis.plot(results.q[:, 0], np.pi/2 -results.q[:, 10], label='q[1]')
#axis.plot(results.q[:, 0], results.q[:, 8], label='q[8]')

# Figure enhancement
axis.grid(True)
axis.set_xlim(left=mbs_dirdyn.get_options('t0'), right=mbs_dirdyn.get_options('tf'))
axis.set_xlabel('Time (s)')
axis.set_ylabel('Coordinate value (m or rad)')

plt.show()
