import numpy as np
from numpy import pi
from scipy.optimize import fsolve
from solve_eq_NL import solve_eq_NL
from trace_rob import trace_rob
from MGI_analytique import MGI_analytique 



L1=0.10  #long segment 1
L2=0.10  #long segment 2
Rb=0.1322594  # Rayon base
Re=0.07 # Rayon effecteur



pos_eff=[0.0, 0.0, 0]


# comme on utilise fsolve : https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.fsolve.html
# on a besoin de donner une estimation de la solution (comme ce qu'on a fait en optimisation)

q0=[0, pi/2, 0, pi/2, 0, pi/2]

q = fsolve(lambda q: solve_eq_NL(q, pos_eff), q0)

trace_rob(q)

q=MGI_analytique(pos_eff)

trace_rob(q)