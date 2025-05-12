# this file is for computing the best params to get the maximum workspace area
# we will pose two constraints : The robot shouldn't have a bigger reach then 15 cm and the arm length shouldn't be bigger than the radius of our base
# and for simplicity we consider that the first and second leg are similair in length  
# we will use differential_evolution to solve our optimisation problem 


from scipy.optimize import differential_evolution
from get_compliant_workspace import get_compliant_workspace
from numpy import pi
import  time
from shapely.geometry import Point

def get_compliant_workspace_polygon(param) : 
    mode = '+ + +'
    home_pos = [0, 0, 0]
    orientation = 0 * pi / 180
    limit = 300* pi / 180
    workspace = get_compliant_workspace(param, limit, home_pos, mode, orientation, plot=False)
    return workspace

def is_inside_base_circle(workspace, Rb):
    center = Point(0, 0)
    base_circle = center.buffer(Rb)
    return workspace.within(base_circle)

def objective_function(param):
    Rb = 150
    L1, Re = param
    L2 = L1
    param = [Rb, L1, L2, Re]

    workspace = get_compliant_workspace_polygon(param)
    if workspace is None or workspace.is_empty:
        return 1e6  # invalid configuration

    # Reject if any part of workspace escapes the base circle
    if not is_inside_base_circle(workspace, Rb) or Re > 0.75 * Rb or Re > 0.8 * L1 :
        return 1e6  # penalty

    return -workspace.area  # maximize area

# Bounds for each variable
bounds = [
    (50, 180),  # L1
    (10, 100)   # Re
]
def main():
    # Run optimization
    tm1 = time.time()
    result = differential_evolution(objective_function, bounds, maxiter=20, updating="deferred", workers=-1)
    tm2 = time.time()
    print(f"result is {result.x}")
    print("Max workspace area:", -result.fun, "mm²")
    print(f"it took {tm2-tm1}s")
    
if __name__ == "__main__":
    main()