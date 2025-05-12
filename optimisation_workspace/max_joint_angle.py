# this fonction is used to give the maximum angle before getting a singularity 
# what it does is it takes the parameteres 
# takes a workspace as a mesh grid 
# calculate for every point the jacobienne 
# if the motor is close to a singularity we stop 
# and the motor would be allowed to work only in this 
import numpy as np
from shapely.geometry import Point, Polygon
from get_compliant_workspace import get_compliant_workspace
def isinterior(polygon_points, x_coords, y_coords):
    """Check if each (x, y) is inside the polygon."""
    poly = Polygon(polygon_points)
    return np.array([poly.contains(Point(x, y)) for x, y in zip(x_coords, y_coords)])

def workspace_optimization(param, loci, threshold):
    """
    Compute the maximum joint angle before entering a singularity.
    
    Parameters:
    - param: [Rb, L1, L2, Re]
    - loci: 2D array of Jacobian determinant or condition
    - threshold: value under which a point is considered singular
    
    Returns:
    - max_angle (degrees)
    """
    Rb, L1, L2, Re = param
    rows, cols = loci.shape
    
    # Find indices of points below threshold (i.e. singularities)
    singular_rows, singular_cols = np.where(loci < threshold)

    # Convert indices to physical coordinates (assuming workspace from -0.21 to 0.21)
    pixel2x = (210 - (-210)) / cols
    pixel2y = (210 - (-210)) / rows
    x_coords = -210 + singular_cols * pixel2x
    y_coords = -210 + singular_rows * pixel2y

    # Test angles from 1° to 180°
    joint_limit = np.linspace(1, 180, 180)
    max_angle = 0

    for angle_deg in joint_limit:
        angle_rad = angle_deg * np.pi / 180
        
        # You must define this yourself or mock it
        comp_workspace = get_compliant_workspace(param, angle_rad, [0, 0, 0], '+ + +', 0)

        # Check if any singularity falls inside the workspace
        mask = isinterior(comp_workspace, x_coords, y_coords)
        if mask.sum() > 10:  # autoriser jusqu'à 10 points "dangereux"
            break
        else:
            max_angle = angle_deg

    return max_angle
    
                
                