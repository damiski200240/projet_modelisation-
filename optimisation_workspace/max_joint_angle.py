import numpy as np
from shapely.geometry import Point, Polygon
from get_compliant_workspace import get_compliant_workspace

def isinterior(polygon: Polygon, x_coords, y_coords):
    """Check if each (x, y) is inside the polygon."""
    return np.array([polygon.contains(Point(x, y)) for x, y in zip(x_coords, y_coords)])

def workspace_optimization(param, loci, threshold, reach):
    """
    Compute the maximum joint angle before entering a singularity.
    
    Parameters:
    - param: [Rb, L1, L2, Re]
    - loci: 2D array of Jacobian determinant or condition
    - threshold: value under which a point is considered singular
    - reach: workspace boundary limit
    
    Returns:
    - max_angle (degrees)
    """
    Rb, L1, L2, Re = param
    rows, cols = loci.shape
    
    # Find indices of points below threshold (i.e., singularities)
    singular_rows, singular_cols = np.where(loci < threshold)

    # Convert indices to physical coordinates (assuming workspace from -reach to reach)
    pixel2x = (reach - (-reach)) / cols
    pixel2y = (reach - (-reach)) / rows
    x_coords = -reach + singular_cols * pixel2x
    y_coords = -reach + singular_rows * pixel2y

    # Test angles from 1° to 180°
    joint_limit = np.linspace(1, 180, 180)
    max_angle = 0

    for angle_deg in joint_limit:
        angle_rad = angle_deg * np.pi / 180
        
        # Get compliant workspace for the current angle
        comp_workspace = get_compliant_workspace(param, angle_rad, [0, 0, 0], '+ + +', 0)

        # Check if any singularity falls inside the workspace
        test = isinterior(comp_workspace, x_coords, y_coords)

        # If no singularity is inside, update max_angle
        if not np.any(test):
            max_angle = angle_deg
        else:
            break

    return max_angle
    
                
# def workspace_optimization(param, loci_serial, loci_parallel, threshold_serial, threshold_parallel, reach):
#     import numpy as np
#     from shapely.geometry import Point
#     from get_compliant_workspace import get_compliant_workspace
#     from shapely.geometry import Polygon

#     rows, cols = loci_serial.shape
#     px2 = (2 * reach) / cols
#     py2 = (2 * reach) / rows
#     x_coords = -reach + np.arange(cols) * px2
#     y_coords = -reach + np.arange(rows) * py2

#     xx, yy = np.meshgrid(x_coords, y_coords)
#     xx_flat = xx.flatten()
#     yy_flat = yy.flatten()

#     mask_serial = loci_serial.flatten() < threshold_serial
#     mask_parallel = loci_parallel.flatten() < threshold_parallel

#     singular_x = xx_flat[mask_serial | mask_parallel]
#     singular_y = yy_flat[mask_serial | mask_parallel]

#     joint_limit = np.linspace(1, 180, 180)
#     max_angle = 0

#     for angle_deg in joint_limit:
#         angle_rad = angle_deg * np.pi / 180
#         try:
#             comp_workspace = get_compliant_workspace(param, angle_rad, [0, 0, 0], '+ + +', 0)
#         except:
#             break

#         if any(comp_workspace.contains(Point(x, y)) for x, y in zip(singular_x, singular_y)):
#             break
#         else:
#             max_angle = angle_deg

#     return max_angle