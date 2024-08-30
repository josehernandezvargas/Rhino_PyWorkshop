#! python3
"""Adaptive layer height

Inputs:
    brep
    samples
    layer_height
"""

__author__ = "joseh"
__version__ = "2024.08.14"

import rhinoscriptsyntax as rs
import math
from ghpythonlib import treehelpers as th

if brep == None:
    msg = "Connect a brep"
    ghenv.Component.AddRuntimeMessage(
        gh.Kernel.GH_RuntimeMessageLevel.Warning, msg)


def interpolate(input_data, output_data, x):
    # Ensure input is within the boundaries
    # Check if input is below the minimum boundary
    if x < input_data[0]:
        print(f"Warning: Input value {x} is below the minimum boundary. Returning the first output value.")
        return output_data[0]
    
    # Check if input is above the maximum boundary
    if x > input_data[-1]:
        print(f"Warning: Input value {x} is above the maximum boundary. Returning the last output value.")
        return output_data[-1]


    # Find the two nearest points
    for i in range(len(input_data) - 1):
        if input_data[i] <= x <= input_data[i + 1]:
            x0, y0 = input_data[i], output_data[i]
            x1, y1 = input_data[i + 1], output_data[i + 1]
            break
    else:
        raise ValueError("Input value is not within the interpolation range.")

    # Perform linear interpolation
    y = y0 + (y1 - y0) * ((x - x0) / (x1 - x0))
    return y


points = []
angles = []
target_heights = []
slice_planes = []

bbox = rs.BoundingBox(brep)
minx = bbox[0][0]
miny = bbox[0][1]
minz = bbox[0][2]
maxx = bbox[6][0]
maxy = bbox[6][1]
maxz = bbox[6][2]

brep_height = maxz - minz
contour_distance = brep_height / samples # height for contour crv
contour_heights = [contour_distance * i for i in range(int(samples))]

# first contour operation to measure overhangs
curves = rs.AddSrfContourCrvs(brep, (bbox[0], bbox[4]), contour_distance)

# Simply sampling the first point of each curve
# for i, crv in enumerate(crvs):
#     if i < len(crvs)-1:
#         startpt = rs.CurveStartPoint(crvs[i])
#         centroid = rs.CurveAreaCentroid(crv)[0]
#         nextstartpt = rs.CurveStartPoint(crvs[i+1])
#         vector = rs.VectorCreate(nextstartpt, startpt)
#         zvect = rs.WorldXYPlane()[3] # world XY plane is OXYZ
#         angle = rs.VectorAngle(zvect, vector)
#         delta = math.sin(math.radians(angle)) # 0 to 90 deg > 0 to 1
#         target_layer_height = max(10 - delta * 10, 3) # 0 to 90 deg > 10 to 0 layer height
#         points.append(startpt)
#         angles.append(angle)
#         target_heights.append(target_layer_height)
#         # deltas.append(delta)
#         # offsets.append(offset)
#         # vectors.append(vector)
#         # offset_crvs.append(offset_crv)
#         # widths.append(print_width + delta)
#         # # widths.append(max(print_width, delta * 2))
#         # deltas.append(delta)

for i, crv in enumerate(curves):
    if i < len(curves)-1:
        pts = rs.DivideCurve(crv, crv_samples)
        crv_angles = []
        # pts_next = cl.divide_crv_equal(crv[i+1], div_length)
        for j, pt in enumerate(pts):
            closest_param = rs.CurveClosestPoint(curves[i+1], pt)
            closest_pt = rs.EvaluateCurve(curves[i+1], closest_param)
            vector = -rs.VectorCreate(pt, closest_pt)
            zvect = rs.WorldXYPlane()[3] # world XY plane is OXYZ
            angle = abs(rs.VectorAngle(zvect, vector))
            crv_angles.append(angle)
        angle = max(crv_angles)
        # startpt = rs.CurveStartPoint(crvs[i])
        # centroid = rs.CurveAreaCentroid(crv)[0]
        # nextstartpt = rs.CurveStartPoint(crvs[i+1])
        # vector = rs.VectorCreate(nextstartpt, startpt)
        # zvect = rs.WorldXYPlane()[3] # world XY plane is OXYZ
        # angle = rs.VectorAngle(zvect, vector)
        delta = math.sin(math.radians(angle)) # 0 to 90 deg > 0 to 1
        target_layer_height = max(10 - delta * 10, 5) # 0 to 90 deg > 10 to 0 layer height
        target_heights.append(target_layer_height)
        # deltas.append(delta)
        # offsets.append(offset)
        # vectors.append(vector)
        # offset_crvs.append(offset_crv)
        # widths.append(print_width + delta)
        # # widths.append(max(print_width, delta * 2))
        # deltas.append(delta)

print(contour_heights)
print(target_heights)

# for i, t in enumerate(target_heights):
#     print(contour_heights[i], target_heights[i])

# Define a safety limit for the maximum number of iterations
max_iterations = 1000  # Set an appropriate limit based on your use case
iteration_count = 0
slice_heights = [0]
slice_deltas = []

# # Interpolate value at x = 1000.0
# x = 200.0
# result = interpolate(input_data, output_data, x)
# print(f"Interpolated value at {x} is {result}")



while slice_heights[-1] < brep_height:

    current_height = slice_heights[-1]
    
    # Interpolate to find the appropriate layer height at the current height
    layer_height = interpolate(contour_heights, target_heights, current_height)
    # print(f"Layer height at {current_height} mm: {layer_height} mm")

    next_height = current_height + layer_height
    
    if next_height > brep_height:
        next_height = brep_height  # Ensure we don't exceed the Brep height
    
    slice_heights.append(next_height)
    slice_deltas.append(layer_height)
    
    # Increment the iteration counter
    iteration_count += 1
    
# Check if the iteration count exceeds the safety limit
    if iteration_count >= max_iterations:
        print("Warning: Exceeded maximum iterations.")
        break

# slice again according to the adaptive layer heights

# slicing_planes = [rs.PlaneFromFrame((0,0,h), (1,0,0), (0,1,0)) for h in slice_heights]
# adaptive_contours = [rs.AddSrfContourCrvs(brep, slicing_planes[i]) for i in range(len(slicing_planes)-1)]
# a = th.list_to_tree(adaptive_contours)

a = slice_heights
b = slice_deltas