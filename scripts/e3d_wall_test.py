#! python3

"""
Short summary of what the function does.

Args:
	base_rect (curve): base rectangle curve
    height (float): height of the wall
    truss_distance (float): target distance between truss nodes (divided evenly along the wall)
	overlap (float): overlap for double lines (in percentage of nozzle size)
    vent_width (float): width of ventilated layer
	truss_contact (float): length of contact segments for truss nodes

Returns:
	ReturnType: What the function returns and in what format.
"""

__author__ = "joseh"
__version__ = "2025.10.31"


import Rhino
import Grasshopper as gh
import rhinoscriptsyntax as rs
from ghpythonlib import treehelpers as th
from itertools import chain

# own libraries
import curvelib as cl
import geometrylib as gl

import math

nozzle_size = 20 # mm
layer_height = 10 # mm
line_distance = nozzle_size * (1 - overlap/100)  # adjusted line distance with overlap

def create_truss_wall(base_rect, shift, truss_distance, overlap, vent_width, truss_contact):
	# rectangle vertices
	A, B, C, D = rs.CurvePoints(base_rect)[:4]

	x_vector = rs.VectorUnitize(rs.VectorCreate(B, A))
	y_vector = rs.VectorUnitize(rs.VectorCreate(D, A))

	A1 = A
	B1 = B
	C1 = C
	D1 = D

	# Shift side AB
	A = rs.CopyObject(A, rs.VectorScale(y_vector, line_distance))
	B = rs.CopyObject(B, rs.VectorScale(y_vector, line_distance))

	# shift ventilated layer
	C = rs.CopyObject(C, rs.VectorScale(y_vector, -vent_width))
	D = rs.CopyObject(D, rs.VectorScale(y_vector, -vent_width))

	# rectangle sides
	AB = rs.AddLine(A, B)
	BC = rs.AddLine(B, C)
	DC = rs.AddLine(D, C) # note reverse order
	DA = rs.AddLine(D, A)

	# shift sides AB and DC for variable truss pattern
	shift_vector = rs.VectorCreate(D, A)*(shift)
	AB = rs.MoveObject(AB, shift_vector)
	DC = rs.MoveObject(DC, -shift_vector)

	#divide long sides
	AB_truss_div = cl.divide_crv_equal(AB, truss_distance)
	DC_truss_div = cl.divide_crv_equal(DC, truss_distance)
	# DC_truss_div = 

	# truss_base
	truss = list(chain.from_iterable(zip(AB_truss_div[::2], DC_truss_div[1::2])))

	new_truss = []
		
	# add contact segments
	for i, pt in enumerate(truss):
		if i == 0 or i == len(truss) -1:
			new_truss.append(pt)
			continue	

		pt_before = rs.CopyObject(pt, rs.VectorScale(x_vector, -truss_contact/2))
		pt_after = rs.CopyObject(pt, rs.VectorScale(x_vector, truss_contact/2))
		new_truss.append(pt_before)
		new_truss.append(pt_after)


	new_truss.insert(0, B1)
	new_truss.insert(0, A1)
	points = [A, B, C, D]
	new_truss.append(D)
	new_truss.append(D1)
	new_truss.append(C1)
	new_truss.append(C)

	return new_truss

# shift  = 1

# a = create_truss_wall(base_rect, shift, truss_distance, overlap, vent_width, truss_contact)

z_vector = (0,0,layer_height)

output = []
for i in range(50):
	shift = (i / 50)  # shift from 0 to 1 
	input_rect = rs.CopyObject(base_rect, rs.VectorScale(z_vector, i))
	layer = create_truss_wall(input_rect, shift, truss_distance, overlap, vent_width, truss_contact)
	# polyline = rs.AddPolyline(layer)
	output.append(layer)

a = th.list_to_tree(output)