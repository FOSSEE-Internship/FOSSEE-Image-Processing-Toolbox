// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Priyanka Hiranandani, Rohan Gurve 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [out]=minAreaRect(x,y)
// This function is used to find a rotated rectangle of the minimum area enclosing the input 2D point set.
//
// Calling Sequence
// x = [x1 x2 x3 ...............xn]
// y = [y1 y2 y3................yn]
// out = minAreaRect(x , y)
//
// Parameters
// x: 1xn matrix denoting the x coordinates of the points
// y: 1xn matrix denoting the corresponding y coordinates of the points
// out: Output structure with the following members - 1) width- width of the rectangle, 2) height- height of the rectangle, 3) center_x- x coordinate of the center of the rectangle, 4) center_y- x coordinate of the center of the rectangle, 5) angle- the angle by which the rectanle is rotated, 6) x_coordinates_vertices- 1x4 Double matrix denoting the x coordinates of the vertices of the rectangle, 7) x_coordinates_vertices- 1x4 Double matrix denoting the x coordinates of the vertices of the rectangle, 8) y_coordinates_vertices- 1x4 Double matrix denoting the corresponding y coordinates of the vertices of the rectangle
//
// Description
// The function calculates and returns the minimum-area bounding rectangle (possibly rotated) for a specified point set. 
// the returned rotatedRect can contain negative indices when data is close to the containing Mat element boundary.
//
// Examples
// x = [0 8 8 0]
// y = [0 0 5 5] 
// out = minAreaRect(x,y)
//
// Examples
// x = [0 8 8 0 8 8]
// y = [0 0 5 5 4 4] 
// out = minAreaRect(x,y)
//
// Authors
// Priyanka Hiranandani 
// Rohan Gurve

        [ lhs rhs ] = argn(0)
		if lhs > 1 then
			error(msprintf("Too many output argument"))
		end
		
		if rhs > 2 then
			error(msprintf("Too many input arguments"))
		elseif rhs < 2 then
		    error(msprintf("input arguments missing"))
		end	
		
                    
    [t1 t2 t3 t4 t5 t6 t7]= raw_minAreaRect(x,y);
    out=struct("width",t1,"height",t2,"center_x",t3,"center_y",t4,"angle",t5,"x_coordinates_vertices",t6,"y_coordinates_vertices",t7);
endfunction;


