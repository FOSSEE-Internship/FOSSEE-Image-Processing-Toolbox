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
function [out]= minEnclosingCircle(x,y)
// This function is used to find the circumcircle of an object constituted by the given set of input points.
//
// Calling Sequence
// x = [x1 x2 x3 ...............xn]
// y = [y1 y2 y3................yn]
// out = minEnclosingCirlce(x , y)
//
// Parameters
// x: 1xn matrix denoting the x coordinates of the points of the object
// y: 1xn matrix denoting the corresponding y coordinates of the points of the object
// out: Output structure with the following members- 1) center_x- x coordinate of the center of the circle, 2)center_y- x coordinate of the center of the circle, 3) radius- radius of the circle
//
// Examples
// x = [0 8 8 0 2]
// y = [0 0 5 5 9] 
// out = minEnclosingCircle(x,y)
//
// Examples
//
// x = [0 8 8 0 8 8]
// y = [0 0 5 5 4 4] 
// out = minEnclosingCircle(x,y)
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
		           
           [t1 t2 t3]= raw_minEnclosingCircle(x,y);
           out=struct("center_x",t1,"center_y",t2,"radius",t3);
endfunction;
