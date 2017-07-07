// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shreyash Sharma
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [out] = ellipse2poly(centre_x,centre_y,width,height,angle,arcstart,arcend,delta)
// This function is used to Approximates an elliptic arc with a polyline. 
//
// Calling Sequence
// B = ellipse2poly(centre_x,centre_y,width,height,angle,arcstart,arcend,delta)
// 
// Parameters 
// center_x:  x coordinate of the center of the arc.
// center_y:  y coordinate of the center of the arc.
// angle:  Rotation angle of the ellipse in degrees. See the ellipse() for details.
// arcStart: Starting angle of the elliptic arc in degrees.
// arcEnd:  Ending angle of the elliptic arc in degrees.
// delta: Angle between the subsequent polyline vertices. It defines the approximation accuracy.
// B: output matrix conatining the vertices of the polyline that aprroximates the elliptic arc. 
//
// Description
// The function ellipse2Poly computes the vertices of a polyline that approximates the specified elliptic arc. It is used by ellipse().
//
// Examples
// ii7 = ellipse2poly(24,34,5,6,34,50,100,5);
//
	out = raw_ellipse2poly(centre_x,centre_y,width,height,angle,arcstart,arcend,delta);
endfunction






