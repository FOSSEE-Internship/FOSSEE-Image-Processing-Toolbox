// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Abhilasha Sancheti & Sukul Bagai
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [out] = fillConvexPoly(img, pstData, npts, r_value, g_value, b_value, linetype, shift)
// This function fills a convex polygon. 
//
// Calling Sequence
// [out] = fillConvexPoly(img, pstData, npts, r_value, g_value, b_value, linetype, shift)
// 
// Parameters
// img : The input source image. 
// pstData : The vector of polygon vertices.
// npts : The number of polygon vertices.
// r_value : The red value of RGB color for the polygon. 
// g_value : The green value of RGB color for the polygon.
// b_value : The blue value of RGB color for the polygon. 
// linetype : This is the type of the polygon boundaries. It has only 3 valid types: 4, 8 and 16(CV_AA). Passing any other value as lineType is not legal.
// shift : This is the number of fractional bits in the vertex coordinates. 
//
// Description
// The function fillConvexPoly draws a filled convex polygon. It can fill not only convex polygons but any monotonic polygon without self-intersections, that is, a polygon whose contour intersects every horizontal line (scan line) twice at the most (though, its top-most and/or the bottom edge could be horizontal).
//
// Examples
// // a simple example
// a = imread("/images/lena.jpeg");
// b = [ 0 10; 10 0; -10 0 ]; 
// c = fillConvexPoly(a, b, 3, 1, 1, 1, 8, 0);
//
// Authors
// Abhilasha Sancheti
// Sukul Bagai        		
	image = mattolist(img);

	a = raw_fillConvexPoly(image, pstData, npts, r_value, g_value, b_value, linetype, shift)

	d = size(a);

	for i=1:d
		out(:,:,i) = a(i);
	end

endfunction
