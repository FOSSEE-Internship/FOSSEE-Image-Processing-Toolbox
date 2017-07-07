// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Tess Zacharias
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [points, varargout] = imfindcircles(image, Rmin, Rmax)
// Finds circles in an image.
//
// Calling Sequence
// [points] = imcontrast(srcImg, Rmin, Rmax)
// [points radii] = imcontrast(srcImg, Rmin, Rmax)
//
// Parameters
// srcImage: Input image.
// Rmin: The minimum value of the radius of the circles to find.
// Rmax: The maximum value of the radius of the circle to find.
// points: The returned coordinates of the center of all the circles found.
// radii: The returned values of the radiis of all the circles found.
// 
// Description
// This function can be used to find the circles of a definite range of radii
// in an image.
//
// Examples
// image = imread("blob.jpg");
// [points radii] = imfindcircles(image, 4, 5);
// nimage = drawKeypoints(image, points);
//
// See also
// imread 
//
// Authors
// Tess Zacharias

	[lhs, rhs] = argn(0)
	
	image_list = mattolist(image)
	
	select lhs
		case 1 then
			out_centres = raw_imfindcircles(image_list, Rmin, Rmax)
		
		case 2 then
			[out_centres radii] = raw_imfindcircles(image_list, Rmin, Rmax)
			
			varargout(1) = radii
	end
	
	total_points = size(out_centres(1),'c')
	
	for i=1:total_points
		points(i, 1) = out_centres(1)(1, i)
		points(i, 2) = out_centres(2)(1, i)	
	end

endfunction
