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

function outputImg= line(inputImage , x1 , y1 , x2,y2,r_value,g_value,b_value,thickness,linetype,shift)
// This function draws a line segment connecting two points. 
//
// Calling Sequence
// B = line(A,x1 , y1 , x2,y2,r_value,g_value,b_value,thickness,linetype,shift)
// 
// Parameters
// A: image matrix of the source image.
// x1: x coordinate of the first point of the line segment.
// y1: y coordinate of the first point of the line segment.
// x2: x coordinate of the second point of the line segment.
// y2: y coordinate of the second point of the line segment.
// r_value: r value of the color of the line.
// g_value: g value of the color of the line.
// b_value: b value of the color of the line.
// thickness: Line thickness.
// linetype : 8 (or omitted) - 8-connected line.4 - 4-connected line.CV_AA - antialiased line
// shift – Number of fractional bits in the point coordinates.
// B : output image with it's histogram matching similar to a given reference image. 
//
// Description
// The function line draws the line segment between pt1 and pt2 points in the image. The line is clipped by the image boundaries. For non-antialiased lines with integer coordinates, the 8-connected or 4-connected Bresenham algorithm is used. Thick lines are drawn with rounding endings. Antialiased lines are drawn using Gaussian filtering. To specify the line color, you may use the macro CV_RGB(r, g, b).
//
// Examples
// i = imread('lena.jpeg',0);
// i1 = line(i,0,0,34,45,0,255,0,1,8,0);
	 
	 inputList=mattolist(inputImage);
         
         outputList=raw_line(inputList , x1 , y1 , x2,y2,r_value,g_value,b_value,thickness,linetype,shift)
       for i=1:size(outputList)
           outputImg(:,:,i)=outputList(i)
       end
endfunction
