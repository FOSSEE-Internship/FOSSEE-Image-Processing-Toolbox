// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Priyanka Hiranandani, Ashish Manatosh Barik     
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [out] = contourArea(inputArrayContour, booloriented)
// This function calculates the contour area. 
//
// Calling Sequence
// [out] = contourArea(inputArrayContour, booloriented)
// 
// Parameters
// inputArrayContour : The input vector of 2D points.
// booloriented : The oriented area flag. If it is true, the function returns a signed area value, depending on the contour orientation (clockwise or counter-clockwise). Using this feature you can determine the orientation of a contour by taking the sign of an area. 
// out : The output is the calculated area.
//
// Description
// It computes the contour area. Also, the function will most certainly give a wrong results for contours with self-intersections.
//
// Examples
// // a simple example
// inputArrayContour = [0 0; 10 0; 10 10; 5 4S];
// booloriented = %t;
// b = contourArea(inputArrayContour, booloriented);
//
// Authors
// Priyanka Hiranandani, NIT Surat
// Ashish Manatosh Barik, NIT Rourkela  
         out = raw_contourArea(inputArrayContour, booloriented);

endfunction
   
