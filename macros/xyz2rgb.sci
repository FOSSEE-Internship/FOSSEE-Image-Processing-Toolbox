// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Tess Zacharias, Ashish Manatosh Barik 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [output] = xyz2rgb(data)
// This function converts CIE 1931 XYZ to RGB.
//
// Calling Sequence
// [output] = xyz2rgb(data)
// 
// Parameters
// data : list of color values to convert.
// output : list of converted color values.
//
// Description
// Converts CIE 1931 XYZ to RGB.
//
// Examples
// // Convert a color value in the XYZ color space to the sRGB color space.
// a = list(0.25, 0.40, 0.10);
// xyz2rgb(a)
// 
// Examples
// // Convert a color value in the XYZ color space to the sRGB color space.
// a = list(3.25, 5.40, 12.10);
// xyz2rgb(a)
//
// Examples
// // Convert a color value in the XYZ color space to the sRGB color space.
// a = list(2, 5, 4);
// xyz2rgb(a)
//
// Examples
// // Convert a color value in the XYZ color space to the sRGB color space.
// a = list(0.65, 0.43, 0.19);
// xyz2rgb(a)
//
// Examples
// // Convert a color value in the XYZ color space to the sRGB color space.
// a = list(89.25, 23, 0.6710);
// xyz2rgb(a)
//
// Examples
// // Convert a color value in the XYZ color space to the sRGB color space.
// a = list(0.2534, 0.4340, 0.143);
// xyz2rgb(a)
//
// Examples
// // Convert a color value in the XYZ color space to the sRGB color space.
// a = list(67.25, 34.40, 44.10);
// xyz2rgb(a)
//
// Examples
// // Convert a color value in the XYZ color space to the sRGB color space.
// a = list(34.25, 56.40, 223.189);
// xyz2rgb(a)
//
// Examples
// // Convert a color value in the XYZ color space to the sRGB color space.
// a = list(0.1, 0.1, 0.1);
// xyz2rgb(a)
//
// Examples
// // Convert a color value in the XYZ color space to the sRGB color space.
// a = list(78.25, 34.40, 23.10);
// xyz2rgb(a)
//
// Authors
// Tess Zacharias
// Ashish Manatosh Barik	

	output = raw_xyz2rgb(data);

endfunction
