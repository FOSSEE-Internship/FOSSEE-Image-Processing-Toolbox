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
function [output] = lab2rgb(pstData)
// This function converts CIE 1976 L*a*b* to RGB.
//
// Calling Sequence
// [output] = lab2rgb(pstData)
// 
// Parameters
// pstData : The color values to convert, specified as a list of values.
// output : The converted color values, returned as an array of the same shape as the input. 
//
// Description
// Convert CIE 1976 L*a*b* to RGB.
//
// Examples
// // Convert a color value in L*a*b* color space to the Adobe RGB (1998) color space. 
// a = list(70, 5, 10); 
// b = lab2rgb(a);
// 
// Examples
// // Convert a color value in L*a*b* color space to the Adobe RGB (1998) color space. 
// a = list(71, 50, 10); 
// b = lab2rgb(a);
//
// Examples
// // Convert a color value in L*a*b* color space to the Adobe RGB (1998) color space. 
// a = list(7.3, 5.53, 10); 
// b = lab2rgb(a);
//
// Examples
// // Convert a color value in L*a*b* color space to the Adobe RGB (1998) color space. 
// a = list(70, 5, 10.6656); 
// b = lab2rgb(a);
//
// Examples
// // Convert a color value in L*a*b* color space to the Adobe RGB (1998) color space. 
// a = list(70, 5.45, 10.45); 
// b = lab2rgb(a);
//
// Examples
// // Convert a color value in L*a*b* color space to the Adobe RGB (1998) color space. 
// a = list(7.343, 5.34, 10); 
// b = lab2rgb(a);
//
// Examples
// // Convert a color value in L*a*b* color space to the Adobe RGB (1998) color space. 
// a = list(70, 500, 1012); 
// b = lab2rgb(a);
//
// Examples
// // Convert a color value in L*a*b* color space to the Adobe RGB (1998) color space. 
// a = list(701.2, 5, 10); 
// b = lab2rgb(a);
//
// Examples
// // Convert a color value in L*a*b* color space to the Adobe RGB (1998) color space. 
// a = list(70, 5.545, 1.0); 
// b = lab2rgb(a);
//
// Examples
// // Convert a color value in L*a*b* color space to the Adobe RGB (1998) color space. 
// a = list(23, 51, 18); 
// b = lab2rgb(a);
//
// Authors
// Tess Zacharias
// Ashish Manatosh Barik		
	out = raw_lab2rgb(pstData)
	
	channels = size(out)

	for i = 1:channels
		output(:, :, i) = (out(i))
	end
	
	output = double(output)
	
endfunction
