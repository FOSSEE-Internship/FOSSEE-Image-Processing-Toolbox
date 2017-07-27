// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Sridhar Reddy, Ashish Manatosh Barik 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [output] = rgb2lab(inputImage)
// This function converts RGB to CIE 1976 L*a*b*.
//
// Calling Sequence
// [output] = rgb2lab(inputImage)
// 
// Parameters
// inputImage : It is a list of color values to convert.
// output : The converted color values, returned as a list.
//
// Description
// Converts RGB to CIE 1976 L*a*b*.
//
// Examples
// // to convert the RGB white value to L*a*b.
// rgb2lab([1 1 1])
// 
// Examples
// // to convert the RGB white value to L*a*b.
// rgb2lab([.2 .3 .4])
//
// Examples
// // Read RGB image to convert
// a = imread("../images/b1.jpeg");
// b = rgb2lab(a);
//
// Examples
// // Read RGB image to convert
// a = imread("../images/b2.jpeg");
// b = rgb2lab(a);
//
// Examples
// // to convert the RGB white value to L*a*b.
// rgb2lab([23 23 22])
//
// Examples
// // Read RGB image to convert
// a = imread("../images/lena.jpeg");
// b = rgb2lab(a);
//
// Examples
// // to convert the RGB white value to L*a*b.
// rgb2lab([34.2 43.3 343.4])
//
// Examples
// // Read RGB image to convert
// a = imread("../images/graf1.jpeg");
// b = rgb2lab(a);
//
// Examples
// // Read RGB image to convert
// a = imread("../images/graf2.jpeg");
// b = rgb2lab(a);
//
// Examples
// // Read RGB image to convert which doesnt exit
// a = imread("../images/b.jpeg");
// b = rgb2lab(a);
//
// Authors
// Sridhar Reddy
// Ashish Manatosh Barik	
	inputList = mattolist(inputImage);

    	output = raw_rgb2lab(inputList);


endfunction

