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
//
function [output] = ntsc2rgb(pstData)
// This function converts NTSC values to RGB color space.
//
// Calling Sequence
// [output] = ntsc2rgb(pstData)
// 
// Parameters
// pstData : It is a list of the NTSC luminance (Y) and chrominance (I and Q) color components.
// output :  It is a list that contains the red, green, and blue values equivalent to those colors.
//
// Description
// Converts NTSC values to RGB color space.
//
// Examples
// // Convert the grayscale image back to RGB color space.
// a = imread("/images/b1.jpeg",0)
// b = ntsc2rgb(a);
// 
// Examples
// // Convert the grayscale image back to RGB color space.
// a = imread("/images/b2.jpeg",0)
// b = ntsc2rgb(a);
//
// Examples
// // Convert the grayscale image back to RGB color space.
// a = imread("/images/graf1.jpeg",0)
// b = ntsc2rgb(a);
//
// Examples
// // Convert the grayscale image back to RGB color space.
// a = imread("/images/graf2.jpeg",0)
// b = ntsc2rgb(a);
//
// Examples
// // input RGB image
// a = imread("/images/b2.jpeg")
// b = ntsc2rgb(a);
//
// Examples
// // input RGB image
// a = imread("/images/graf1.jpeg")
// b = ntsc2rgb(a);
//
// Examples
// // input RGB image
// a = imread("/images/garf2.jpeg")
// b = ntsc2rgb(a);
//
// Examples
// // Convert the grayscale image back to RGB color space.
// a = imread("/images/lena.jpeg",0)
// b = ntsc2rgb(a);
//
// Authors
// Tess Zacharias

	out = raw_ntsc2rgb(pstData)

	channels = size(out)

	for i= 1:channels
		output(:, :, i) = out(i)
	end

endfunction
