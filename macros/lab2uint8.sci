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
function [output] = lab2uint8(pstData)
// This function converts L*a*b* data to uint8.
//
// Calling Sequence
// [output] = lab2uint8(pstData)
// 
// Parameters
// pstData : It is a list of color values.
// output : The converted uint8 value. lab8 has the same size as lab.
//
// Description
// Converts L*a*b* data to uint8.
//
// Examples
// // to convert L*a*b* color values from double to uint8.
// a = list(70, 5, 10); 
// b = lab2rgb(a); 
// 
// Examples
// // to convert L*a*b* color values from double to uint8.
// a = list(71, 5, 10); 
// b = lab2rgb(a); 
//
// Examples
// // to convert L*a*b* color values from double to uint8.
// a = list(0, 5, 10); 
// b = lab2rgb(a); 
//
// Examples
// // to convert L*a*b* color values from double to uint8.
// a = list(89, 50, 10); 
// b = lab2rgb(a); 
//
// Examples
// // to convert L*a*b* color values from double to uint8.
// a = list(70, 5, 10.78); 
// b = lab2rgb(a); 
//
// Examples
// // to convert L*a*b* color values from double to uint8.
// a = list(7, 5, 89); 
// b = lab2rgb(a); 
//
// Examples
// // to convert L*a*b* color values from double to uint8.
// a = list(70.344, 5.34, 10); 
// b = lab2rgb(a); 
//
// Examples
// // to convert L*a*b* color values from double to uint8.
// a = list(0, 0, 10); 
// b = lab2rgb(a); 
//
// Examples
// // to convert L*a*b* color values from double to uint8.
// a = list(70.89, 5.11, 10.33); 
// b = lab2rgb(a); 
//
// Examples
// // to convert L*a*b* color values from double to uint8.
// a = list(10, 5, 10); 
// b = lab2rgb(a); 
//
// Authors
// Tess Zacharias	
	
	out = raw_lab2uint8(pstData);

	channels = size(out)

	for i = 1:channels
		output(:, :, i) = (out(i))
	end
	
endfunction
