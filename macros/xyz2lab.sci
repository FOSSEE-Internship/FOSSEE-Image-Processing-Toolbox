// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Ashish Manatosh Barik, Priyanka Hiranandani
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [output] = xyz2lab(vartype)
// This function converts CIE 1931 XYZ to CIE 1976 L*a*b*.
//
// Calling Sequence
// [output] = xyz2lab(vartype)
// 
// Parameters
// varType : list of color values to convert.
// output : list of converted color values.
//
// Description
// Convert CIE 1931 XYZ to CIE 1976 L*a*b*.
//
// Examples
// // Convert an XYZ color value to L*a*b*
// a = list(0.25, 0.40, 0.10)
// xyz2lab(a)
// 
// Examples
// // Convert an XYZ color value to L*a*b*
// a = list(0.29, 0.23, 0.11)
// xyz2lab(a))
//
// Examples
// // Convert an XYZ color value to L*a*b*
// a = list(0.29, 34, 0.10)
// xyz2lab(a)
//
// Examples
// // Convert an XYZ color value to L*a*b*
// a = list(0.25, 0.56, 0.18)
// xyz2lab(a)
//
// Examples
// // error - inpput ahould be M by 3 or M by N by 3
// a = uint16([100 32 67 56]);
// b = xyz2double(a);
//
// Examples
// // Convert an XYZ color value to L*a*b*
// a = list(89.25, 89.40, 0.10)
// xyz2lab(a)
//
// Examples
// // Convert an XYZ color value to L*a*b*
// a = list(78, 89, 11)
// xyz2lab(a)
//
// Examples
// // Convert an XYZ color value to L*a*b*
// a = list(0.25, 0.40, 90.67)
// xyz2lab(a)
//
// Examples
// // Convert an XYZ color value to L*a*b*
// a = list(0.76, 0.67, 9.10)
// xyz2lab(a)
//
// Examples
// // Convert an XYZ color value to L*a*b*
// a = list(78.25, 34.40, 0.10)
// xyz2lab(a))
//
// Authors
// Ashish Manatosh Barik, NIT Rourkela	
// Priyanka Hiranandani, NIT Surat 	
	output = raw_xyz2lab(vartype)
		
endfunction

