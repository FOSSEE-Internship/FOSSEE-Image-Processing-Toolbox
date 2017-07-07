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
function [output] = xyz2uint16(pstData)
// This function converts XYZ color values to uint16.
//
// Calling Sequence
// [output] = xyz2uint16(pstData)
// 
// Parameters
// pstData : list of uint16 or double array that must be real and nonsparse
// output : list of puint8.
//
// Description
// Converts an M-by-3 or M-by-N-by-3 array of XYZ color values to uint16. output has the same size as pstData.
//
// Examples
// // Create a double vector specifying a color in XYZ colorspace.
// a = list(0.1, 0.5, 1.0)
// xyz2uint16(a)
// 
// Examples
// // Create a double vector specifying a color in XYZ colorspace.
// a = list(0.14, 0.35, 1.20)
// xyz2uint16(a)
//
// Examples
// // Create a double vector specifying a color in XYZ colorspace.
// a = list(45.1, 22.5, 45.0)
// xyz2uint16(a)
//
// Examples
// // Create a double vector specifying a color in XYZ colorspace.
// a = list(200, 334, 2112)
// xyz2uint16(a)
//
// Examples
// // Create a double vector specifying a color in XYZ colorspace.
// a = list(56.1, 0.5, 1.0)
// xyz2uint16(a)
//
// Examples
// // Create a double vector specifying a color in XYZ colorspace.
// a = list(0.1, 8378.5, 1.0)
// xyz2uint16(a)
//
// Examples
// // Create a double vector specifying a color in XYZ colorspace.
// a = list(878.1, 32.5, 1.0)
// xyz2uint16(a)
//
// Examples
// // Create a double vector specifying a color in XYZ colorspace.
// a = list(0.12323, 0.53434, 1.878)
// xyz2uint16(a)
//
// Examples
// // Create a double vector specifying a color in XYZ colorspace.
// a = list(44, 55, 1.0)
// xyz2uint16(a)
//
// Examples
// // Create a double vector specifying a color in XYZ colorspace.
// a = list(0.134, 55.5, 1.121)
// xyz2uint16(a)
//
// Authors
// Tess Zacharias
// Ashish Manatosh Barik 
	output = raw_xyz2uint16(pstData)

endfunction
