// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Ashish Manatosh Barik 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [output] = xyz2double(pstData)
// This function converts XYZ color values to double.
//
// Calling Sequence
// [output] = xyz2double(pstData)
// 
// Parameters
// pstData : list of uint16 or double array that must be real and nonsparse.
// output : list of converted values.
//
// Description
// Converts an M-by-3 or M-by-N-by-3 array of pstData color values to double. output has the same size as XYZ.
//
// Examples
// // check for boundary level values
// a = uint16([100 32768 65535]);
// b = xyz2double(c);
// 
// Examples
// // check for boundary level values
// a = uint16([100 32768 65536]);
// b = xyz2double(a);
//
// Examples
// // check for lower values
// a = uint16([1 3 5]);
// b = xyz2double(a);
//
// Examples
// // error - inpput should be M by 3 or M by N by 3
// a = uint16([100 32768]);
// b = xyz2double(a);
//
// Examples
// // error - inpput should be M by 3 or M by N by 3
// a = uint16([100 32 67 56]);
// b = xyz2double(a);
//
// Examples
// // float value input
// a = uint16([0.0031 1 2]);
// b = xyz2double(a);
//
// Examples
// // error - inpput should be M by 3 or M by N by 3
// a = uint16([100 32 678]);
// b = xyz2double(a);
//
// Examples
// // error - inpput should be M by 3 or M by N by 3
// a = uint16([100 32768 3244]);
// b = xyz2double(a);
//
// Examples
// // float value input
// a = uint16([0.0031 1.56 2.454]);
// b = xyz2double(a);
//
// Examples
// // error - inpput is double, no conversion takes place.
// a = double([9 1 2]);
// b = xyz2double(a);
//
// Authors
// Ashish Manatosh Barik, NIT Rourkela	
	pstList = mattolist(pstData)	

	out = raw_xyz2double(pstList);

	channels = size(out)

	for i = 1:channels
		output(:, :, i) = (out(i))
	end

	output = double(output)

endfunction
