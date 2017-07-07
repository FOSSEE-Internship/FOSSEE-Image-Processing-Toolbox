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

function [output] = lab2uint16(pstData)
// This function is used to Convert L*a*b* data to uint16. 
//
// Calling Sequence
// B = lab2uint16(pstData) 
//
// Parameters
// psdata: List containing the double matrices for the lab values of the source image.
// B : Output List containing the double matrices for the converted values of the source image. 
//
// Description
//lab16 = lab2uint16(lab) converts an M-by-3 or M-by-N-by-3 array of L*a*b* color values to uint16. lab16 has the same size as lab. L*a*b* arrays that are uint8 or uint16 follow the convention in the ICC profile specification (ICC.1:2001-4, www.color.org) for representing L*a*b* values as unsigned 8-bit or 16-bit integers. The ICC encoding convention is illustrated by these tables.
//
// Examples
// vw = [100,0,0];
// w1 = list(w);
// w1(1) = [100 0 0];
// w1(2) = [0 100 0];
// w1(3) = [0 0 100];
// tr = lab2uint16(w1);
// tr
//
	a = raw_lab2uint16(pstData);
	d = size(a);
	for i=1:d
		output(:,:,i) = a(i);
	end
endfunction
