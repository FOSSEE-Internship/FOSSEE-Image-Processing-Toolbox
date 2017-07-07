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

function [output] = lab2xyz(vartype,varargin)
// This function is used to Convert CIE 1976 L*a*b* to CIE 1931 XYZ. 
//
// Calling Sequence
// xyz = lab2xyz(lab)
// xyz = lab2xyz(lab,Name,Value) 
//
// Parameters
// lab: Color values to convert, specified as a P-by-3 matrix of color values (one color per row), an M-by-N-by-3 image array, or an M-by-N-by-3-by-F image stack.
// Whitepoint: Reference white point, specified as a 1-by-3 vector or one of the CIE standard illuminants.eg,'a','c','e','d50','d55','d65','icc'.
// xyz : Converted color values, returned as an array the same shape and type as the input. 
//
// Description
// xyz = lab2xyz(lab) converts CIE 1976 L*a*b* values to CIE 1931 XYZ values.xyz = lab2xyz(lab,Name,Value) specifies additional options with one or more Name,Value pair arguments.
//
// Examples
// tr1 = lab2xyz([50 10 -5]);
// tr1
// tr2 = lab2xyz([50 10 -5],'WhitePoint','d50');
// tr2
//

	[lhs rhs] = argn(0);
	if(rhs>3)
		eror(msprintf("Too many input arguments"));
	end
	select rhs
	case 1 then
		a = raw_lab2xyz(vartype);
	case 2 then
		a = raw_lab2xyz(vartype,varargin(1));
	case 3 then
		a = raw_lab2xyz(vartype,varargin(1),varargin(2));
	end
	
	if(type(a)==15)
	d = size(a);
	for i=1:d
		output(:,:,i) = a(i);
	end
	end
	
	output = a
	
	
endfunction
		
