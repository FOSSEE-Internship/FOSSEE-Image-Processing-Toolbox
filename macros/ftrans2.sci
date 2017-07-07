// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Vinay
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function h = ftrans2(b, varargin)
// 2-D FIR filter using frequency transformation.
//
// Calling Sequence
// h = ftrans2(b, t);
//
// Parameters
// b: A bandpass filter
// t: Transformation matrix
//
// Description
// Produces the two-dimensional FIR filter h that corresponds to the one-dimensional FIR filter b
// using the transform t. 
// (ftrans2 returns h as a computational molecule, which is the appropriate form to use with filter2.)
// b must be a one-dimensional, Type I (even symmetric, odd-length) filter such as can be returned by 
// fir1, fir2, or firpm in the Signal Processing Toolbox software. The transform matrix t contains 
// coefficients that define the frequency transformation to use. If t is m-by-n and b has length Q,
// then h is size ((m - 1) * (Q - 1) / 2 + 1)-by-((n - 1) * (Q - 1) / 2 + 1).
//
// Examples
// b = [1, 1, 1]
// h = ftrans2(b);
//
// Authors
// Vinay

	[lhs, rhs] = argn(0)
	
	_b = mattolist(b);

	select rhs
		case 1 then
			out = raw_ftrans2(_b)
	
		case 2 then
			out = raw_ftrans2(_b, varargin(1))
	end
	
	channel = size(out)
	
	for i = 1: channel
		h(:,:,i) = out(i)
	end
endfunction
