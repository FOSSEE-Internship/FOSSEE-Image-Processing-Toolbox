// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Siddhant Narang
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function varargout = isFilter(data, sz)
// Decides if a filter is separable or not.
//
// Calling Sequence
// [isFilter s u] = isFilter(filter, size);
//
// Parameters
// filter: Input filter (datatype- mat(double)).
// size: Size of the filter.
// 
// Description
// This function uses SVD to compute if the filter is separable or not. It takes as input the
// filter matrix and is the filter is separable it returns the calculated singular and the left
// singular values.
//
// Examples
// filter = [1 0; 0 1]; // Any matrix of dimensions n x n.
// [isfilter] = isFilter(filter);
//
// Examples
// filter = [1 0; 0 1]; // Any matrix of dimensions n x n.
// [isfilter s u] = isFilter(filter);
//
// Authors
// Siddhant Narang
	
	[lhs rhs] = argn(0)
	if rhs > 2 then
		error(msprintf("Too many input arguments. Two expected"))
	end
	if rhs < 2 then
		error(msprintf("Insufficient input arguments. Two expected"))
	end

	varargout = raw_isFilter("Data", data, "Size", sz);
endfunction