// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Tanmay Chaudhari
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function out = integralFilter(intimage, bbox, weights, filterSize)
// Integral Image based Filter.
//
// Calling Sequence
// filter = integralFilter(intimage,bbox,weights,filterSize);
//
// Parameters
// intimage: Integral Image, which can be obtained from the function integralImage.
// bbox: Bounding box of the filter object, which can be obtained from integralKernel function.
// weight: Weights of the bounding box, which can be obtained from integralKernel function.
// filterSize: Size of the filter, which can be obtained from integralKernel function.
//
// Description
// This function filters image using box filters and integral images.
//
// Examples
// i = imread("images/lena.jpg");
// intImg = integralImage(i);
// kernel = integralKernel([2 2 11 11], 1/51);
// filter = integralFilter(intImg, kernel.bbox, kernel.weights, kernel.filterSize);
//
// See also
// integralImage
// integralKernel
// imread
//
// Authors
// Tanmay Chaudhari


	[lhs rhs] = argn(0)
	if rhs < 4
		error(msprintf("Not enough input arguments"));
	elseif rhs > 4
		error(msprintf("Too many input arguments"));
	end
	if lhs > 1
		error(msprintf("Too many output arguments"));
	end

 	inputimage1 = mattolist(inputimage);
 	a = raw_integralFilter(inputimage1, bbox, weights, filterSize);
 	out(:, :, 1) = a(1);
endfunction