// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Rohan Gurve 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function dst = convert(src,rtype, varargin)
// This function converts a matrix to another data type with optional scaling.
//
// Calling Sequence
// src = imread('location-of-src');
// dst = convert(src,rtype, alpha,beta);
// 
// Parameters
// src : the source matrix/image
// alpha: optional scale factor (default value is 1) (type- Double)
// beta: optional delta added to the scaled values (default value is 0) (type- Double)
// rtype: (type- string)desired output matrix type/depth.It supports the following types -> 1) CV_8U, 2) CV_8S, 3) CV_16U ,4) CV_16S, 5) CV_32S, 6) CV_64F
//
// Description
// The method converts source pixel values to the target data type. saturate_cast<> is applied at the end to avoid possible overflows-
//
// Examples
// src = imread('/images/color2.jpeg');
// dst = convert(src,'CV_8U') //convert to 8 bit unsigned int using default values of alpha and beta 
// dst //viewing the content of dst
//
// dst2 = convert(src,'CV_64F') //convert to 64 bit floating point using default values of alpha and beta
// dst2 //viewing the content of dst2
//
// Authors
// Rohan Gurve


		[ lhs rhs ] = argn(0)
		if lhs > 1 then
			error(msprintf("Too many output argument"))
		end
		
		if rhs > 4 then
			error(msprintf("Too many input arguments"))
		elseif rhs < 2 then
			error(msprintf("Input arguments missing"))
		end	
		
	    image_list = mattolist(src)
		
		alpha = argindefault ( varargin , 1 , 1 );//default value is 1
        beta = argindefault ( varargin , 2 , 0 );//default value is 0	
	    
	    temp = raw_convert(image_list, rtype, alpha,beta)
		
		
		sz = size(temp)
		
		for i=1 : sz
			dst(:, :, i) = temp(i)
		end
		
endfunction
