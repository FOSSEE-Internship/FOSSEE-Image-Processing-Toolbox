// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shubheksha Jalan
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function output = getgaussiankernel(ksize, sigma, ktype)
	//Returns Gaussian filter coefficients 
	//
	//Calling Sequence
	//output = getgaussiankernel(ksize, sigma, ktype)
	//
	//Parameters
	//ksize : Aperture size. It should be odd and positive.
	//sigma : Gaussian standard deviation.
	//ktype : Type of filter coefficients. It can be CV_32f or CV_64F. 
	//
	//Description
	//The function computes and returns the ksize x 1 matrix of Gaussian filter coefficients.
	//
	//Examples
	//output = getgaussiankernel(3,1,'CV_32F');
	//Authors
	//    Shubheshka Jalan   
		
	output = raw_getgaussiankernel(ksize, sigma, ktype)
	
endfunction
