// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Samiran Roy 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [outputImg] = gabor(wavelength,orientation)
// This function creates a Gabor filter.
//
// Calling Sequence
// [outputImg] = gabor(wavelength,orientation)
// 
// Parameters
// wavelength : It is the wavelength of sinusoid, specified as a numeric scalar or vector, in pixels/cycle.
// orientation : It is the orientation of filter in degrees, specified as a numeric scalar in the range [0 180], where the orientation is defined as the normal direction to the sinusoidal plane wave.
// outputImg : The Gabor filter.
//
// Description
// It creates a Gabor filter with the specified wavelength (in pixels/cycle) and orientation (in degrees). If you specify wavelength or orientation as vectors, gabor returns an array of gabor objects, called a filter bank, that contain all the unique combinations of wavelength and orientation. For example, if wavelength is a vector of length 2 and orientation is a vector of length 3, then the output array is a vector of length 6. 
//
// Examples
// // Create an array of Gabor filters.
// wavelength = 20;
// orientation = 45;
// a = gabor(wavelength, orientation);
//
// Authors
// Samiran Roy	
	outputList = raw_gabor(wavelength,orientation);

	for i=1:size(outputList)
        	outputImg(:,:,i)=outputList(i)
    	end

endfunction
