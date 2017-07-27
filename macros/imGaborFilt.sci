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
function new_image = imGaborFilt(image, wavelength, orientation)
// The function applies Gabor filter or set of filters to 2-D image.
//
// Calling Sequence
// [new_image] = imGaborFilt(image, wavelength, orientation)
// 
// Parameters
// image : The input grayscale image.
// wavelength : It is the wavelength of the sinusoidal carrier, specified as a numeric scalar in the range [2,Inf), in pixels/cycle.
// orientation : Orientation value of filter in degrees, specified as a numeric scalar in the range [0 360], where the orientation is defined as the normal direction to the sinusoidal plane wave.
//
// Description
// It computes the magnitude and phase response of a Gabor filter for the input grayscale image. 
//
// Examples
// // apply Single Gabor Filter to Input Image
// a = imread("/images/lena.jpeg", 0);
// wavelength = 4;
// orientation = 90;
// b = imGaborFilt(a, wavelength, orientation)
//
// Authors
// Samiran Roy 		
	image_list = mattolist(image)
	
	out = raw_imGaborFilt(image_list, wavelength, orientation)
	
	sz = size(out)
	
	for i=1:sz
		new_image(:, :, i) = out(i)
	end

endfunction
