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

function new_image = sobel(image, ddepth, dx, dy, ksize, scale, delta)
// This function is used to calculate the first, second, third, or mixed image derivatives using an extended Sobel operator. 
//
// Calling Sequence
// B = sobel(A,ddepth,dx,dy,ksize,scale,delta)
//
// Parameters
// A: image matrix of the source image.
// ddepth : output image depth; the following combinations of src.depth() and ddepth are supported such as ,src.depth() = CV_8U, ddepth = -1/CV_16S/CV_32F/CV_64F,src.depth() = CV_16U/CV_16S, ddepth = -1/CV_32F/CV_64F,src.depth() = CV_32F, ddepth = -1/CV_32F/CV_64F,src.depth() = CV_64F, ddepth = -1/CV_64F,when ddepth=-1, the destination image will have the same depth as the source; in the case of 8-bit input images it will result in truncated derivatives.
// dx: order of the derivative x.
// dy: order of the derivative y.
// ksize: size of the extended Sobel kernel; it must be 1, 3, 5, or 7.
// scale: optional scale factor for the computed derivative values; by default, no scaling is applied (see getDerivKernels() for details).
// delta: optional delta value that is added to the results prior to storing them in dst.
// B : output image with it's histogram matching similar to a given reference image. 
//
// Description
// This function is used to calculate the first, second, third, or mixed image derivatives using an extended Sobel operator.In all cases except one, the ksize X ksize separable kernel is used to calculate the derivative. When ksize = 1 , the 3 X 1 or 1 X 3 kernel is used (that is, no Gaussian smoothing is done). ksize = 1 can only be used for the first or the second x- or y- derivatives.
//
// Examples
// i = imread('lena.jpeg',0);
// ii = sobel(i,"CV_8U",1,0,3,1,0);
//

	
	image_list = mattolist(image)
	
	out = raw_sobel(image_list, ddepth, dx, dy, ksize, scale, delta)
	
	sz = size(out)
	
	for i=1:sz
		new_image(:, :, i) = out(i)
	end
	
endfunction

