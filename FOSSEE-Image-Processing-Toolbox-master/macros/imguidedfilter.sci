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

function guidedFiltered_image = imguidedfilter(image,varargin)
// The function is used for smoothening of an image by application of a filter.
//
// Calling Sequence
// B = imguidedfilter(A);
// B = imguidedfilter(A,G);
// B = imguidedfilter(A,G,r);
// B = imguidedfilter(A,G,r,e);
//
// Parameters
// A: image matrix of the source image.
// G: image matrix of the guidance image.
// B: output image with it's histogram matching similar to a given reference image. 
// r: Size of the rectangular neighborhood around each pixel used in guided filtering, specified as a scalar  of positive integers.For eg if the value is specified  as Q, the neighborhood is a square of size [Q Q].
// e: Amount of smoothing in the output image, specified as a positive scalar. If you specify a small value, only neighborhoods with small variance (uniform areas) will get smoothed and neighborhoods with larger variance (such as around edges) will not be smoothed. If you specify a larger value, high variance neighborhoods, such as stronger edges, will get smoothed in addition to the relatively uniform neighborhoods. Start with the default value, check the results, and adjust the default up or down to achieve the effect you desire.
//
// Description
// B = imguidedfilter(A,G) filters binary, grayscale, or RGB image A using the guided filter, where the filtering process is guided by image G. G can be a binary, grayscale or RGB image and must have  the same number of rows and columns as A.
// B = imguidedfilter(A) filters input image A under self-guidance, using A itself as the guidance image. This can be used for edge-preserving smoothing of image A.B = imguidedfilter(__,r,e) filters the image A using name-value pairs to control aspects of guided filtering. Parameter names can be abbreviated.
//
// Examples
// img = imread("lena.jpeg");
// imshow(img);
// guidedFiltered_image = imguidedfilter(img, img, 9);
// imshow(guidedFiltered_image);
//

	
	[lhs, rhs] = argn(0)
	
	image_list = mattolist(image)
	
	select rhs
		case 1
			out = raw_imguidedfilter(image_list)
		
		case 2
			out = raw_imguidedfilter(image_list, varargin(1))
		
		case 3
			out = raw_imguidedfilter(image_list, varargin(1), varargin(2))
		
		case 4
			out = raw_imguidedfilter(image_list, varargin(1), varargin(2), varargin(3))
	end
	
	sz = size(out)
	for i=1:sz
		guidedFiltered_image(:, :, i) = out(i)
	end
	
endfunction
