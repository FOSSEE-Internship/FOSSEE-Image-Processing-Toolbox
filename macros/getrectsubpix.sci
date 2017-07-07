// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shubheksha Jalan,Gursimar Singh
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function dst = getrectsubpix(I, alpha, beta, centerX, centerY, patchType)
// This fucntion is used to retrieve a pixel rectangle from an image with sub-pixel accuracy.
//
// Calling Sequence
// dst = getrectsubpix(I, alpha, beta, centerX, centerY, patchType)
//
// Parameters
// I : image matrix of the source image.
// alpha : width of the extracted patch.
// beta : height of the extracted patch.
// center_x : Floating point x coordinate of the center of the extracted rectangle within the source image. The center must be inside the image.
// center_y : Floating point y coordinate of the center of the extracted rectangle within the source image. The center must be inside the image.
// dst : Extracted patch that has the alpha*beta and the same number of channels as source image.
// patchType :  It is the depth of the extracted pixels.By default, the patchType have the same depth as source image.
//
// Description
// The function getRectSubPix extracts pixels from I : dst(x, y) = I(x + center.x- ( dst.cols -1)*0.5, y +  center.y - ( dst.rows -1)*0.5), where the values of the pixels at non-integer coordinates are retrieved using bilinear interpolation.	
//
// Examples
// I = imread('images/lena.jpeg',0);
// rr = getrectsubpix(I,5,5,6,7,1);
// imshow(rr);
//
//Authors
//Shubheksha Jalan
	
	image_list = mattolist(image)
	
	out = raw_getrectsubpix(image_list, alpha, beta, centerX, centerY, patchType)
	
	sz = size(out)
	
	for i = 1 : sz
		dst(:, :, i) = out(i)
	end
	
endfunction
