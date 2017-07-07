// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Tess  Zacharias, Ashish Manatosh Barik , Rohan Gurve
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function[dstImg] = watershed(srcImg, varargin)
// This function returns a marker based segmented image for the given input image.
// This function returns a marker based segmented image for the given input image.
//
// Calling Sequence
//
// src = imread("image-location-for-src");
// [dstImg] = watershed(srcImg) //uses distance transform to find the contours
// 
// src = imread("image-location-for-src");
// markerMat = roiFreeHand(src); //using roiFreeHand to roughly mark the contours manually by selecting the ROI
// [dstImg] = watershed(srcImg,markerMat) 
// src = imread("image-location-for-src");
//
// Parameters
// dstImg: output segmented image 
// src: Input source image 
// markerMat: a mask image with the the contours marked(region selected)
//
// Description
// The second calling sequence gives better results.Here the user has to manually select the region of interest using roiFreeHand.
// Go through the documentation of roiFreeHand to see how it works.  
//
// Examples
// src= imread("/images/watershed2.jpg");
// w2=watershed(src);// markers are created after performing distance transform by default
// imshow(w2);
//
// Examples
// src= imread("../images/watershed2.jpg");
// mask=roiFreeHand(src); //creating markers - selecting each coin separately using the function roiFreeHand
// w2=watershed(src,mask); //retrieve the segmented image
// imshow(w2);
//
//  Authors
//  Tess  Zacharias
//  Ashish Manatosh Barik
//  Rohan Gurve 


	srcMat = mattolist(srcImg)
	
	[lhs, rhs] = argn(0)

	select rhs
		case 1 then 
			out = raw_watershed(srcMat)
		case 2 then
			markerMat = mattolist(varargin(1))
			out = raw_watershed(srcMat, markerMat)
	end

	channels = size(out)
	
	for i = 1 : channels
		dstImg(:,:,i) = out(i)
	end

endfunction
