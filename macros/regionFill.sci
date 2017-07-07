// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Vinay Bhat, Rohan Gurve
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function[dstImg] = regionFill(srcImg,inpaintRadius,varargin)
// This function restores the selected region in an image using the region neighborhood.
// This function restores the selected region in an image using the region neighborhood.
//
//
// Calling Sequence
// src = imread(location-for-image) 
// x = [x1 x2 ...........xn ] //x coordinates of polygon covering the region that you want to fill
// y = [y1 y2 ...........yn ] //corresponding y coordinates of polygon covering the region that you want to fill
// dstImg 1= regionFill(srcImg,inpaintRadius,x,y)
// 
// mask = roiFreeHand(srx) //making a mask - you can also use other function to make the mask
// dstImg2 = regionFill(srcImg,inpaintRadius,mask)
// 
// Parameters
//
// srcImg:	input source imge //it is converted to 8 bit internally
// inpaintRadius:	Radius of a circular neighborhood of each point inpainted that is considered by the algorithm(Double)
// mask:	Inpainting mask. Non-zero pixels indicate the area that needs to be inpainted.
// x: 1xn matrix denoting x coordinates of polygon covering the region that you want to fill
// y: 1xn matrix denoting y coordinates of polygon covering the region that you want to fill
//
// Examples
// src= imread("/images/regionFill_test5.jpg");
// p=regionFill(src,1,[100 110 110 100],[150 150 200 200]);
// imshow(p);
//
// Examples
// src= imread("/images/regionFill_test5.jpg"); //reading an image
// mask=roiFreeHand(src); //making a mask
// p=regionFill(src,5,mask);
// imshow(p);
//
//Authors
//Vinay Bhat
//Rohan Gurve
	
	[lhs, rhs] = argn(0)
	
	if rhs < 3
	    error(msprintf("input arguments missing"));
	elseif rhs > 4
	    error(msprintf(" Too many input arguments"));
    end
	
	srcMat = mattolist(srcImg)

	if rhs == 3 then
		maskMat = mattolist(varargin(1))
		out = raw_regionFill(srcMat,inpaintRadius, maskMat)
	elseif rhs == 4 then
		x= varargin(1)
		y= varargin(2)
		out = raw_regionFill(srcMat,inpaintRadius, x, y)
	end
	
	channels = size(out)
	
	for i = 1:channels
		dstImg(:,:,i) = out(i)
	end
endfunction
