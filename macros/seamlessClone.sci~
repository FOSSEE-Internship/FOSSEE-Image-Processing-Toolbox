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
function output_image = seamlessClone(src,dst,mask,	pointMatirx, flag)
// This function is used to seamlessly clone a source image onto a destination image.
//
// Calling Sequence
//  src = imread("image-location-for-src");
//  mask = roiFreeHand(src); //we can also use other function to create the mask
//  outputImage = seamlessClone(src,dst,mask, pointMatirx, flag) 
//
// Parameters
// outputImage: resultant Output image 
// src: Input  source image.
// dst: Input  destination image on which the source image will be cloned onto .
// mask: Input mask image with same size as that of source image.
// pointMatirx: a 1x2 vector denoting the x & y coordinates of the point in dst where the src image will be cloned (x & y are Double)
// flag( Cloning method that could be one of the following):1) 1 - NORMAL_CLONE ; 2) 2 - MIXED_CLONE ; 3) 3 - FEATURE_EXCHANGE
//  
// Description
// Image editing tasks concern either global changes (color/intensity corrections, filters, deformations) or 
// local changes concerned to a selection. Here we are interested in achieving local changes, ones 
// that are restricted to a region manually selected (ROI), in a seamless and effortless manner. 
// The extent of the changes ranges from slight distortions to complete replacement by novel content
// 
//
//Note:  The size of the source image should be equal to or less than the destination image. The point 'p' 
//       should be chosen such that it fully fits inside the destination image.If not, then this function will throw the error=>
//        -(minxd >= 0 && minyd >= 0 && maxxd <= dest.rows && maxyd <= dest.cols) in seamlessClone
//   
// Examples
// dst = imread("/images/color3.jpg"); //read destination image
// src = imread("/images/cartoon.jpg"); //read source image
// mask = roiFreeHand(src); //create a mask for source image
// output = seamlessClone(src,dst,mask,[75 120], 1 ); //perform the seamelessClone 
// imshow(output);
//
// Examples
// dst = imread("/images/color3.jpg"); //read destination image
// src = imread("/images/cartoon.jpg"); //read source image
// [rows cols] = size(src)  //creating a mask for source image
// mask = zeros(rows,cols);
// mask(50:rows,1:cols) = 255;
// output = seamlessClone(src,dst,mask,[75 120], 1 ); //perform the seamelessClone 
// imshow(output);
//
//
// Authors
//  Rohan Gurve

	
	[lhs rhs]=argn(0);
    if rhs>5
        error(msprintf(" Too many input arguments"));
    elseif rhs<5
        error(msprintf("input arguments missing"));
    end
    
    if lhs>1
	    error(msprintf("Too many output arguments"));
	end
	
	image_list1 = mattolist(src);
	image_list2 = mattolist(dst);
	image_list3 = mattolist(mask);
	
	out = raw_seamlessClone(image_list1,image_list2,image_list3,pointMatirx, flag);
	
	sz = size(out);
	
	for i=1:sz
		output_image(:, :, i) = out(i)
	end
	
endfunction

