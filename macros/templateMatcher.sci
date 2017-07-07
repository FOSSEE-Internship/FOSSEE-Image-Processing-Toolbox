// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Suraj Prakash,Rohan Gurve 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [new_image,x,y] = templateMatcher(image, template_image)
// This function compares a template in overlapped image regions and returns the coordinates of the point of match.
//
// Calling Sequence
// [new_image x y] = templateMatcher(image, template_image)
// 
// Parameters
// image : 8-bit int or 32-bit floating image
// template_image : Searched template. It must not be greater than the source image and have the same data type of input image.
// newimage : Map of comparison results. It must be single channel 32-bit floating-point. Its size is (W-w+1) * (H-h+1) if image 
// is W * H and template_image is w * h
// x: the row no. of the location where the template matched occured in the input image
// y: x: the column no. of the location where the template matched in the input image
// 
// Description
// The function compares the w * h sized images of the original image against the template image and stores the result in the new_image.
// The point of maxima in the image gives the position of the match.
//
// Examples
// src= imread("/images/color3.jpg"); //reading the source image
// gray_src = rgb2gray(src); //converting src to grayscale
// tem = imread("/images/smile.jpg"); //reading the template image
// tem = rgb2gray(tem);//converting template image to gray 
// imshow(tem);//view the template image
// [j x y] =templateMatcher(gray_src,tem); //perform the template matching
// [rows cols] =size(tem)
// t = rectangle(src,x,y,x+cols,y+rows,0,0,255,2,8,0);  //mark the position of the point of template match in the src image  
// imshow(j) //view the CV_64F output image of templateMatcher   
// imshow(t) //view the src image with the template object marked
// 
//
// Authors
// Suraj Prakash
// Rohan Gurve


		[ lhs rhs ] = argn(0)
		if lhs > 3 then
			error(msprintf("Too many output argument"))
		elseif lhs < 1 then
			error(msprintf("output arguments missing"))
		
		end
		
		
		if rhs > 2 then
			error(msprintf("Too many input arguments"))
		end	
		
		[imagerows imagecols imagechannel] = size(image)
		[t_rows t_cols t_channel] = size(template_image)
		if t_rows > imagerows | t_cols > imagecols then
			error(msprintf("Template image is greater than image\n"))
		end
		
		image_list = mattolist(image)
		template_image_list = mattolist(template_image)
		
			
			[temp x y] = raw_templateMatcher(image_list, template_image_list)
		
		
		sz = size(temp)
		
		for i=1 : sz
			new_image(:, :, i) = temp(i)
		end
		
endfunction
