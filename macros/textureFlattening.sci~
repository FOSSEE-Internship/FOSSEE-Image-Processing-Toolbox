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
function output_image = textureFlattening(src,mask,varargin) 
// This function washes out the texture of the selected region in the input image, giving its contents a flat aspect. 
//
//  Calling Sequence
//  src = imread("image-location-for-src");
//  mask = roiFreeHand(src); //we can also use other function to create the mask
//  outputImage = textureFlattening(src,mask,low_threshold,high_threshold, kernel_size); 
//
// Parameters
// outputImage: resultant Output image 
// src:	Input source image.
// mask: Input mask image with same size as that of source image ( 1 or 3-channel image)
// low_threshold:	Range from 0 to 100. (double) (default value is 30)
// high_threshold:	Value > 100 & <=255. (double) (default value is 120)
// kernel_size:	The size of the Sobel kernel to be used: it can be 3 , 5 or 7
//  
// Description
// It retain the gradients at edge locations, before integrating with the Poisson solver. It uses Canny Edge Detector. 
// The algorithm assumes that the color of the source image is close to that of the destination. 
// This assumption means that when the colors don't match, the source image color gets tinted toward the color of the destination image.
//
// Examples
// 
// src = imread("/images/color2.jpeg");
// mask = roiFreeHand(src); //creating a mask
// imshow(mask);//view mask
//
// outputImage1 = textureFlattening(src,mask) // using default value for optional parameters
// imshow(outputImage1); // view outputImage1 
//
// outputImage2 = textureFlattening(src,mask,50 , 180.5 ) //providing value for 1st and 2nd optional argument
// imshow(outputImage2); // view outputImage2 
//
// outputImage2 = textureFlattening(src,mask,[],[], 7 ) //providing value for only 3rd optional argument
// imshow(outputImage3); // view outputImage3 
//
// Examples
// 
// src = imread("/images/color2.jpeg"); 
// [row col] = size(src)
// mask = zeros(row,col) //creating a mask
// mask(100:140,100:150) =255
// imshow(mask); view the mask 
// outputImage = textureFlattening(src,mask)
// imshow(outputImage);//view the output
//
// Authors
//  Rohan Gurve
	
	[lhs rhs]=argn(0);
    if rhs>5 //max i/p arguments are 5
        error(msprintf(" Too many input arguments"));
    elseif rhs<2 //min i/p arguments are 2
        error(msprintf("input arguments missing"));
    end
    
    if lhs>1
	    error(msprintf("Too many output arguments"));
	end
	
	image_list1 = mattolist(src);
	image_list2 = mattolist(mask);
	
	low_threshold = argindefault ( varargin , 1 , 30 );//default value is 30
    high_threshold = argindefault ( varargin , 2 , 120 );//default value is 120
    kernel_size = argindefault ( varargin , 3 , 3 );//default value is 3
    
	
	out = raw_textureFlattening(image_list1,image_list2,low_threshold,high_threshold, kernel_size);
	
	sz = size(out);
	
	for i=1:sz
		output_image(:, :, i) = out(i)
	end
	
endfunction


