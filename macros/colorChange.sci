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
function output_image = colorChange(src,mask,varargin)
// This function seamlessly mixes two differently colored versions of the input image.
//
// Calling Sequence
// src = imread("image-location-for-src");
// mask = roiFreeHand(src); //we can also use other function to create the mask
// outputImage = colorChange(src, mask, red_mul, green_mul, blue_mul ) 
//
// Parameters
// outputImage: resultant Output image 
// src: Input source image 
// mask: Input mask image with same size as that of source image.
// red_mul:	    R-channel multiply factor (float) (default value= 1.0f)
// green_mul:	G-channel multiply factor.(float) (default value= 1.0f)  
// blue_mul: 	B-channel multiply factor.(float) (default value= 1.0f)
// -Multiplication factor is between .5 to 2.5. 
//
// Description
// Given an original color image, two differently colored versions of this image can be mixed seamlessly.
//
// Examples
// src = imread("/images/color2.jpeg");
// mask = roiFreeHand(src); //creating a mask
//
// outputImage1 = colorChange(src, mask) //using default value for all optional parameters
// imshow(outputImage1);//view the output
//
// outputImage2 = colorChange(src, mask,0.5 ) //specifying just the value of red_mul
// imshow(outputImage2);//view the output
//
// outputImage3 = colorChange(src, mask,0.5 , 1, 2.5 ) //specifying value of all optional parameter
// imshow(outputImage3);//view the output
// 
// outputImage4 = colorChange(src, mask,0.5 , [] , 2.5 ) //skipping the second argument
// imshow(outputImage4);//view the output
//
// Examples
// src = imread("../images/color2.jpeg"); 
// [row col] = size(src)
// mask = zeros(row,col) //creating a mask
// mask(100:140,100:150) =255
// imshow(mask); view the mask 
// outputImage = colorChange(src, mask,0.5 , 1, 2.5 ) //specifying value of all optional parameter
// imshow(outputImage);//view the output
//
// Authors
// Rohan Gurve
	
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
	
    
    red_mul = argindefault ( varargin , 1 , 1.0 );//default value is 1.0
    green_mul = argindefault ( varargin , 2 , 1.0 );//default value is 1.0
    border = argindefault ( varargin , 3 , 1.0 );//default value is 1.0 	
	
	out = raw_colorChange(image_list1,image_list2,red_mul,green_mul,blue_mul );
	
	sz = size(out);
	
	for i=1:sz
		output_image(:, :, i) = out(i)
	end
	
endfunction


