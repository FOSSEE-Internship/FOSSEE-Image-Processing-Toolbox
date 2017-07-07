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
function output_image = illuminationChange(src,mask,varargin)
// This function is used to reduce specular reflections or highlight under-exposed foreground objects.
//
// Calling Sequence
// src = imread("image-location-for-src");
// mask = roiFreeHand(src); //we can also use other function to create the mask
// outputImage = illuminationChange(src, mask, alpha, beta ) 
//
// Parameters
// outputImage: resultant Output image 
// src: Input source image
// mask: Input mask image with same size as that of source image.
// alpha:	Value ranges between 0-2. (float) (default value= 0.2f)
// beta:	Value ranges between 0-2. (float) (default value= 0.4f)
//  
//
// Description
// Applying an appropriate non-linear transformation to the gradient field inside the selection and then integrating 
// back with a Poisson solver, modifies locally the apparent illumination of an image.
//
// Examples
// 
// src = imread("/images/color2.jpeg");//reading an image
// mask = roiFreeHand(src); //creating a mask
// imshow(mask);//view mask
//
// outputImage1 = illuminationChange(src, mask) //using default arguments 
// imshow(outputImage1); // view outputImage1
//
// outputImage2 = illuminationChange(src, mask,1.9, 0.3) //providing all optional input arguments
// imshow(outputImage2); // view outputImage2
//
// outputImage3 = illuminationChange(src, mask,[], 0.3) //skipping first optional input argument
// imshow(outputImage3); // view outputImage3
//
// Examples
//
// src = imread("/images/color2.jpeg"); 
// [row col] = size(src)
// mask = zeros(row,col) //creating a mask
// mask(100:140,100:150) =255
// imshow(mask); view the mask 
// outputImage = illuminationChange(src, mask) //using default arguments 
// imshow(outputImage);//view the output
//
// Authors
// Rohan Gurve

	[lhs rhs]=argn(0);
    if rhs>4 // max i/p arguments are 4
        error(msprintf(" Too many input arguments"));
    elseif rhs<2// min i/p arguments are 2
        error(msprintf("input arguments missing"));
    end
    
    if lhs>1
	    error(msprintf("Too many output arguments"));
	end
	
	image_list1 = mattolist(src);
	image_list2 = mattolist(mask);
	
	alpha = argindefault ( varargin , 1 , 0.2 );//default value is 0.2
    beta = argindefault ( varargin , 2 , 0.4 );//default value is 0.4
    
	out = raw_illuminationChange(image_list1,image_list2,alpha,beta );
	
	sz = size(out);
	
	for i=1:sz
		output_image(:, :, i) = out(i)
	end
	
endfunction


