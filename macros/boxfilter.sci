// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shubham Lohakare, Yash Balghat 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [out]=boxfilter(InputArraysrc, intddepth,size1x,size1y,pointx,pointy, bool_normalize)
//Blurs an image using the box filter.
//
//Calling Sequence
//outputImage = boxfilter(InputArraysrc, intddepth,size1x,size1y,pointx,pointy, bool_normalize)
//
//Parameters
//InputArraysrc : Input image to be blurred
//intddepth : The output image depth (-1 to use src.depth())
//size1x : Width for the blurring kernel size
//size1y : Height for the blurring kernel size
//pointx : x-coordinate for anchor point (Default value -1)
//pointy : y-coordinate for anchor point (Default value -1)
//bool_normalize : flag, specifying whether the kernel is normalized by its area or not
//
//Description
//Unnormalized box filter is useful for computing various integral characteristics over each pixel neighborhood, such as covariance matrices of image derivatives (used in dense optical flow algorithms, and so on).
//
//Examples
//a = imread("lena.jpeg");
//k=boxfilter(a,-1,40,80,-1,-1,"False");
//
//Examples
//a = imread("photo.jpg");
//k=boxfilter(a,-1,8,8,-1,-1,"True");
//
//Authors
//Shubham Lohakare, NITK Surathkal
//Priyanka Hiranandani, NIT Surat


         input_image1=mattolist(InputArraysrc)
         a=raw_boxfilter(input_image1, intddepth,size1x,size1y,pointx,pointy, bool_normalize)
         dimension=size(a)
         for i = 1:dimension
              out(:,:,i)=(a(i))
         end
     
endfunction
