// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Sukul Bagai 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [out]=dilate(input_image,actualkernel,anchor_x,anchor_y,iteration)
	//Dilates an image 
	//
	//Calling Sequence
	//input_image=imread('path of the image')
	//out = dilate(input_image,actualkernel,anchor_x,anchor_y,iteration)
	//
	//Parameters
	//input_image : source image
	//actualkernel : structuring element used for dilation
	//anchor_x : x-coordinate of the anchor within the kernel (default is -1)
	//anchor_y : y-coordinate of the anchor within the kernel (default is -1)
	//iteration : number of times dilation is applied
	//
	//Description
	//The function dilates the source image using the specified structuring element(actualkernel) that determines the shape of a pixel neighborhood
	//
	//Examples
	//img = imread('images/lena.jpg');
	//actualkernel=[1 1 1;1 1 1;1 1 1];
	//out = dilate(img,actualkernel,-1,-1,1);
	//Authors
	//    Sukul Bagai 

    input_image1=mattolist(input_image);
    a=raw_dilate(input_image1,actualkernel,anchor_x,anchor_y,iteration);
    dimension=size(a)
    for i = 1:dimension
        out(:,:,i)=a(i);
    end
endfunction;
