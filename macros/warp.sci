// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: M Avinash Reddy 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [out1,out2] = warp(img1,img2)
	//This function maps an image onto a unit sphere located at the origin (center of the image).  
	//
	//Calling Sequence
	//stacksize('max')
	//img1=imread('path of the image file')
	//img2=imread('path of the image file')
	//[out1,out2] = warp(img1,img2)
	//
	//Parameters
	//img1 : an image 
	//img2 : an image
	//
	//Description
	//The function takes two input images which are part of a panoromic image and outputs the respective masks to be applied on the input images to get a spherical oriented image.
        //This is useful in creating a 360 panaroma image.
	//
	//Examples
	//stacksize('max');
	//img1=imread('images/campus_017.jpg');
	//img2=imread('images/campus_016.jpg');
	//[out1,out2]=warp(img1,img2);
	//Authors
	//    M Avinash Reddy 
	i1 = mattolist(img1)
	i2 = mattolist(img2)
	[res1,res2] = raw_warp(i1,i2)
	channel = size(res1)
	for i = 1: channel
		out1(:,:,i) = (res1(i))
  	end
  	channel = size(res2)
  	for i = 1:channel
    		out2(:,:,i) = (res2(i))
	end
	out1 = double(out1)
  	out2 = double(out2)
endfunction
