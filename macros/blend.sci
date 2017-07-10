// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: M Avinash Reddy & Manoj Sree Harsha
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [out]=blend(img1,img2,w1,w2)
	//Blends two images according to corresponding weights given by the user
	//
	//Calling Sequence
	//out = blend(img1,img2,w1,w2)
	//
	//Parameters
	//img1 : first image 
	//img2 : second image
	//w1 : a number between 1 and 100 denoting the proportion of the first image in the output 
	//w2 : a number between 1 and 100 denoting the proportion of the second image in the output
	//
	//Description
	//out = blend(img1,img2,w1,w2) returns a blended image which has size equal to that of first image.
	//Both the images are resized to same dimensions before blending the images.
	//Note that the weights are in percentage.
	//
	//Examples
	//w1=40;
	//w2=60;
	//img1=imread('images/lena.jpg');
	//img2=imread('images/monkey.jpg');
	//out=blend(img1,img2,w1,w2);
	//Authors
	//    M Avinash Reddy , Manoj Sree Harsha

	i1=mattolist(img1)
	i2=mattolist(img2)
	res=raw_Blend(i1,i2,w1,w2)
	channel=size(res)
	for i = 1: channel
		out(:,:,i) = (res(i))
	end
	out=double(out)
endfunction
