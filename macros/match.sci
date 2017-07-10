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

function [x] = match(a,b)
	//Performs matching features between two images.
	//
	//Calling Sequence
	//img1=imread('path of the image file')
	//img2=imread('path of the image file')
	//x=match(img1,img2)
	//
	//Parameters
	//img1 : an image
	//img2 : an image
	//
	//Description
	//The images pass through a stiching pipeline.
	//Features are extracted from each image and matching is done on two images.
	//
	//Examples
	//img1=imread('images/campus_000.jpg');
	//img2=imread('images/campus_001.jpg');
	//x=match(img1,img2);
	//Authors
	//    Manoj Sree Harsha

c=mattolist(a);
d=mattolist(b);
[y]=raw_match(c,d);
channels = size(y)
	for i = 1:channels
		x(:, :, i) = ((y(i)))
	end
endfunction
