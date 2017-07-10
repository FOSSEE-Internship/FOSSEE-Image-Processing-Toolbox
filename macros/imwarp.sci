// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: V Srinivas & M Avinash Reddy 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [output] = imwarp(img,m)
	//The function applies an affine transformation to the input image.
	// 
	//Calling Sequence
	//img=imread('path of the image')
	//output=imwarp(img,transformation_matrix)
	//
	//Parameters
	//img : Input image 
	//m : It is the 2X3 transformation matrix.
	//
	//Description
	//The function warpAffine transforms the source image using the specified matrix:
	// dst(x,y) = src( ( m(1,1)*x + m(1,2)*y + m(1,3) ) , ( m(2,1)*x + m(2,2)*y + m(2,3) ) )	
	//
	//Examples
	//stacksize('max');
	//img=imread('images/lena.jpg');
	//m=[1 2 0;2 1 0];
	//out=imwarp(img,m);
	//Authors
	//    V Srinivas , M Avinash Reddy 

	image = mattolist(img);
	a = raw_imwarp(image,m)
	channels = size(a)
	for i=1:channels
		output(:,:,i) = a(i);
	end
	output=double(output)
endfunction
