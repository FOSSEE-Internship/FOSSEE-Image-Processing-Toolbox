// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Tess Zacharias, Shubham Lohakare 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [out] = imadd(srcImg1, srcImg2)
//Add the contents of 2 matrices or images and store it in another
//
//Calling Sequence
//outputMat = imadd(input1, input2)
//
//Parameters
//input1 : First input matrix or an image
//input2 : Second input matrix or image
//
//Description
//Z = imadd(X,Y) adds each element in array X with the corresponding element in array Y and returns the sum in the corresponding element of the output array Z. X and Y are real, nonsparse numeric arrays with the same size and class, or Y is a scalar double. Z has the same size and class as X, unless X is logical, in which case Z is double.
//
//Examples
//a = [1 5 6;4 2 3];
//b = [8 10 15;5 1 9];
//c = imadd(a,b);
//
//Examples
//a = imread("lena.jpeg");
//b = imread("lena1.jpeg");
//c = imadd(a,b);
//imshow(c)
//
//Authors
//Tess Zacharias
//Shubham Lohakare

	srcMat1=mattolist(srcImg1)
	srcMat2=mattolist(srcImg2)
	
	output = raw_imadd(srcMat1, srcMat2)
	channels = size(output)

	for i = 1: channels
		out(:,:,i)= (output(i))
	end
	out=double(out)
	
endfunction
