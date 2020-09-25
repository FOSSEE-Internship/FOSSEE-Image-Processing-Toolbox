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
function [multiplyMat] = immultiply(srcImg1, srcImg2)
//Multiply the contents of 2 matrices and store it in another
//
//Calling Sequence
//multiplyMat = immultiply(input1, input2)
//
//Parameters
//input1 : First input matrix or an image
//input2 : Second input matrix or image
//
//Description
//p=immultiply(a,b) multiplies each of the element in the first argument's array with the corrresponding element in the second argument's array and the answer is stored in p.
//
//Examples
//a = imread("lena.jpeg");
//k = immultiply(a,2);
//imshow(k)
//
//Examples
//a = imread("photo.jpg");
//k = immultiply(a,0.5);
//imshow(k)
//
//Examples
//a = [1 2 3; 4 5 6];
//b = [10 20 30;10 20 30];
//k = immultiply(a,b);
//disp(k)
//
//Authors
//Tess Zacharias
//Shubham Lohakare
	srcMat1=mattolist(srcImg1)
	srcMat2=mattolist(srcImg2)
	out = raw_immultiply(srcMat1, srcMat2)
	
	channel = size(out)
	
	for i = 1: channel
		multiplyMat(:,:,i) = out(i)
	end
endfunction
