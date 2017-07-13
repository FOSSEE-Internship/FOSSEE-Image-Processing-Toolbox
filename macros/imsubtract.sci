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
function [out] = imsubtract(srcImg1, srcImg2)
//Subtract the contents of 2 matrices or images and store it inn another
//
//Calling Sequence
//outputMat = imsubtract(input1, input2)
//
//Parameters
//input1 : First input matrix or an image
//input2 : Second input matrix or image
//
//Description
//p=imsubtract(a,b) subtracts each of the element in the first argument's array with the corrresponding element in the second argument's array and the answer is stored in p.
//
//Examples
//a = imread("lena.jpeg");
//k = imsubtract(a,50);
//imshow(k)
//
//Examples
//a = imread("lena.jpeg");
//b = imread("lena1.jpeg"):
//k = imsubtract(a,b);
//imshow(k)
//
//Examples
//a = [19 20 21;23 54 11];
//b = [9 10 11;3 34 -9];
//c = imsubtract(a,b);
//disp(c)
//
//Authors
//Tess Zacharias
//Shubham Lohakare
	srcMat1=mattolist(srcImg1)
	srcMat2=mattolist(srcImg2)
	
	output = raw_imsubtract(srcMat1, srcMat2)
	channels = size(output)

	for i = 1: channels
		out(:,:,i)= (output(i))
	end
	out=double(out)
	
endfunction
