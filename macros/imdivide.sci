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
function [out] = imdivide(srcImg1, srcImg2)
// Divides one image with another or divide image by constant.
//
//Calling Sequence
//outputMat = imdivide(input1, input2)
//
//Parameters
//input1 : Image or an array
//input2 : Image or an array of same size or a scalar
//
//Description
//imdivide takes 2 inputs, either both images which should be of the same size or one image and another scalar which is used as the divisor. Also the inputs can be two arrays of the same size or one array and one scalar. The output is the quotient of the two.
//
//Examples
//k = imread("lena.jpeg");
//p = imdivide(k,2);
//imshow(p);
//
//Examples
//k = imread("photo.jpg");
//p = imdivide(k,0.8);
//imshow(p);
//
//Examples
//a = [10 20;50 10];
//b = 5;
//p = imdivide(a,b);
//
//Authors
//Tess Zacharias
//Shubham Lohakare        	
	srcMat1=mattolist(srcImg1)
	srcMat2=mattolist(srcImg2)
			
   	output=raw_imdivide(srcMat1, srcMat2)
         
	channels = size(output)
         
	for i = 1: channels
        	out(:,:,i)=(output(i))
        end
     	out = double(out)
endfunction;
