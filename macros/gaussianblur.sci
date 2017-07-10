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

function [outputImg]= gaussianblur(inputImage,ksize_height,ksize_width,sigmaX,sigmaY)
	//Blurs an image using a Gaussian filter
	//
	//Calling Sequence
	//inputImage = imread('path of the image')
	//outputImg = gaussianblur(inputImage,ksize_height,ksize_width,sigmaX,sigmaY)
	//
	//Parameters
	//inputImage : source image 
	//ksize_height : Gaussian kernel height. It must be positive and odd.
	//ksize_width : Gaussian kernel width. It must be positive and odd.
	//sigmaX : Gaussian kernel standard deviation in X direction.
	//sigmaY : Gaussian kernel standard deviation in Y direction.
	//
	//Description
	//The function convolves the source image with the specified Gaussian kernel.
	//
	//Examples
	//inputImage = imread('images/lena.jpg');
	//outputImg = gaussianblur(inputaImage,5,5,1,1);
	//Authors
	//    Sukul Bagai
	inputList=mattolist(inputImage);
    outputList=raw_gaussianblur(inputList,ksize_height,ksize_width,sigmaX,sigmaY);
    for i=1:size(outputList)
       outputImg(:,:,i)=outputList(i)
   end
endfunction
