// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shubheksha Jalan
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [outputImg]= cornerMinEigenVal(inputImage, blockSize, kSize, borderType)
	  //Calculates the minimal eigenvalue of gradient matrices for corner detection.
	  //
	  //Calling Sequence
	  //inputImage=imread('path of the image file')
	  //outputImg = cornerMinEigenVal(inputImage, blockSize, kSize, borderType)
	  //imshow(outputImg)
	  //
	  //Parameters
	  //inputImage : an image.
	  //blockSize : Neighborhood size 
	  //kSize : Aperture parameter for the Sobel() operator.
	  //borderType : Pixel extrapolation method
	  //
	  //Description
	  //outputImg = cornerMinEigenVal(inputImage, blockSize, kSize, borderType)
	  //The function is similar to cornerEigenValsAndVecs() but it calculates only the minimal eigenvalue of the covariance matrix of derivatives.
	  //After that, it stores them in the destination image(outputImg).
	  //
	  //Examples
	  //inputImage=imread('images/lena.jpeg');
	  //blockSize = 7;
	  //kSize=3;
	  //k=0.04;
	  //borderType="BORDER_DEFAULT"
	  //outputImg=cornerMinEigenVal(inputImage, blockSize, kSize, borderType);
	  //imshow(outputImg)
	  //Authors
	  //    Shubheksha Jalan

	inputList=mattolist(inputImage);
    outputList=raw_cornerMinEigenVal(inputList, blockSize, kSize, borderType);
    for i=1:size(outputList)
       outputImg(:,:,i)=outputList(i)
   end
endfunction
