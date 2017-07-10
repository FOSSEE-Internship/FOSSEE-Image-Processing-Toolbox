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

function [outputImg]= cornerEigenValsAndVecs(inputImage, blockSize, kSize, borderType)
	  //Calculates eigenvalues and eigenvectors of image blocks for corner detection.
	  //
	  //Calling Sequence
	  //inputImage=imread('path of the image file')
	  //outputImg = cornerEigenValsAndVecs(inputImage, blockSize, kSize, borderType)
	  //imshow(outputImg)
	  //
	  //Parameters
	  //inputImage : an image.
	  //blockSize : Neighborhood size 
	  //kSize : Aperture parameter for the Sobel() operator.
	  //borderType : Pixel extrapolation method
	  //
	  //Description
	  //outputImg = cornerEigenValsAndVecs(inputImage, blockSize, kSize, borderType)
	  //For every pixel p , the function cornerEigenValsAndVecs considers a blockSize x blockSize    neighborhood  S(p) . It calculates the covariation matrix of derivatives over the neighbourhood 
	  //After that, it finds eigenvectors and eigenvalues and stores them in the destination image(outputImg).
	  //
	  //Examples
	  //inputImage=imread('images/lena.jpeg');
	  //blockSize = 7;
	  //kSize=3;
	  //borderType="BORDER_DEFAULT"
	  //outputImg=cornerEigenValsAndVecs(inputImage, blockSize, kSize, borderType);
	  //imshow(outputImg)
	  //Authors
	  //    Shubheksha Jalan

	inputList=mattolist(inputImage);
    outputList=raw_cornerEigenValsAndVecs(inputList, blockSize, kSize, borderType);
    for i=1:size(outputList)
       outputImg(:,:,i)=outputList(i)
   end
endfunction
