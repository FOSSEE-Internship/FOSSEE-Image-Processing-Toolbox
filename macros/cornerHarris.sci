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

function [outputImg]= cornerHarris(inputImage, blockSize, kSize, k, borderType)
 	//Harris edge detector.
	  //
	  //Calling Sequence
	  //inputImage=imread('path of the image file')
	  //outputImg = cornerHarris(inputImage, blockSize, kSize, k, borderType)
	  //imshow(outputImg)
	  //
	  //Parameters
	  //inputImage : an image.
	  //blockSize : Neighborhood size 
	  //kSize : Aperture parameter for the Sobel() operator.
	  //k :  Harris detector free parameter.
	  //borderType : Pixel extrapolation method
	  //
	  //Description
	  //outputImg = cornerHarris(inputImage, blockSize, kSize, k, borderType)
	  //This function runs the Harris edge detector on the image.
	  //outputImg is used to store the Harris detector responses
	  //
	  //Examples
	  //inputImage=imread('images/lena.jpeg');
	  //blockSize = 7;
	  //kSize=3;
	  //k=0.04;
	  //borderType="BORDER_DEFAULT"
	  //outputImg=cornerHarris(inputImage, blockSize, kSize, k, borderType);
	  //imshow(outputImg)
	  //Authors
	  //    Shubheksha Jalan

	inputList=mattolist(inputImage);
    outputList=raw_cornerHarris(inputList, blockSize, kSize, k, borderType);
    for i=1:size(outputList)
       outputImg(:,:,i)=outputList(i)
   end 
endfunction
