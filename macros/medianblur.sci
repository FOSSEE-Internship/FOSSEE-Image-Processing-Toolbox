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

function outputImg= medianblur(inputImage,ksize)
	//Blurs an image using the median filter. 
	//
	//Calling Sequence
	//inputImage = imread('path of the image')
	//outputImg = medianblur(inputImage,ksize)
	//
	//Parameters
	//inputImage : source image 
	//ksize :  aperture linear size; it must be odd and greater than 1
	//
	//Description
	//The function smoothes an image using the median filter with the ksize x ksize aperture. 
	//
	//Examples
	//inputImage = imread('images/lena.jpg');
	//ouputImg = medianblur(inputImage,3);
	//Authors
	//    Sukul Bagai

	inputList=mattolist(inputImage);
    outputList=raw_medianblur(inputList,ksize)
    for i=1:size(outputList)
        outputImg(:,:,i)=outputList(i)
    end
endfunction
