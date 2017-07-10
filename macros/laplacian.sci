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

function outputImg = laplacian(inputImage , depth , ksize , scale, delta)
	//The function calculates laplacian of an image
	//
	//Calling Sequence
	//inputImage = imread('path of the source image')
	//outputImg = laplacian(inputImage , depth , ksize , scale, delta)
	//
	//Parameters
	//inputImage : source image 
	//depth : Depth of the output image
	//ksize : Aperture size	
	//scale : Optional scale factor for the computed Laplacian values
	//delta : This value is added to each pixel in the output image
	//
	//Description
	//Laplacian Operator is also a derivative operator which is used to find edges in an image.
	//
	//Examples
	//img=imread('images/lena.jpg');
	//out=laplacian(img,'CV_8U',3,1,0);
	//Authors
	//    Sukul Bagai

    inputList=mattolist(inputImage)
    outputList=raw_laplacian(inputList , depth , ksize , scale, delta)
    for i=1:size(outputList)
        outputImg(:,:,i)=outputList(i)
    end
    outputImg=double(outputImg)	
endfunction
