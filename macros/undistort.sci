// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Gursimar Singh,Sukul Bagai,Shubheksha Jalan
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [outputImg]=undistort(inputImage,cameraMatrix,distCoeffActual,varargin)
//The function undistorts an input image.The distortions in the image are due to the properties of the camera.    
//
//Calling Sequence 
//[outputImg]=undistort(inputImage,cameraMatrix,distCoeffActual)
//[outputImg]=undistort(inputImage,cameraMatrix,distCoeffActual,NewCameraMatrix);
//
//Parameters
//outputImg:It is the unsdistorted ouput image with same size as inputImage.
//inputImage:The distorted image.
//cameraMatrix: Input camera matrix 
//distCoeffActual=Input vector of distortion coefficients of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
//NewCameraMatrix:Camera matrix of the distorted image. By default, it is the same as cameraMatrix but you may additionally scale and shift the result by using a different matrix. 
//
//Description
//Transforms an image to compensate for lens distortion.Those pixels in the destination image, for which there is no correspondent pixels in the source image, are filled with zeros (black color).The function transforms an image to compensate radial and tangential lens distortion.
//
//Examples
//boardCols=7;
//boardRows=10;
//checkerSize=10;
//worldPoint=genCheckerboardPoints([boardCols boardRows],checkerSize);
//imagePoints=detectCheckerboardCorner(im,[boardRows,boardCols]);
//imagePoints=list(imagePoints)
//im=imread("images/checkerboard.jpg",0);
//sz=size(im);
//f=calibrateCamera(worldPoints,imagePoints,[sz(2),sz(1)]);
//image=undistort(im,f.cameraMatrix,f.distortionCoefficients);
//imshow(image)
//
//Authors
//Gursimar Singh
//Sukul Bagai
//Shubheksha Jalan
//
//See also
//genCheckerboardPoints
//detectCheckerboardCorner
//calibrateCamera

    [lhs,rhs]=argn(0);

	if rhs>4 then
        error(msprintf("Too many input arguments"));
     end
     if rhs<3 then
	   error(msprintf("Not enough input arguments"));
	 end
	 if lhs >1
	   error(msprintf("Too many output arguments"));
     end

	inputList=mattolist(inputImage);
	if rhs==3 then 
        outputList=raw_undistort(inputList,cameraMatrix,distCoeffActual);
    elseif rhs==4 then
        outputList=raw_undistort(inputList,cameraMatrix,distCoeffActual,varargin(1));
    end
    for i=1:size(outputList)
        outputImg(:,:,i)=outputList(i)
    end
endfunction
