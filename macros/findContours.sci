// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Abhilasha Sancheti, Shubham Lohakare, Sukul Bagai 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [outputMat]= findContours(inputImage, Mode, method, point_x, point_y)
//Finds contours in an image
//
//Calling Sequence
//outputMat = findContours(inputImage, Mode, method, point_x, point_y)
//
//Parameters
//inputImage : The input image
//Mode : Contour retrieval mode (Enter 1 for CV_RETR_EXTERNAL, 2 for CV_RETR_LIST, 3 for CV_RETR_CCOMP, 4 for CV_RETR_TREE)
//method : Contour approximation method (Enter 1 for CV_CHAIN_APPROX_NONE, 2 for CV_CHAIN_APPROX_SIMPLE, 3 for CV_CHAIN_APPROX_TC89_L1, 4 for CV_CHAIN_APPROX_TC89_KCOS)
//point_x : x-coordinate for point offset
//point_y : y-coordinate for point offset
//
//Description
//The function retrieves contours from the images using the algorithm [Suzuki85]. The contours are a useful tool for shape analysis and object detection and recognition. 
//
//Examples
//a = imread("lena.jpeg");
//k = finContours(a,3,2,10,10);
//
//Examples
//a = imread("photo.jpeg");
//k = findContours(a,1,1,40,60);
//
//Examples
//a = imread("photo1.jpg");
//k = findContours(a,2,3,50,50);
//
//Authors
//Abhilasha Sancheti
//Shubham Lohakare
//Sukul Bagai
	inputList=mattolist(inputImage);
	outputList=raw_findContours(inputList,Mode, method, point_x, point_y)
    	for i=1:size(outputList)
       		outputMat(:,:,i)=outputList(i)
   	end
endfunction
