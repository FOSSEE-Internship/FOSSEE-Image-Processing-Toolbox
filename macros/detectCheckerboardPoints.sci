// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Gursimar Singh 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [corners]=detectCheckerboardPoints(image,patternSize,varargin)
//This function detect internal corners in a checkerboard image.
//
//Calling Sequence
//corners=detectCheckerboardPoints(image,patternSize)
//corners=detectCheckerboardPoints(image,patternSize,flag) 
//corners=detectCheckerboardPoints(image,patternSize,flag,winSize) 
//corners=detectCheckerboardPoints(image,patternSize,flag,termcriteria) 
//
//Parameters
//corners:detected corners in the checkerboard.
//image:checkerBoard image which corners are to be detected.Recommended grayscale
//patternSize:It is the board size:[nrows,ncols].
//flag:It is a 1XN matrix where N ranges from 1 to 4.Default-[ 1 4 8].<itemizedlist><listitem>1 - CV_CALIB_CB_ADAPTIVE_THRESH - Use adaptive thresholding to convert the image to black and white, rather than a fixed threshold level (computed from the average image brightness).</listitem><listitem>2 - CV_CALIB_CB_NORMALIZE_IMAGE - Normalize the image gamma with equalizeHist before applying fixed or adaptive thresholding.</listitem><listitem>4 - CV_CALIB_CB_FILTER_QUADS Use additional criteria (like contour area, perimeter, square-like shape) to filter out false quads extracted at the contour retrieval stage.</listitem><listitem>8 - CALIB_CB_FAST_CHECK - Run a fast check on the image that looks for chessboard corners, and shortcut the call if none is found. This can drastically speed up the call in the degenerate condition when no chessboard is observed.</listitem></itemizedlist> 
//
//winSize:window Size for movement across the image while correcting corners.Defalt:[11,11]
//termcriteria:It is a 1X2 matrix reprenting termination ctitreia for algoritm.[maxCount,Eps].Default:[30,0.1]
//
//Description
//This function detect internal corners in a checkerboard image.Function is used in camera callibration.
//
//Examples
////The examples calculates the image and worldpoints in an image and thus determine the camera matrix and other parameters for callibration camera.
//boardCols=7;
//boardRows=10;
//checkerSize=10;
//worldPoint=genCheckerboardPoints([boardCols boardRows],checkerSize);
//image=imread("images/checkerboard.jpg",0)
//corners=detectCheckerboardPoints(image,[boardRows,boardCols]);
//param=calibrateCamera(corners,worldPoint)
//
//See also
//genCheckerboardPoints
//
//Authors
//Gursimar Singh

image_list = mattolist(image)

[lhs,rhs]=argn(0);

	if lhs>1
    	error(msprintf(" Too many output arguments"));
	elseif rhs>5
    	error(msprintf(" Too many input arguments"));
	elseif rhs<2
   		error(msprintf("Too less arguments provided!,minimum is 2!"));
	end

	if rhs==2 then
		corners = raw_detectCheckerboardPoints(image_list,patternSize);
	end
	if rhs==3 then
		corners = raw_detectCheckerboardPoints(image_list,patternSize,varargin(1));
	end
	if rhs==3 then
		corners = raw_detectCheckerboardPoints(image_list,patternSize,varargin(1),varargin(2));
	end
	if rhs==5 then
		corners = raw_detectCheckerboardPoints(image_list,patternSize,varargin(1),varargin(2),varargin(3));
	end

endfunction
