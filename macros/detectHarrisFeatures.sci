// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Siddhant Narang
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [cornerPoints] = detectHarrisFeatures(image,varargin)
// This function is used to find corner points in an image using Harris algorithm.
//
// Calling Sequence
// points = detectHarrisFeatures(I);
// points = detectHarrisFeatures(I, Name, Value, ...);
//
// Parameters
// points: Structure of corner points
// I: Input image to detectHarrisFeatures()
// MinQuality: (Optional) Minimum accepted quality of corners (Default- 0.01)
// FilterSize: (Optional) Dimension of Gaussian Filter (Default: 5)
// ROI: (Optional) Rectangular region for corner detection
// SensitivityFactor: (Optional) SensitivityFactor of Harris algorithm (Default- 0.04)
//
// Description
// This function detects corners in an image I. These corner points are used to extract features and hence recognize the contents of an image.
//
// Examples
// img_1 = imread("images/checkerBoard.jpg", 0);
// lis1 = detectHarrisFeatures(img_1);
// dimage = drawKeypoints(img_1, lis1.Location, "color", [0, 255, 0]);
//
// See also
// imread
// drawKeypoints
//
// Authors
// Rohit Suri
// Sridhar Reddy
// Siddhant Narang

    [lhs rhs]=argn(0);
    if lhs>1
         error(msprintf("Too many output arguments"));
    elseif rhs>9
        error(msprintf("Too many input arguments"));
    elseif modulo(rhs, 2) == 0
       error(msprintf("Either Argument Name or its Value missing"));
    end
    imageList=mattolist(image);
    select rhs-1
        case 0 then
            [location metric count]=ocv_detectHarrisFeatures(imageList);
            //[location metric]=ocv_detectHarrisFeatures(imageList);
        case 2 then
            [location metric count]=ocv_detectHarrisFeatures(imageList,varargin(1),varargin(2));
        case 4 then
            [location metric count]=ocv_detectHarrisFeatures(imageList,varargin(1),varargin(2),varargin(3),varargin(4));
        case 6 then
            [location metric count]=ocv_detectHarrisFeatures(imageList,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6));
        case 8 then
            [location metric count]=ocv_detectHarrisFeatures(imageList,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6),varargin(7),varargin(8));
    end
    //cornerPoints=struct('Type','cornerPoints','Location',location,'Metric',metric,'Count',count);
    cornerPoints=struct('Location',location, 'Metric', metric);
endfunction
