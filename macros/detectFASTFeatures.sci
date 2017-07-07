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

function [cornerPoints] = detectFASTFeatures(image,varargin)
// This function is used to detect the corner points using FAST Alogrithm
//
// Calling Sequence
//   [ Location Count Metric ] = detectFASTFeatures( Image, Name, Value... )
//
// Parameters
//  Image: Input Image, should be a 2-D grayscale. The Input Image should be real
//  MinQuality [Optional Input Argument]: Minimum Accepted Quality of Corners, can be specified as a scalar value between [0,1]. Default: 0.1
//  MinContrast [Optional Input Argument]: Minimum Intensity difference for Corners to be detected, can be specified as a scalar value between[0,1]. Default: 0.2
//  ROI [Optional Input Argument]: Specify a rectangular region of operation. Format [ x y width height ]. Default: [1 1 size(Image,2) size(Image,1)]
//  Location: Set of x,y coordinates for the deteccted points
//  Count: Number of corner points detected
//  Metric: Value describing the strength of each detected Point
//
// Description
//  The detectFASTFeatures function uses the Features from Accelerated Segment Test (FAST) algorithm to find feature points.
//
// Examples
// stacksize("max");
// img_1 = imread("images/table.jpg", 0);
// img_2 = imread("images/table1.jpg", 0);
// lis1 = detectFASTFeatures(img_1);
// lis2 = detectFASTFeatures(img_2);
// features_1 = extractFeatures(img_1, lis1.Location, "cornerPoints", "Metric", lis1.Metric);
// features_2 = extractFeatures(img_2, lis2.Location, "cornerPoints", "Metric", lis2.Metric)
// dimage = drawKeypoints(img_1, lis1.Location);
// [matches, distance] = matchFeatures(features_1.Features, features_2.Features);
// matchedImage = drawMatch(img_1, img_2, lis1.Location, lis2.Location, matches, distance);
//
// Examples
// stacksize("max");
// img_1 = imread("sample_image1.jpg", 0);
// img_2 = imread("sample_image2.jpg", 0);
// lis1 = detectFASTFeatures(img_1, "MinConstrast", 0.2);
// lis2 = detectFASTFeatures(img_2, "MinConstrast", 0.2);
// features_1 = extractFeatures(img_1, lis1.Location, "cornerPoints", "Metric", lis1.Metric);
// features_2 = extractFeatures(img_2, lis2.Location, "cornerPoints", "Metric", lis2.Metric)
// dimage = drawKeypoints(img_1, lis1.Location);
// [matches, distance] = matchFeatures(features_1.Features, features_2.Features);
// matchedImage = drawMatch(img_1, img_2, lis1.Location, lis2.Location, matches, distance);
//
// See also
// imread
// drawMatch
// drawKeypoints
// matchFeatures
//
// Authors
// Umang Agrawal
// Sridhar Reddy
// Siddhant Narang

    [lhs rhs]=argn(0);
    if lhs>3
         error(msprintf(" Too many output arguments"));
    elseif rhs-1>6
        error(msprintf(" Too many input arguments"));
    elseif modulo(rhs-1,2)<>0
       error(msprintf("Either Argument Name or its Value missing"));
    end
    imageList=mattolist(image);
    select rhs-1
        case 0 then
            [location count metric]=ocv_detectFASTFeatures(imageList);
        case 2 then
            [location count metric]=ocv_detectFASTFeatures(imageList,varargin(1),varargin(2));
        case 4 then
            [location count metric]=ocv_detectFASTFeatures(imageList,varargin(1),varargin(2),varargin(3),varargin(4));
        case 6 then
            [location count metric]=ocv_detectFASTFeatures(imageList,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6));
    end
    cornerPoints=struct('Type','cornerPoints','Location',location,'Metric',metric,'Count',count);
endfunction
