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

function [indexPairs,varargout]=matchFeatures(features1,features2,varargin)
// This function is used to find the corresponding matches between two features sets
//
// Calling Sequence
//   [ indexPairs ] = matchFeatures( features1, features2, Name, Value... )
//   [ indexPairs matchMetric ] = matchFeatures( features1, features2, Name, Value... )
//
// Parameters
// features1: Feature Set 1, a set of M1-by-N Matrix, M1 corresponding to number of features and N corresponds to length of each feature vector
// features2: Feature Set 2, a set of M2-by-N Matrix, M2 corresponding to number of features and N corresponds to length of each feature vector
// Method [Optional Input Argument]: Method of matching to be used. Values: ['Exhaustive' (default) | 'Approximate']
// MatchThreshold [Optional Input Argument]: Matching Threshold for selecting the percentage of strongest matches. Values in range [0 100], default - 10.0
// Unique [Optional Input Argument]: Boolean value for selecting only unique matches between features. Default-False(0)
// Metric [Optional Input Argument]: Metric to be used for matching features. Values: ['SSD'(default)|'SAD'|'Hamming'|'Hamming_2']
// indexPairs: P-by-2 matrix containing the indices of corresponding features matched between two input feature sets
// matchMetric: P-by-1 Vector containing the distance metric between matched Features
//
// Description
// MatchFeatures function takes in the Feature Descriptors value of two images as its input and finds the best match between each feature vector of the first image to that of the second image and returns the corresponding indices of each feature matrix
//
// Examples
//stacksize("max");
//
// img_1 = imread("images/table.jpg", 0);
// img_2 = imread("images/table1.jpg", 0);
// 
// lis1 = detectSURFFeatures(img_1);
// lis2 = detectSURFFeatures(img_2);
// 
// dimage = drawKeypoints(img_2, lis2.KeyPoints);
// 
// features_1 = extractFeatures(img_1, lis1.KeyPoints, "SURFPoints", "Metric", lis1.Metric, "Orientation", lis1.Orientation, "Scale", lis1.Scale, "SignOfLaplacian", lis1.SignOfLaplacian);
// features_2 = extractFeatures(img_2, lis2.KeyPoints, "SURFPoints", "Metric", lis2.Metric, "Orientation", lis2.Orientation, "Scale", lis2.Scale, "SignOfLaplacian", lis2.SignOfLaplacian);
// 
// 
// [matches, distance] = matchFeatures(features_1.Features, features_2.Features, "Method", "Approximate");
// matchedImage = drawMatch(img_1, img_2, lis1.KeyPoints, lis2.KeyPoints, matches, distance);
// Examples
// stacksize('max');
//
// img_1 = imread("sample_image1.jpg", 0);
// img_2 = imread("sample_image2.jpg", 0);
//
// lis1 = detectBRISKFeatures(img_1);
// lis2 = detectBRISKFeatures(img_2);
//
// features_1 = extractFeatures(img_1, lis1.KeyPoints, "BRISKPoints", "Metric", lis1.Metric, "Orientation", lis1.Orientation, "Scale", lis1.Scale);
// features_2 = extractFeatures(img_2, lis2.KeyPoints, "BRISKPoints", "Metric", lis2.Metric, "Orientation", lis2.Orientation, "Scale", lis2.Scale);
//
// [matches, distance] = matchFeatures(features_1.Features, features_2.Features);
// matchedImage = drawMatch(img_1, img_2, lis1.KeyPoints, lis2.KeyPoints, matches, distance);
// 
// See also
// imread 
// extractFeatures
// drawMatch
// matches
//
// Authors
//  Umang Agrawal
//  Sridhar Reddy
//  Siddhant Narang

    [lhs rhs]=argn(0);
    if lhs>2
         error(msprintf("Too many output arguments"));
    elseif rhs>10
        error(msprintf("Too many input arguments"));
    elseif modulo(rhs, 2) <> 0
       error(msprintf("Either Argument Name or its Value missing"));
    end
    select rhs-2
        case 0 then
            [indexPairs, matchmetric]=raw_matchFeatures(features1,features2);
        case 2 then
            [indexPairs, matchmetric]=raw_matchFeatures(features1,features2,varargin(1),varargin(2));
        case 4 then
            [indexPairs, matchmetric]=raw_matchFeatures(features1,features2,varargin(1),varargin(2),varargin(3),varargin(4));
        case 6 then
            [indexPairs, matchmetric]=raw_matchFeatures(features1,features2,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6));
        case 8 then
            [indexPairs, matchmetric]=raw_matchFeatures(features1,features2,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6),varargin(7),varargin(8));
    end
    if lhs==2 then
        varargout(1)=matchmetric;
    end
endfunction
