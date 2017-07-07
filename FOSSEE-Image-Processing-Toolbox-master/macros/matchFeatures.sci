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
//    image_1 = imread('sample1.jpg');
//    image_2 = imread('sample2.jpg');
//    points_1 = detectFASTFeatures(image_1);
//    points_2 = detectFASTFeatures(image_2);
//    [ f1 vpts_1 ] = extractFeatures(image_1, points_1);
//    [ f2 vpts_2 ] = extractFeatures(image_2, points_2);
//    indexPairs = matchFeatures(f1, f2);
//    matchedPoints_1 = vpts_1(indexPairs(:,1));
//    matchedPoints_2 = vpts_2(indexPairs(:,2));
//
// With Optional Arguments:
//    [indexPairs matchMetric] = matchFeatures(f1, f2, "Metric", "SAD");
//
// Authors
//  Umang Agrawal
//  Sridhar Reddy

    [lhs rhs]=argn(0);
    if lhs>2
         error(msprintf(" Too many output arguments"));
    elseif rhs>10
        error(msprintf(" Too many input arguments"));
    elseif modulo(rhs,2)<>0
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
