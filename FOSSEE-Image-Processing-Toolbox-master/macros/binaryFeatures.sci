function [features]=binaryFeatures(featureVectors)
// Object for storing Feature Vectors
//
// Calling Sequence
//   features = binaryFeatures(featureVector);
//
// Parameters
//  featureVector: M-by-N matrix consisting of M features stored in N uint8 containers
//  features: Binary Feature object for the extracted Features
//
// Description
//  This object provides the ability to pass data between extractFeatures and matchFeatures function
//
// Examples
//    image_1 = imread('sample1.jpg');
//    points_1 = detectFASTFeatures(image_1);
//    [ f1 vpts_1 ] = extractFeatures(image_1, points_1);
//    features1 = binaryFeatures(f1);
//
// Authors
//  Umang Agrawal
//  Sridhar Reddy

    [lhs rhs]=argn(0);
    if lhs>1
         error(msprintf(" Too many output arguments"));
    elseif rhs>1
        error(msprintf(" Too many input arguments"));
    elseif inttype(featureVectors) <> 11 then
        error(msprintf("wrong argument #%d: FeatureVectors must be uint8",1));
    end
    [rows cols]=size(featureVectors);
    features=struct('Type','binaryFeatures','Features',featureVectors,'NumBits',cols*8,'NumFeatures',rows);
endfunction
