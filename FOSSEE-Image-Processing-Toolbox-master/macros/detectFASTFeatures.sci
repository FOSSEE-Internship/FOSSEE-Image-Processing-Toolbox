function [cornerPoints]=detectAgastFeatures(image,varargin)
// This function is used to detect the corner points using FAST Alogrithm
//
// Calling Sequence
//   [ Location Count Metric ] = detectFASTFeatures( Image, Name, Value... )
//
// Parameters
//  Image: Input Image, should be a 2-D grayscale. The Input Image should be real
//  MinQuality [Optional Input Argument]: Minimum Accepted Quality of Corners, can be specified as a scalar value between [0,1]. Default: 0.1
//  MinContrast [Optional Input Argument]: Minimum Intensity difference for Corners to be detected, can be specified as a scalar value between[0,1]. Default: 0.2
//  type [Optional Input Argument]: specifies the type of agast algorithm u want use 
//  ROI [Optional Input Argument]: Specify a rectangular region of operation. Format [ x y width height ]. Default: [1 1 size(Image,2) size(Image,1)]
//  Location: Set of x,y coordinates for the deteccted points
//  Count: Number of corner points detected
//  Metric: Value describing the strength of each detected Point
//
// Description
//  The detectAgastFeatures function uses the Features from Accelerated Segment Test (FAST) algorithm to find feature points.
//
// Examples
//    image = imread('sample.jpg');
//    [location count metric] = detectAgastFeatures(image);
//
// With Optional Arguments:
//    [location count metric] = detectAgastFeatures(image,"MinContrast",0.2);
//
// Authors
//  Umang Agrawal
//  Sridhar Reddy

    [lhs rhs]=argn(0);
    if lhs>3
         error(msprintf(" Too many output arguments"));
    elseif rhs-1>8
        error(msprintf(" Too many input arguments"));
    elseif modulo(rhs-1,2)<>0
       error(msprintf("Either Argument Name or its Value missing"));
    end
    imageList=mattolist(image);
    select rhs-1
        case 0 then
            [location count metric]=raw_detectAgastFeatures(imageList);
        case 2 then
            [location count metric]=raw_detectAgastFeatures(imageList,varargin(1),varargin(2));
        case 4 then
            [location count metric]=raw_detectAgastFeatures(imageList,varargin(1),varargin(2),varargin(3),varargin(4));
        case 6 then
            [location count metric]=raw_detectAgastFeatures(imageList,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6));
        case 8 then    
            [location count metric]=raw_detectAgastFeatures(imageList,varargin(1),varargin(2),varargin(3),varargin(4),varargin(5),varargin(6),varargin(7),varargin(8));
    end
    cornerPoints=struct('Type','cornerPoints','Location',location,'Metric',metric,'Count',count);
    //for i=1:count
      //  cornerPoints(i)=struct('Location',location(i,:),'metric',metric(i,:),'Count',1);
    //end
endfunction
