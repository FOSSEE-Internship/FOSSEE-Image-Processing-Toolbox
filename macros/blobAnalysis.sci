// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Deepshikha
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [blob] = blobAnalysis(srcImg, varargin)
// Detects blob in the source image
//
// Calling Sequence
//   [blob] = blobAnalysis(srcImg)
//	 [blob] = blobAnalysis(srcImg, Name, Value)
//
// Parameters
// srcImg: The input image Matrix.
// ROI: This defines a particular in the image in of which you want the features.
// filterByThreshold: This filter compares the intensity of a binary image at the center of a
// blob to blobColor. If they differ, the blob is filtered out. Use blobColor = 0 to extract 
// dark blobs and blobColor = 255 to extract light blobs.
// filterByArea: Extracted blobs have an area between minArea (inclusive) and maxArea (exclusive).
// filterByCircularity: Extracted blobs have circularity ( 4∗π∗Areaperimeter∗perimeter) between
// minCircularity (inclusive) and maxCircularity (exclusive).
// filterByConvexity: Extracted blobs have convexity (area / area of blob convex hull) between minConvexity
// (inclusive) and maxConvexity (exclusive).
//
// Description
// The function uses SimpleBlobDetector function to detect the blobs then it checks for different Name 
// Value pair arguments and accordingly returns the parameters of the blob such as 2D coordinates of the
// blob, size of the blob.
//
// The Name-Value pair may be any of following types<itemizedlist><listitem><para>bool filterByArea, vector [minArea maxArea]</para></listitem><listitem><para>bool filterByCircularity, vector [minCircularity maxCircularity]</para></listitem><listitem><para>bool filterByConvexity, vector [minConvexity maxConvexity]</para></listitem><listitem><para>double ROI, vector </para></listitem><listitem><para>bool filterByThreshold, vector [minThreshold maxThreshold]</para></listitem></itemizedlist>
//
// Examples
// img_2 = imread("/images/blob.jpg", 0);
//
// lis1 = blobAnalysis(img_2, "filterByThreshold", [200, 255]);
// lis2 = blobAnalysis(img_2, "filterByCircularity", [0.1, 0.9]);
// lis3 = blobAnalysis(img_2, "filterByArea", [1500, 1600]);
//
// dimage1 = drawKeypoints(img_2, lis1.Points, "color", [0, 255, 0]);
// dimage2 = drawKeypoints(img_2, lis2.Points, "color", [0, 255, 0]);
// dimage3 = drawKeypoints(img_2, lis3.Points, "color", [0, 255, 0]);
//
// See also
// imread, drawKeypoints
//
// Authors
// Deepshikha

[lhs,rhs] = argn(0)
    
    // To check the number of input and output arguments
    
    if rhs < 1 then
         error(msprintf(" Not enough input arguments"))
    elseif rhs > 10 then
         error(msprintf(" Too many input arguments to the function"))
    elseif lhs < 1 then
         error(msprintf(" Not enough output arguments"))
    elseif lhs > 1 then
    	 error(msprintf(" Too many output arguments"))
    end
    
    srcMat = mattolist(srcImg)
    
    if modulo(rhs,2) == 0 then
    	error(msprintf("Number of input arguments must be odd"))
    end
    
    select rhs
    	case 1 then
    		output = raw_blobAnalysis(srcMat)
    		
    	case 3 then
    		if typeof(varargin(1)) <> "string"
    			error(msprintf("argument at position 2 must be string"))
    		end
    		output = raw_blobAnalysis(srcMat, varargin(1), varargin(2))
    		
    	case 5 then
    		if typeof(varargin(1)) <> "string" 
    			error(msprintf("argument at position 2 must be string"))
    		end
    		if typeof(varargin(3)) <> "string"
    			error(msprintf("argument at position 4 must be string"))
    		end
    		output = raw_blobAnalysis(srcMat, varargin(1), varargin(2), varargin(3), varargin(4))
    	
    	case 7 then
    		if typeof(varargin(1)) <> "string" 
    			error(msprintf("argument at position 2 must be string"))
    		end
    		if typeof(varargin(3)) <> "string"
    			error(msprintf("argument at position 4 must be string"))
    		end
    		if typeof(varargin(5)) <> "string" 
    			error(msprintf("argument at position 6 must be string"))
    		end
    		output = raw_blobAnalysis(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6))
    		
    	case 9 then
    		if typeof(varargin(1)) <> "string" 
    			error(msprintf("argument at position 2 must be string"))
    		end
    		if typeof(varargin(3)) <> "string"
    			error(msprintf("argument at position 4 must be string"))
    		end
    		if typeof(varargin(5)) <> "string" 
    			error(msprintf("argument at position 6 must be string"))
    		end
    		if typeof(varargin(7)) <> "string"
    			error(msprintf("argument at position 8 must be string"))
    		end
    		output = raw_blobAnalysis(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8))

        case 11 then
            if typeof(varargin(1)) <> "string" 
                error(msprintf("argument at position 2 must be string"))
            end
            if typeof(varargin(3)) <> "string"
                error(msprintf("argument at position 4 must be string"))
            end
            if typeof(varargin(5)) <> "string" 
                error(msprintf("argument at position 6 must be string"))
            end
            if typeof(varargin(7)) <> "string"
                error(msprintf("argument at position 8 must be string"))
            end
            output = raw_blobAnalysis(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9), varargin(10))
    end
    
    blob = struct("Points", output(1), "Size", output(2))
	
endfunction

