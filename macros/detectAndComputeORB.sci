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


function [orb] = detectAndComputeORB(srcImg, varargin)
// Oriented FAST and rotated BRIEF (ORB) is used to detect and compute the corners in an image. 
//
// Calling Sequence
//   [orb] = detectAndComputeORB(srcImg)
//	 [orb] = detectAndComputeORB(srcImg, Name(same as the ones given under description), Value, ...)
//
// Parameters
// srcImg: The input image Matrix
// ROI: This defines a particular in the image in of which you want the features.
// maxFeatures: The maximum number of features to retain.
// scaleFactor: Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid,
// where each next level has 4x less pixels than the previous, but such a big scale factor will 
// degrade feature matching scores dramatically. On the other hand, too close to 1 scale factor will
// mean that to cover certain scale range you will need more pyramid levels and so the speed will suffer.
// nLevels: The number of pyramid levels. The smallest level will have linear size equal to 
// input_image_linear_size / pow(scaleFactor, nLevels).
// edgeThreshold: This is size of the border where the features are not detected. It should roughly match
// the patchSize parameter.
// firstLevel: It should be 0 in the current implementation. 
// scoreType: The default HARRIS_SCORE (flag value = 0) means that Harris algorithm is used to rank features
// (the score is written to KeyPoint score and is used to retain best nfeatures features); FAST_SCORE (flag
// value = 1) is alternative value of the parameter that produces slightly less stable keypoints, but it is
// a little faster to compute.
// patchSize: Size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers
// the perceived image area covered by a feature will be larger.
// fastThreshold: The threshold value used for feature detection.
//
// Description
// Oriented FAST and rotated BRIEF (ORB) is a fast robust local feature detector, first presented
// by Ethan Rublee in  2011,that can be used in computer vision tasks like object recognition 
// or 3D reconstruction.
// It is based on the FAST keypoint detector and the visual descriptor BRIEF (Binary Robust 
// Independent Elementary Features). 
// Its aim is to provide a fast and efficient alternative to SIFT.
//
// The Name-Value pair may be any of following types<itemizedlist><listitem><para> edgeThreshold  </para></listitem><listitem><para> fastThreshold  </para></listitem><listitem><para> firstLevel     </para></listitem><listitem><para> maxFeatures    </para></listitem><listitem><para> nLevels        </para></listitem><listitem><para> patchSize      </para></listitem><listitem><para> scaleFactor    </para></listitem><listitem><para> scoreType      </para></listitem></itemizedlist>
//
// Examples
// img_1 = imread("images/checkerBoard.jpg", 0);
// img_2 = imread("images/chess.jpg", 0);
// lis1 = detectAndComputeORB(img_1);
// lis2 = detectAndComputeORB(img_2);
// dimage = drawKeypoints(img_2, lis2.Points);
// [matches, distance] = matchFeatures(lis1.Features, lis1.Features, "Method", "Exhaustive", "Metric", "Hamming");
// matchedImage = drawMatch(img_1, img_2, lis1.Points, lis2.Points, matches, distance);
//
// See also
// imread
// drawMatch
// drawKeypoints
// matchFeatures
//
// Authors
// Siddhant Narang

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
    		output = raw_detectAndComputeORB(srcMat)
    		
    	case 3 then
    		if typeof(varargin(1)) <> "string"
    			error(msprintf("argument at position 2 must be string"))
    		end
    		output = raw_detectAndComputeORB(srcMat, varargin(1), varargin(2))
    		
    	case 5 then
    		if typeof(varargin(1)) <> "string" 
    			error(msprintf("argument at position 2 must be string"))
    		end
    		if typeof(varargin(3)) <> "string"
    			error(msprintf("argument at position 4 must be string"))
    		end
    		output = raw_detectAndComputeORB(srcMat, varargin(1), varargin(2), varargin(3), varargin(4))
    	
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
    		output = raw_detectAndComputeORB(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6))
    		
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
    		output = raw_detectAndComputeORB(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8))

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
            if typeof(varargin(9)) <> "string"
                error(msprintf("argument at position 10 must be string"))
            end
            output = raw_detectAndComputeORB(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(9), varargin(10))

        case 13 then
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
            if typeof(varargin(9)) <> "string"
                error(msprintf("argument at position 10 must be string"))
            end
            if typeof(varargin(11)) <> "string"
                error(msprintf("argument at position 12 must be string"))
            end
            output = raw_detectAndComputeORB(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(9), varargin(10), varargin(11), varargin(12))

        case 15 then
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
            if typeof(varargin(9)) <> "string"
                error(msprintf("argument at position 10 must be string"))
            end
            if typeof(varargin(11)) <> "string"
                error(msprintf("argument at position 12 must be string"))
            end
            if typeof(varargin(13)) <> "string"
                error(msprintf("argument at position 14 must be string"))
            end
            output = raw_detectAndComputeORB(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(9), varargin(10), varargin(11), varargin(13), varargin(14))

        case 17 then
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
             if typeof(varargin(9)) <> "string"
                error(msprintf("argument at position 10 must be string"))
            end
            if typeof(varargin(11)) <> "string"
                error(msprintf("argument at position 12 must be string"))
            end
            if typeof(varargin(13)) <> "string"
                error(msprintf("argument at position 14 must be string"))
            end
            if typeof(varargin(15)) <> "string"
                error(msprintf("argument at position 16 must be string"))
            end
            output = raw_detectAndComputeORB(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(9), varargin(10), varargin(11), varargin(13), varargin(14), varargin(15), varargin(16))

        case 19 then
            if typeof(varargin(1))<> "string" 
                error(msprintf("argument at position 2 must be string"))
            end
            if typeof(varargin(3))<> "string"
                error(msprintf("argument at position 4 must be string"))
            end
            if typeof(varargin(5))<> "string" 
                error(msprintf("argument at position 6 must be string"))
            end
            if typeof(varargin(7))<> "string"
                error(msprintf("argument at position 8 must be string"))
            end
             if typeof(varargin(9))<> "string"
                error(msprintf("argument at position 10 must be string"))
            end
            if typeof(varargin(11))<> "string"
                error(msprintf("argument at position 12 must be string"))
            end
            if typeof(varargin(13))<> "string"
                error(msprintf("argument at position 14 must be string"))
            end
            if typeof(varargin(15))<> "string"
                error(msprintf("argument at position 16 must be string"))
            end
            if typeof(varargin(17))<> "string"
                error(msprintf("argument at position 18 must be string"))
            end
            output = raw_detectAndComputeORB(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(9), varargin(10), varargin(11), varargin(13), varargin(14), varargin(15), varargin(16), varargin(17), varargin(18))
    end
    
    orb = struct("Points", output(1), "Features", output(2))
endfunction