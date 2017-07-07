function [blob] = blobAnalysis(srcImg, varargin)
// Detects blob in the source image
//
// Calling Sequence
//   [blob] = blobAnalysis(srcImg)
//	 [blob] = blobAnalysis(srcImg, Name, Value)
//
// Parameters
// srcImg: The input image Matrix 
// Name: filteration method
// Value: filteration method constraints, [1X2] vector  
// blob: stores different parameters of the blob
//
// Description
// The function uses SimpleBlobDetector function to detect the blobs then it checks for different Name Value pair arguments and accordingly returns the parameters of the blob such as 2D coordinates of the blob, size of the blob.
//
// The Name-Value pair may be any of following types :-
// <itemizedlist>
// <listitem><para>bool filterByArea, vector [minArea maxArea]</para></listitem>
// <listitem><para>bool filterByCircularity, vector [minCircularity maxCircularity]</para></listitem>
// <listitem><para>bool filterByConvexity, vector [minConvexity maxConvexity]</para></listitem>
// <listitem><para>bool filterByThreshold, vector [minThreshold maxThreshold]</para></listitem>
// </itemizedlist>
//
// Examples
// [srcImg] = imread('blobdetection.jpg');
// [blob] = blobAnalysis(srcImg);	
// [blob] = blobAnalysis(srcImg, "filterByArea", [0.01 1]);
//
// Authors
// Deepshikha

[lhs,rhs] = argn(0)
    
    // To check the number of input and output arguments
    
    if rhs<1 then
         error(msprintf(" Not enough input arguments"))
    elseif rhs>10 then
         error(msprintf(" Too many input arguments to the function"))
    elseif lhs<1 then
         error(msprintf(" Not enough output arguments"))
    elseif lhs>1 then
    	 error(msprintf(" Too many output arguments"))
    end
    
    srcMat = mattolist(srcImg)
    
    if modulo(rhs,2) == 0 then
    	error(msprintf("Number of input arguments must be odd"))
    end
    
    select rhs
    	case 1 then
    		output = opencv_blobAnalysis(srcMat)
    		
    	case 3 then
    		if typeof(varargin(1))<> "string"
    			error(msprintf("argument at position 2 must be string"))
    		end
    		output = opencv_blobAnalysis(srcMat, varargin(1), varargin(2))
    		
    	case 5 then
    		if typeof(varargin(1))<> "string" 
    			error(msprintf("argument at position 2 must be string"))
    		end
    		if typeof(varargin(3))<> "string"
    			error(msprintf("argument at position 4 must be string"))
    		end
    		output = opencv_blobAnalysis(srcMat, varargin(1), varargin(2), varargin(3), varargin(4))
    	
    	case 7 then
    		if typeof(varargin(1))<> "string" 
    			error(msprintf("argument at position 2 must be string"))
    		end
    		if typeof(varargin(3))<> "string"
    			error(msprintf("argument at position 4 must be string"))
    		end
    		if typeof(varargin(5))<> "string" 
    			error(msprintf("argument at position 6 must be string"))
    		end
    		output = opencv_blobAnalysis(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6))
    		
    	case 9 then
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
    		output = opencv_blobAnalysis(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8))
    end
    
    blob = struct("Points", output(1), "Size", output(2))
	
endfunction

