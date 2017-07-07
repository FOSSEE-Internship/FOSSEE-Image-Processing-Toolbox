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
// Detects orb in the source image
//
// Calling Sequence
// [orb] = orbAnalysis(srcImg)
// [orb] = orbAnalysis(srcImg, Name, Value)
//
// Parameters
// srcImg: The input image Matrix 
// Name: filteration method
// Value: filteration method constraints, [1X2] vector  
// orb: stores different parameters of the orb
//
// Description
// The function uses orbDetectAndCompute function to detect the orbs then it checks for different Name Value pair arguments and accordingly returns the parameters of the orb such as 2D coordinates of the orb, size of the orb.
//
// The Name-Value pair may be any of following types<itemizedlist><listitem><para>int edgeThreshold  </para></listitem><listitem><para>int fastThreshold  </para></listitem><listitem><para>int firstLevel     </para></listitem><listitem><para>int setMaxFeatures </para></listitem><listitem><para>int nLevels        </para></listitem><listitem><para>int patchSize      </para></listitem><listitem><para>int scaleFactor    </para></listitem><listitem><para>int scoreType      </para></listitem></itemizedlist>
//
// Examples
// [srcImg] = imread('images/table.jpg');
// [orb] = orbAnalysis(srcImg);	
// [orb] = orbAnalysis(srcImg, "filterByArea", [0.01 1]);
//
// Authors
// Siddhant Narang

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
    		output = raw_detectORB(srcMat)
    		
    	case 3 then
    		if typeof(varargin(1))<> "string"
    			error(msprintf("argument at position 2 must be string"))
    		end
    		output = raw_detectORB(srcMat, varargin(1), varargin(2))
    		
    	case 5 then
    		if typeof(varargin(1))<> "string" 
    			error(msprintf("argument at position 2 must be string"))
    		end
    		if typeof(varargin(3))<> "string"
    			error(msprintf("argument at position 4 must be string"))
    		end
    		output = raw_detectORB(srcMat, varargin(1), varargin(2), varargin(3), varargin(4))
    	
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
    		output = raw_detectORB(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6))
    		
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
    		output = raw_detectORB(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8))

        case 11 then
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
            output = raw_detectORB(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(9), varargin(10))

        case 13 then
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
            output = raw_detectORB(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(9), varargin(10), varargin(11), varargin(12))

        case 15 then
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
            output = raw_detectORB(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(9), varargin(10), varargin(11), varargin(13), varargin(14))

        case 17 then
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
            output = raw_detectORB(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(9), varargin(10), varargin(11), varargin(13), varargin(14), varargin(15), varargin(16))

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
            output = raw_detectORB(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(9), varargin(10), varargin(11), varargin(13), varargin(14), varargin(15), varargin(16), varargin(17), varargin(18))
    end
    
    orb = struct("Points", output(1), "Size", output(2))
endfunction