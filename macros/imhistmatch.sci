// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shreyash Sharma
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [outputImg]=imhistmatch(inputImage,refImage,varargin)
// This function is used to adjust histogram of 2-D image to match histogram of reference image. 
//
// Calling Sequence
// B = imhistmatch(A,ref)
// B = imhistmatch(A,ref,nbins)
// [B] = imhistmatch(___)
//
// Parameters
// A: image matrix of the source image.
// ref: image matrix of the reference image.
// B : output image with it's histogram matching similar to a given reference image. 
//
// Description
// B = imhistmatch(A,ref) transforms the 2-D grayscale or truecolor image A returning output image B whose histogram approximately matches the histogram of the reference image ref.
//
// Examples
// i = imread('color.png',0);
// i1 = imread('grey.png',0);
// i2 = imhistmatch(i,i1);
// imshow(i2);
//
     
     
     [lhs rhs]=argn(0);
    if rhs>3
        error(msprintf(" Too many input arguments"));
    end
    inputList=mattolist(inputImage);
    refList=mattolist(refImage);
    select rhs
        case 2 then
            outputList=raw_imhistmatch(inputList,refList);
        case 3 then
            outputList=raw_imhistmatch(inputList,refList,varargin(1));
   end
    for i=1:size(outputList)
        outputImg(:,:,i)=outputList(i)
    end
endfunction
