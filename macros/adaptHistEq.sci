// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shubham Lohakare, Yash Balghat 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [outputImg]=adaptHistEq(inputImage,varargin)
//Performs Contrast Limited Adaptive Histogram Equalisation
//
//Calling Sequence
//outputImage = adaptHistEq(inputImage)
//outputImage = adaptHistEq(inputImage, clip_limit)
//
//Parameters
//inputImage : The input image on which histogram equalisation is to be implemented
//clip_limit : The target hgram bins.
//
//Description
//As an alternative to using histeq, you can perform contrast-limited adaptive histogram equalization (CLAHE) using the adapthisteq function. While histeq works on the entire image, adapthisteq operates on small regions in the image, called tiles. Each tile's contrast is enhanced, so that the histogram of the output region approximately matches a specified histogram. After performing the equalization, adapthisteq combines neighboring tiles using bilinear interpolation to eliminate artificially induced boundaries.
//
//Examples
//a=imread("lena.jpeg");
//k=adaptHistEq(a);
//
//Examples
//a=imread("photo.jpg");
//k=adaptHistEq(a,64);
//
//Authors
//Shubham Lohakare, NITK Surathkal
//Yash Balghat

     [lhs rhs]=argn(0);
    if rhs>2
        error(msprintf(" Too many input arguments"));
    end
    inputList=mattolist(inputImage);
    select rhs
        case 1 then
            outputList=raw_adaptHistEq(inputList);
        case 2 then
            outputList=raw_adaptHistEq(inputList,varargin(1));
   end
    for i=1:size(outputList)
        outputImg(:,:,i)=outputList(i)
    end
endfunction
