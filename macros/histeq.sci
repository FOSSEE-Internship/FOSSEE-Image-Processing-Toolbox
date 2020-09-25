// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Priyanka Hiranandani, Shubham Lohakare 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [out]=histeq(src)
//Equalizes the histogram of an image
//
//Calling Sequence
//outputImage = histeq (inputImage)
//
//Parameters
//inputImage : The input image for which the histogram has to be equalised
//
//Description
//The function equalizes the histogram of the input image using the following algorithm: 1. Calculate the histogram H for src. 2. Normalize the histogram so that the sum of histogram bins is 255. 3. Compute the integral of the histogram. 4. Transform the image using H' as a look-up table: dst(x,y) = H'(src(x,y)). The algorithm normalizes the brightness and increases the contrast of the image.
//
//Examples
//a = imread("lena.jpeg");
//k = histeq(a);
//imshow(k);
//
//Examples
//a = imread("bryan.jpg");
//k = histeq(a);
//imshow(k);
//
//Examples
//a = imread("photo.jpg");
//k = histeq(a);
//imshow(k);
//
//Authors
//Shubham Lohakare, NITK Surathkal
//Priyanka Hiranandani, NIT Surat
         srcMat=mattolist(src)
         output=raw_histeq(srcMat)
         channels = size(output)
         for i = 1: channels
              out(:,:,i)=(output(i))
         end
     	out = double(out)
endfunction;
