// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shubham Lohakare, Ashish Manatosh Barik 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [outImg1, outImg2] = imdecolor(srcImg1)
//Decolors and color boosts an image giving 2 outputs for the same respectively.
//
//Calling Sequence
//[outImg1, outImg2] = imdecolor(srcImg1)
//
//Parameters
//srcImg1 : It is a 3-channel input image. 
//outImg1 : It is a grayscale image of the image passed as input.
//outImg2 : It is a color boosted image of the image passed as input.
//
//Description
//This function is used to decolor an image and also add a boost to it's color
//The output are 2 images, one is a grayscale and the other is a color boosted image
//
//Examples	
//var=imread("ImageName");
//[decoloredImage,colorBoostedImage] = imdecolor(var);
//PRESS ENTER
//imshow(decoloredImage); shows the decolored Image
//imshow(colorBoostedImage); shows the color boosted image
//
//Examples
//a = imread("images.jpeg");
//[b, c] = imdecolor(a);
//PRESS ENTER
//imshow(b); shows the decolored Image
//imshow(c); shows the color boosted image
//
//Authors
//Shubham Lohakare, NITK Surathkal
//Ashish Mantosh, NIT Rourkela
	srcMat1 = mattolist(srcImg1)
	

	[out1, out2] = raw_imdecolor(srcMat1)

	channels1 = size(out1)
	channels2 = size(out2)

	for i = 1:channels1
		outImg1(:, :, i) = (out1(i))
	end

	for j = 1:channels2
		outImg2(:, :, j) = (out2(j))
	end

endfunction
