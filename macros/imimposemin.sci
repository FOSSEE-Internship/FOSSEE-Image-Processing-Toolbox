// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Gursimar Singh,Vinay Bhat
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [dstMat] = imimposemin(maskImg, markerImg)
// This fucntion is used to impose minima.
//
// Calling Sequence
// dstMat = imimposemin(maskImg,markerImg)
//
// Parameters
// maskImg: The source image must be in grayscale.
// markerImg:  BW is a binary image the same size as I.
// dstMat: Transformed image, returned as a nonsparse numeric array of any class, the same size as I. 
//
// Description
// dstMat = imimposemin(maskImg,markerImg) modifies the intensity image maskImg using morphological reconstruction so it only has regional minima wherever markerImg is nonzero. markerImg is a binary image the same size as maskImg.
//
//Examples
//im=imread("images/lena.jpeg",0);
//marker = zeros(im);
//marker(2:100,2:100) = 255;
//im2=imimposemin(im,marker);
//imshow(im2)
//
//Author
//Vinay Bhat
//

	maskImg_list=mattolist(maskImg);
	sz=size(maskImg_list);
    if sz >=3 then
       error(msprintf("Input image must be grayscale"));
    end   
	markerImg_list=mattolist(markerImg);

	out = raw_imimposemin(maskImg_list, markerImg_list)
	
	channel = size(out)
	
	for i = 1: channel
		dstMat(:,:,i) = out(i)
	end
	
endfunction
