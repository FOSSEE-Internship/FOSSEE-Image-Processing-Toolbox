// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shubham Lohakare, Priyanka Hiranandani 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [out] = imrotate(srcImg, angle, varargin)	
//Rotates an image by the specified angle
//
//Calling Sequence
//outputImage = imrotate(srcImg, angle)
//outputImage = imrotate(srcImg, angle, BoundingBoxType)
//
//Parameters
//srcImg : The input image which is to be rotated
//angle : The angle by which the image is to be rotated
//BoundingBoxType : The type of the bounding box for the image displayed, whether a cropped one or one displaying full image.
//
//Description
//The function rotates the input image in the anticlockwise direction by the angle specified around the centre point of the image. The bounding box determines whether the complete image or a cropped image will be displayed based on the BoundingBoxType i.e. 'loose' will display the complete image and 'crop' will display the cropped image.
//
//Examples
//k = imread("lena.jpeg');
//p = imrotate(k, 45);
//imshow(p)
//
//Examples
//k = imread("lena.jpeg');
//p = imrotate(k, 30, 'loose');
//imshow(p)
//
//Examples
//k = imread("photo.jpg");
//p = imrotate(k, 60, 'crop');
//imshow(p)
//
//Authors
//Shubham Lohakare, NITK Surathkal
//Priyanka Hiranandani, NIT Surat
srcMat = mattolist(srcImg)

	[lhs, rhs] = argn(0)

	if rhs > 3 then
		error(msprintf("Too many input arguments"))
	end	
	if lhs > 1 then
		error(msprintf("Too many output arguments"))
	end		


	select rhs
		case 2 then
			output = raw_imrotate(srcMat,angle)
		case 3 then
			output = raw_imrotate(srcMat, angle,varargin(1))
	end

	channels = size(output)
	for i = 1: channels
		out(:,:,i) = (output(i))
	end
	out = double(out)

endfunction
