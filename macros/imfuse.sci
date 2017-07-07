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

function new_image = imfuse(image1, image2, method, scaling)
// This function is used to calculate Composite of two images. 
//
// Calling Sequence
// C = imfuse(A,B,C,D)
//
// Parameters
// A: Image to be combined into a composite image, specified as a grayscale, truecolor, or binary image.
// B: Image to be combined into a composite image, specified as a grayscale, truecolor, or binary image.
// C,D: Name-Value Pair Arguments.Specify optional comma-separated pairs of Name,Value arguments. Name is the argument name and Value is the corresponding value. Name and value must appear inside single quotes (' '). You can specify several name and value pair arguments in any order as Name1,Value1,...,NameN,ValueN. eg.'Scaling','joint' scales the intensity values of A and B together as a single data set. 
//
// Description
// C = imfuse(A,B) creates a composite image from two images, A and B. If A and B are different sizes, imfuse pads the smaller dimensions with zeros so that both images are the same size before 
//     creating the composite. The output, C, is a numeric matrix containing a fused version of images A and B.
//
// Examples
// i = imread('lena.jpeg');
// i1 = imread('lena2.jpg');
// i2 = imfuse(i,i1,"blend","joint");
// imshow(i2);
//
	
	image_list1 = mattolist(image1)
	image_list2 = mattolist(image2)
	out = raw_imfuse(image_list1, image_list2, method, scaling)
	
	sz = size(out)
	
	for i=1:sz
		new_image(:, :, i) = out(i)
	end
	
endfunction
