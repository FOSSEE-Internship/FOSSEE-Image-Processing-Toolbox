// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Priyanka Hiranandani 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function new_image = bilateralFilter(image, d, sigmaColor, sigmaSpace)
	//Applies the bilateral filter to an image 
	//
	//Calling Sequence
	//image = imread('path of image')
	//out = bilateralFilter(image, d, sigmaColor, sigmaSpace)
	//
	//Parameters
	//image : source image 
	//d : Diameter of each pixel neighborhood that is used during filtering
	//sigmaColor : Filter sigma in the color space
	//sigmaSpace : Filter sigma in the coordinate space
	//
	//Description
	//bilateralFilter can reduce unwanted noise very well while keeping edges fairly sharp.
	//A larger value of the sigmaColor parameter means that farther colors within the pixel neighborhood will be mixed together.
	//A larger value of the sigmaSpace parameter means that farther pixels will influence each other as long as their colors are close enough.	
	//It is recommended to use d=5 for real-time applications.
	//For simplicity, you can set the 2 sigma values to be the same.
	//
	//Examples
	//img=imread('images/lena.jpeg');
	//d=5;
	//sigmaColor=150;
	//sigmaSpace=150;
	//out=bilateralFilter(image, d, sigmaColor, sigmaSpace);
	//Authors
	//    Priyanka Hiranandani

	image_list = mattolist(image)
	
	out = raw_bilateralfilter(image_list, d, sigmaColor, sigmaSpace)
	
	sz = size(out)
	
	for i = 1: sz
		new_image(:, :, i) = out(i)
	end
	new_image=double(new_image)
endfunction
