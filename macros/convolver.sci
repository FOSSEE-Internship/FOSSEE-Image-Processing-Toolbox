// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Kevin George , Manoj Sree Harsha
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [output] = convolver(image1,ksize,values1,scalar)
	  //Convolves an image with the kernel.
	  //
	  //Calling Sequence
	  //out_image = convolver(in_image,ksize,kvalues,scalar)
	  //
	  //Parameters
	  //in_image : an image
	  //out_image : the output image
	  //ksize : kernel size (belongs to {3,4,5})
	  //kvalues : kernel matrix values
	  //scalar :  any floating value
	  //
	  //Description
	  //out_image = convolver(in_image,ksize,kvalues,scalar)
	  //Returns the convoluted image of the input image. It uses filter2d function to perform convolution.
	  //The convolution is with the given kernel.
	  //
	  //Examples
	  //a = imread('images/lena.jpeg')
	  //b=[1 1 1;1 1 1;1 1 1]
	  //y=convolver(a,3,b,9)
	  //imshow(y)
	  //Authors
	  //    Kevin George , Manoj Sree Harsha

	image = mattolist(image1)
	a = raw_convolver(image,ksize,values1,scalar);
	d = size(a);
	for i = 1:d
		output(:,:,i) = a(i);
	end
endfunction
