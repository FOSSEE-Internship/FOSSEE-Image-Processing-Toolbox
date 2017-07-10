// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author:  Dhruti Shah , Manoj Sree Harsha
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [] = imattributes(image)
	  //Provides information about image attributes.
	  //
	  //Calling Sequence
	  //inputImage=imread('path of the image file')
	  //imattributes(inputImage)
	  //
	  //Parameters
	  //inputImage : an image.
	  //
	  //Description
	  //imattributes gives image attribute information in the form of a 4-by-2 or 6-by-2 cell array, depending on the image type.
	  //The first column of the cell array contains the name of the attribute.
	  // The second column contains the value of the attribute. Both attribute names and values are character vectors
      //'Minimum intensity': Regarding intensity images, this value represents the lowest intensity value of any pixel. In case ofindexed images, this value represents the lowest index value into a color map.
      //'Maximum intensity':Regarding intensity images, this value represents the highest intensity value of any pixel. In case of indexed images, this value represents the highest index value into a color map.
	  //
	  //Examples
	  //inputImage=imread('images/lena.jpeg');
	  //imattributes(inputImage)
	  //Authors
	  //    Dhruti Shah , Manoj Sree Harsha

img=mattolist(image);
raw_imattributes(img);
endfunction
