function depth = getDepth(src)
// Returns the depth of a matrix element  
//
//  Calling Sequence
//  src = imread("image-location-for-src");
//  depth = getDepth(src)
//
//Parameters
//src: Input 8-bit 3-channel image.
//depth: a string which denoted the depth of the src.It identifies the following
//
//CV_8U - 8-bit unsigned integers ( 0..255 )
//CV_8S - 8-bit signed integers ( -128..127 )
//CV_16U - 16-bit unsigned integers ( 0..65535 )
//CV_16S - 16-bit signed integers ( -32768..32767 )
//CV_32S - 32-bit signed integers ( -2147483648..2147483647 )
//CV_64F - 64-bit floating-point numbers ( -DBL_MAX..DBL_MAX, INF, NAN )
//
//
// Examples
// 
// src = imread("../images/1.jpg"); //reading an image
// depth = getDepth(src) ; //get the depth
// disp(depth) ; //view the output
//
//Note
//Scilab does not support CV_32F - it would be considered as CV_64F
//
// Authors
//  Rohan Gurve
	
	[lhs rhs]=argn(0);
    if rhs>1 //max i/p arguments is 1
        error(msprintf(" Too many input arguments"));
    elseif rhs<1 //min i/p argument is 1
        error(msprintf("input arguments missing"));
    end
    
    if lhs>1
	    error(msprintf("Too many output arguments"));
	end
	
	image_list1 = mattolist(src);
	
	d = raw_getDepth(image_list1);	
	depth = d(1);
	
endfunction


