function [dstImg] = Deinterlacer(srcImg, varargin)
// Removes the motion antiquity from Images
//
// Calling Sequence
//   [dstImg] = Deinterlacer(srcImg)
//	 [dstImg] = Deinterlacer(srcImg, method)
//
// Parameters
// srcImg: The input Matrix  
// method: for specifying the method  used for deinterlacing
// dstimg: The output Matrix  
//
// Description
// The input is an image, srcImg, containing the motion antiquity. The varargin specifies the 2nd argument(method) for the function. If second argument i.e method is not specified then default method is used.
// The method may be any of following types :-
// <itemizedlist>
// <listitem><para>Line repeatation</para></listitem>
// <listitem><para>Linear interpolation</para></listitem>
// <listitem><para>Vertical temporal median filtering</para></listitem>
// </itemizedlist>
//
// Examples
// srcImg = imread('lena.jpeg')
// [dstImg] = Deinterlacer(srcImg) // no method specified, default method used
// [dstImg] = Deinterlacer(srcImg, "Linear interpolation") // Deinterlacing done using Linear interpolation 
//
// start of coding after on empty line to mark the end of the head_comments section
//

	[lhs,rhs]=argn(0)
	
    //To check the number of input and output arguments
    if rhs>2 then
         error(msprintf(" Too many input arguments to the function"))
    end
    
    srcMat = mattolist(srcImg)
    
    select rhs 
		case 1 then		// if no method specified then use default method	
			output = opencv_Deinterlacer(srcMat)
		case 2 then		// if method specified
			if typeof(varargin(1)) <> "string" then
				error(msprintf(" argument 2 must string"))
			end
			output = opencv_Deinterlacer(srcMat, varargin(1))
    end
	
	channels = size(output)
	
	for i = 1:channels
		dstImg(:,:,i) = output(i)
	end
	
endfunction

