function new_image = TemplateMatcher(image, template_image, varargin)
// Compares a template in overlapped image regions
//
// Calling Sequence
// newimage = TemplateMatcher(image, template_image)
// 
// Parameters
// image : 8-bit or 32-bit floating image
// template_image : Searched template. It must not be greater than the source image and have the same data type of input image.
// newimage : Map of comparison results. It must be single channel 32-bit floating-point. Its size is (W-w+1) * (H-h+1) if image is W * H and template_image is w * h
//
// Description
// The function compares the w * h sized images of the original image against the template image and stores the result in the new_image.
//
// Examples
// I = imread('cameraman.tif');
// Igray = rgb2gray(I);
// T = Igray(20:75, 90:135);
// J = TemplateImage(I, T);
// imshow(J);
//
// Authors
// Suraj Prakash


		[ lhs rhs ] = argn(0)
		if lhs > 1 then
			error(msprintf("Too many output argument"))
		end
		
		if rhs > 2 then
			error(msprintf("Too many input arguments"))
		end	
		
		[imagerows imagecols imagechannel] = size(image)
		[t_rows t_cols t_channel] = size(template_image)
		if t_rows > imagerows | t_cols > imagecols then
			error(msprintf("Template image is greater than image\n"))
		end
		
		image_list = mattolist(image)
		template_image_list = mattolist(template_image)
		
		if rhs == 2 then
			
			temp = opencv_TemplateMatcher(image_list, template_image_list)
		
		elseif rhs == 3 then
			
			temp = opencv_TemplateMatcher(image_list, template_image_list, varargin(1))
		
		end
		
		sz = size(temp)
		
		for i=1 : sz
			new_image(:, :, i) = temp(i)
		end
		
endfunction
