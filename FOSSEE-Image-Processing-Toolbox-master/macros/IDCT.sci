function new_image = IDCT(image)
// Compute inverse Discrete Transform of image
//
// Calling Sequence
// newimage = IDCT(image)
//
// Parameters
// image : floating-point image
// newimage : IDCT of the input image
//
// Description
// The IDCT function computes the inverse cosine transform of a floating-point image with even number of rows and columns.
//
// Examples
// I = imread('cameraman.tif')
// I = double(I)
// J = IDCT(I)
// imshow(J)
//
// Authors
// Suraj Prakash
	
	[rows, cols, channel] = size(image);
	
	if (modulo(rows, 2) <> 0) & (modulo(cols, 2) <> 0) then
		error(msprintf("Image doesnot have even number of rows and columns\n"));
	elseif (modulo(rows, 2)) <> 0 then
		error(msprintf("Image doesnot have even number of rows\n"));
	elseif (modulo(cols, 2)) <> 0 then
		error(msprintf("Image doesnot have even number of cols\n"));
	end
	
	if channel > 1 then
		error(msprintf("Input image should be single channel"));
	end
	image_list = mattolist(image)
	
	out = opencv_IDCT(image_list)
	
	length_out = size(out)
	
	for i = 1 : length_out
		new_image(:, :, i) = out(i)
	end
	
endfunction
