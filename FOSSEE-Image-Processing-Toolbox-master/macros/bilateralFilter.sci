function new_image = bilateralFilter(image, d, sigmaColor, sigmaSpace)

	image_list = mattolist(image)
	
	out = opencv_bilateralFilter(image_list, d, sigmaColor, sigmaSpace)
	
	sz = size(out)
	
	for i = 1: sz
		new_image(:, :, i) = out(i)
	end
	
endfunction
