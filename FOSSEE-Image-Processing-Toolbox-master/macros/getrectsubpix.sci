function new_image = getrectsubpix(image, alpha, beta, centerX, centerY, patchType)
	
	image_list = mattolist(image)
	
	out = opencv_getrectsubpix(image_list, alpha, beta, centerX, centerY, patchType)
	
	sz = size(out)
	
	for i = 1 : sz
		new_image(:, :, i) = out(i)
	end
	
endfunction
