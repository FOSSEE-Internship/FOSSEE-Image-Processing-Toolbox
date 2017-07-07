function new_image = scharr(image, ddepth, dx, dy, scale, delta)
	
	image_list = mattolist(image)
	
	out = opencv_scharr(image_list, ddepth, dx, dy, scale, delta)
	
	sz = size(out)
	
	for i=1:sz
		new_image(:, :, i) = out(i)
	end
	
endfunction
