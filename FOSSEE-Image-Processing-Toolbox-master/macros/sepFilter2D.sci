function new_image = sepFilter2D(image, ddepth, kernel_x, kernel_y, anchor_x, anchor_y, delta, border)
	
	image_list = mattolist(image)
	
	out = opencv_sepFilter2D(image_list, ddepth, kernel_x, kernel_y, anchor_x, anchor_y, delta, border)
	
	sz = size(out)
	
	for i = 1 : sz
		new_image(:, :, i) = out(i)
	end
	
endfunction
