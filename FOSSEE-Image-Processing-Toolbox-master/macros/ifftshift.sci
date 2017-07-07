function new_image = ifftshift(image)
	
	image_list = mattolist(image)
	
	out = opencv_ifftshift(image_list)
	
	sz = size(out)
	for i = 1 : sz
		new_image(:, :, i) = out(i)
	end	
	
endfunction
