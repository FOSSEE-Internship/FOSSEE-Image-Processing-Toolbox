function new_image = decorrstretch(image)
	
	image_list = mattolist(image)
	
	out = opencv_decorrstretch(image_list)
	
	sz = size(out)
	for i=1:sz
		new_image(:, :, i) = out(i)
	end
	
endfunction
