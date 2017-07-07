function new_image = imgaborfit(image, wavelength, orientation)
	
	image_list = mattolist(image)
	
	out = opencv_imgaborfit(image_list, wavelength, orientation)
	
	sz = size(out)
	
	for i=1:sz
		new_image(:, :, i) = out(i)
	end

endfunction
