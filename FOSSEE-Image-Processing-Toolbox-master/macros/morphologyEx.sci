function new_image = morphologyEx(image, operation, kernel, anchorX, anchorY, iterations, borderType)
	
		
		image_list = mattolist(image)
		
		
		out = opencv_morphologyEx(image_list, operation, kernel, anchorX, anchorY, iterations, borderType)
		
		sz = size(out)
		
		for i=1:sz
			new_image(:, :, i) = out(i)
		end

endfunction
