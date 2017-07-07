function tform = affine2d(image)
	
	image_list = mattolist(image)
	
	out = opencv_affine2d(image_list)
	
	sz = size(out)
	for i=1:sz
		tform(:, :, i) = out(i)	
	end

endfunction
