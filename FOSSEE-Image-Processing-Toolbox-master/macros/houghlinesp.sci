function output = houghlinesp(image, rho, theta, threshold, minLineLength, maxLineGap)
	
	image_list = mattolist(image)
	
	output = opencv_houghlinesp(image_list, rho, theta, threshold, minLineLength, maxLineGap)
	
endfunction
