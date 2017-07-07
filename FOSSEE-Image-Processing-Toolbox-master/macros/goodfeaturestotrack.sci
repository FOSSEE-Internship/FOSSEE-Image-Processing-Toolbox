function corner_points = goodfeaturestotrack(image, maxcorners, qualityLevel, minDistance, blocksize, usedetector, k)
	
	image_list = mattolist(image)
	
	corner_points = opencv_goodfeaturestotrack(image_list, maxcorners, qualityLevel, minDistance, blocksize, usedetector, k)
	
endfunction
