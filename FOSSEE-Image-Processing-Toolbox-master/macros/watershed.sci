function[dstImg] = watershed(srcImg)
	
	srcMat = mattolist(srcImg)
	out = opencv_watershed(srcMat)
	
	channels = size(out)
	
	for i = 1 : channels
		dstImg(:,:,i) = out(i)
	end
endfunction
