function[dstImg] = bwulterode(srcImg)
	
	srcMat = mattolist(srcImg)

	out = opencv_bwulterode(srcMat)
	
	channels = size(out)
	
	for i = 1:channels
		dstImg(:,:,i) = out(i)
	end
	
endfunction
