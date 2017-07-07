function[dstMat] = imextendedmin(srcImg, Hmin)

	srcMat = mattolist(srcImg)

	out = opencv_imextendedmin(srcMat, Hmin)
	
	channel = size(out)
	
	for i = 1: channel
		dstMat(:,:,i) = out(i)
	end
	
endfunction
