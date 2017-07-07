function[dstMat] = imfill(srcImg, Hmin)
	
	[lhs, rhs] = argn(0)
	
	srcMat = mattolist(srcImg)

	out = opencv_imfill(srcMat, Hmin)
	
	channel = size(out)
	
	for i = 1: channel
		dstMat(:,:,i) = out(i)
	end
	
endfunction
