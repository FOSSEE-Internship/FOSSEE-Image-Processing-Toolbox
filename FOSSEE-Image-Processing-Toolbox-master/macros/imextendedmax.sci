function[dstMat] = imextendedmax(srcImg, Hmax)

	srcMat = mattolist(srcImg)

	out = opencv_imfill(srcMat, Hmax)
	
	channel = size(out)
	
	for i = 1: channel
		dstMat(:,:,i) = out(i)
	end
	
endfunction
