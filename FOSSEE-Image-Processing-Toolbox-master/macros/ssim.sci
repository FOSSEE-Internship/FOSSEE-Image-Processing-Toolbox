function[ssim_val] = ssim(srcImg, reference)

	srcMat = mattolist(srcImg)

	ssim_val = opencv_ssim(srcMat, reference)
	 
endfunction
