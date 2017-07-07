function[contrastMat] = imcontrast(srcImg, alpha, beta)
	
	srcMat = mattolist(srcImg)

	contrastMat = opencv_imcontrast(srcMat, alpha, beta)
	
endfunction
