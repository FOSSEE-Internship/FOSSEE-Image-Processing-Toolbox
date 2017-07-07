function[contourMat] = imcontour(matA)
	
	srcMat = mattolist(matA)

	contourMat = opencv_imcontour(srcMat)
	
endfunction
