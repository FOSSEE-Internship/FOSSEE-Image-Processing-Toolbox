// Function Name = 2-D convolution matrix
// Output = Convolution matrix T
// Input = Input matrix srcMat, Dimensions [m, n]

function[T] = convmtx2(srcImg, m, n)
	
	srcMat = mattolist(srcImg)

	out = opencv_convmtx2(srcMat, m, n)
	
	channels = size(out)
	
	for i = 1:channels
		T(:,:,i) = out(i)
	end
	
endfunction
