function[cropImgs] = imcrop(srcImg, xcoor, ycoor, width, height)
	srcMat = mattolist(srcImg)
//	disp(srcMat)
	cropImg = raw_imcrop(srcMat, xcoor, ycoor, width, height)
        channel = size(cropImg)          // for converting to hyper matrix
	
	for i = 1: channel
		cropImgs(:,:,i) = (cropImg(i))
	end
	cropImgs=double(cropImgs)
endfunction
