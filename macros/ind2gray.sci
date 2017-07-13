function [output] = ind2gray(srcImg,choice)
	srcMat = mattolist(srcImg)

	out = raw_ind2gray(srcMat,choice)
	
	channel = size(out)
	
	for i = 1: channel
		output(:,:,i) = out(i)
	end
endfunction
