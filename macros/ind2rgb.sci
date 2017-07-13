function [output] = ind2rgb(srcImg,choice)
	srcMat = mattolist(srcImg)

	out = raw_ind2rgb(srcMat,choice)
	
	channel = size(out)
	
	for i = 1: channel
		output(:,:,i) = out(i)
	end
endfunction
