function[xyz] = whitepoint(input_string)

		out = whitepoint(input_string)
	
	channels = size(out)
	
	for i = 1:channels
		xyz(:,:,1) = out(1)
	end
endfunction
