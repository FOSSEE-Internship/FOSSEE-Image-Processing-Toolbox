function new_image = roifill(image, mask_orig, varargin)
	
	[ lhs, rhs ] = argn(0)
	
	if rhs < 2 then
		error(msprintf("Too less input arguments"))
	elseif rhs > 3 then
		error(msprintf("Too many input arguments"))
	end
	
	
	image_list = mattolist(image)
	
	
	select rhs
		case 2
			out = opencv_roifill(image_list, mask_orig)
		
		case 3
			out = opencv_roifill(image_list, mask_orig, varargin(1))
	end
	
	sz = size(out)
	for i=1 : sz
		new_image(:, :, i) = out(i)
	end 
	
endfunction
