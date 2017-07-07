//Author: Ebey Abraham
function pcshow(mat,varargin)
	[lhs rhs] = argn(0)
	mat_size = size(mat)
	no_pts = mat_size(1)
	x = mat(1:no_pts)
	y = mat(no_pts+1:2*no_pts)
	z = mat(2*no_pts+1:3*no_pts)
	drawlater()
	param3d(x, y, z, alpha = 55, theta = 55)
	p = gce();
	p.line_mode = 'off';
	p.mark_mode = 'on';
	xgrid(color(200,200,200),1,1)
	if rhs == 2
		p.mark_style = 0
		p.thickness = 3
		if type(varargin(1)) == 10
			p.mark_foreground = color(varargin(1))
		elseif type(varargin(1)) == 1
			rgb_mat = varargin(1)
			s = size(rgb_mat)
			rows = s(1)
			cols = s(2)
			if rows == 1 & cols == 3 
				if rgb_mat >= [0,0,0] & rgb_mat <= [1,1,1]
					rgb_mat = varargin(1) * 255
					p.mark_foreground = color(rgb_mat(1),rgb_mat(2),rgb_mat(3))
				else
					error(msprintf('Element out of range 0.0 <= value <=1.0'))
				end
			elseif rows == no_pts & cols == 3
				for i=1:no_pts
					param3d(x(i), y(i), z(i), alpha = 55, theta = 55)
					r = rgb_mat(i)
					g = rgb_mat(i+rows)
					b = rgb_mat(i+2*rows)
					p.mark_foreground = color(r,g,b)
				end
			else
				error(msprintf("No of rows equal to 1 or equal to number of points."))
			end
		elseif type(varargin(1)) == 17
			i_mat = varargin(1)
			s = size(i_mat)
			size_mat = s(1) * s(2)
			if size_mat == no_pts
				for i=1:no_pts
					param3d(x(i), y(i), z(i), alpha = 55, theta = 55)
					r = i_mat(i)
					g = i_mat(i+size_mat)
					b = i_mat(i+2*size_mat)
					p.mark_foreground = color(r,g,b)
				end
			else
				error(msprintf("M * N should be equal to number of points"))
			end
		else
			error(msprintf("Invalid argument #2"))
		end
	end	 
	drawnow()
endfunction
