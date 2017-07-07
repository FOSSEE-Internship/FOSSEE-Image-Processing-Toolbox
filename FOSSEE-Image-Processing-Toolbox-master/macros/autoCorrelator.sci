function relation = autoCorrelator(inputMatrix)
//Calculate 2-D auto correlation of input matrix
//
//Calling Sequence
//	relation=autoCorrelator(inputMatrix)
//
//Parameters
// inputMatrix: input matrix to calculate auto correlation
//
//Description
//This function computes 2-D auto correlation of input matrix
//
// Examples
// rel = autoCorrelator([3 1;2 4]) // examples of use

    [lhs rhs]=argn(0);
    if  rhs<1 then
        error(msprintf("Wrong arguments: Need atleast one argument"))
    end
	[r,c,channels] = size(inputMatrix);
    if  channels<> 1 then
        error(msprintf("Auto correlator function do not support more than two dimensions"))
    elseif type(inputMatrix)<> 1
        error(msprintf("Auto correlator function do not support input type %s",typeof(inputMatrix)))
    end
	Rep = zeros(r + r*2-2, c + c*2-2);
	for x = r : r+r-1
		for y = c : c+c-1
		    Rep(x,y) = inputMatrix(x-r+1, y-c+1);
		end
	end
    conjMatrix=conj(inputMatrix);
    //(mROw,mCols)- It indicates middle value of the output matrix
	relation = zeros(r+r-1,c+c-1); //output matrix
    //calculating only half of the output matrix
	for x = 1 : r+r-1
		for y = 1 : c+c-1
		    for i = 1 : r
		        for j = 1 : c
		            relation(x, y) = relation(x, y) + (Rep(x+i-1, y+j-1) * conjMatrix(i, j));
		        end
		    end
		end
	end
    //Copying remainig values
endfunction
