function relation = crossCorrelator(inputMatrix1,inputMatrix2) 
//Calculate 2-D cross correlation of two input matrices
//
//Calling Sequence
//	relation=crossCorrelator(inputMatrix1, inputMatrix2);
//
//Parameters
// inputMatrix1: matrix of any size
// inputMatrix2: matrix of any size
//
//Description
//This function computes 2-D cross correlation of two input matrices
//
// Examples
// A=[1 4;2 3];
// B=[2 3 5;6 1 4];
// relation=crossCorrelator(A,B);

[r,c,channels1] = size(inputMatrix1);
[m,n,channels2] = size(inputMatrix2);
 if  channels1<> 1 then
        error(msprintf("wrong argument #1: cross correlator function do not support more than two dimensions"))
 elseif type(inputMatrix1)<> 1
        error(msprintf("wrong argument #1: cross correlator function do not support input type %s",typeof(A)))
 end
 if  channels2<> 1 then
        error(msprintf("wrong argument #2: cross correlator function do not support more than two dimensions"))
 elseif type(inputMatrix2)<> 1
        error(msprintf("wrong argument #2: cross correlator function do not support input type %s",typeof(B)))
 end
conjMatrix=conj(inputMatrix2);
Rep = zeros(r + m*2-2, c + n*2-2);
for x = m : m+r-1
    for y = n : n+c-1
        Rep(x,y) = inputMatrix1(x-m+1, y-n+1);
    end
end
relation = zeros(r+m-1,n+c-1);
for x = 1 : r+m-1
    for y = 1 : n+c-1
        for i = 1 : m
            for j = 1 : n
                relation(x, y) = relation(x, y) + (Rep(x+i-1, y+j-1) * conjMatrix(i, j));
            end
        end
    end
end
endfunction
