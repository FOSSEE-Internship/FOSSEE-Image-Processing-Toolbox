function [out]=minimumenclosingcirlce(InputArraypoints)
           [t1 t2 t3]= opencv_minimumenclosingcirlce(InputArraypoints);
           out=struct("centerx",t1,"centery",t2,"radius",t3);
endfunction;
