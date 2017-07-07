function checkCEE(varargin)
    extErr = varargin(1)   
    
    if ~isfield(extErr, 'RotVectorsError') | ~isfield(extErr, 'TransVectorsError')  then
        error(msprintf("An ExtrinsicsEstimationErrors Struct expected."))
    end
    
    rvErr = extErr.RotVectorsError;
    tvErr = extErr.TransVectorsError;    

    if type(rvErr)~=1 | type(tvErr)~=1 | ~isreal(rvErr) | ~isreal(tvErr) then
        error(msprintf("An ExtrinsicsEstimationErrors Struct expected."))
    end
    
    [size11 size12] = size(rvErr);
    [size11 size12] = size(tvErr);
    
    if  size12~=3 | size11==0 | size11==0 | size12~=3 then
        error(msprintf("An ExtrinsicsEstimationErrors Struct expected."))
    end
    
endfunction
