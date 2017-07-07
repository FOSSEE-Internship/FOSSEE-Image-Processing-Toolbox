function checkCIE(varargin)
    intErr = varargin(1)   
    if ~isfield(intErr, 'SkewError') | ~isfield(intErr,'FocalLengthError' ) | ~isfield(intErr, 'PrincipalPointError') | ~isfield(intErr, 'RadDistortionError') | ~isfield(intErr, 'TanDistortionError' ) then
         error(msprintf("An IntrinsicsEstimationErrors Struct expected."))
    end
    
    skewErr  = intErr.SkewError;
    flErr = intErr.FocalLengthError;
    ppErr = intErr.PrincipalPointError;
    rdErr = intErr.RadDistortionError;
    tdErr = intErr.TanDistortionError;
    
    if type(skewErr)~=1 | type(flErr)~=1 | type(ppErr)~=1 | type(rdErr)~=1 | type(tdErr)~=1 | ~isreal(skewErr) | ~isreal(flErr) | ~isreal(ppErr) | ~isreal(rdErr) | ~isreal(tdErr)  then
        error(msprintf("An IntrinsicsEstimationErrors Struct expected."))
    end
    
    [size11 size12] = size(skewErr);
    [size21 size22] = size(flErr);
    [size31 size32] = size(ppErr);
    [size41 size42] = size(rdErr);
    [size51 size52] = size(tdErr);   
        
    if  (size11*size12)~=1 | (size21*size22)~=2 | (size31*size32)~=2 | (size41*size42)<2 | (size41*size42)>3 | (size51*size52)~=2 then
        error(msprintf("An IntrinsicsEstimationErrors Struct expected."))
    end
       
endfunction
