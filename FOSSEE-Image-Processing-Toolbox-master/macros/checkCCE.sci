function checkCCE(varargin)
    errors = varargin(1)
    
    if ~isfield(errors, 'IntrinsicsErrors' ) | ~isfield(errors, 'ExtrinsicsErrors') then
        error(msprintf("A CameraCalibrationErrors Struct expected."))        
    end
    
    checkCIE(errors.IntrinsicsErrors);
    checkCEE(errors.ExtrinsicsErrors);
endfunction

