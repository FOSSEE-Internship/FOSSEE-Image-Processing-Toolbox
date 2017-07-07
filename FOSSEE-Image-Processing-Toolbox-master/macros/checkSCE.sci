function checkSCE(varargin)
    stErrors = varargin(1)
    
    if ~isfield(stErrors,'Camera1IntrinsicsErrors' ) | ~isfield(stErrors,'Camera1ExtrinsicsErrors' ) | ~isfield(stErrors,'Camera2IntrinsicsErrors' ) | ~isfield(stErrors, 'RotationOfCamera2Error' ) | ~isfield(stErrors, 'TransOfCamera2Error')  then
        error(msprintf("A  StereoCalibrationErrors struct expected."))
    end
    
    cam1intErr = stErrors.Camera1IntrinsicsErrors;
    cam1extErr = stErrors.Camera1ExtrinsicsErrors;
    cam2intErr = stErrors.Camera2IntrinsicsErrors;
    
    checkCIE(cam1intErr);
    checkCEE(cam1extErr);
    checkCIE(cam2intErr);
    rotErr = stErrors.RotationOfCamera2Error;
    transErr = stErrors.TransOfCamera2Error;

    if type(rotErr)~=1 | type(transErr)~=1 | ~isreal(rotErr) | ~isreal(transErr) then
        error(msprintf("A  StereoCalibrationErrors struct expected."))
    end

    [size11 size12] = size(rotErr);
    [size21 size22] = size(transErr);
    
    if size11*size12~=3 | size21*size22~=3 then
        error(msprintf("A  StereoCalibrationErrors struct expected."))        
    end        
endfunction

