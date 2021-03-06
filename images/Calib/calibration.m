% Auto-generated by cameraCalibrator app on 08-Sep-2016
%-------------------------------------------------------


% Define images to process
imageFileNames = {'C:\Users\Ana\Documents\MATLAB\ControlEx\images\Calib\1.jpg',...
    'C:\Users\Ana\Documents\MATLAB\ControlEx\images\Calib\2.jpg',...
    'C:\Users\Ana\Documents\MATLAB\ControlEx\images\Calib\3.jpg',...
    'C:\Users\Ana\Documents\MATLAB\ControlEx\images\Calib\4.jpg',...
    'C:\Users\Ana\Documents\MATLAB\ControlEx\images\Calib\5.jpg',...
    'C:\Users\Ana\Documents\MATLAB\ControlEx\images\Calib\6.jpg',...
    'C:\Users\Ana\Documents\MATLAB\ControlEx\images\Calib\7.jpg',...
    'C:\Users\Ana\Documents\MATLAB\ControlEx\images\Calib\8.jpg',...
    'C:\Users\Ana\Documents\MATLAB\ControlEx\images\Calib\9.jpg',...
    };

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Generate world coordinates of the corners of the squares
squareSize = 25;  % in units of 'mm'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm');

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams, 'BarGraph');

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
originalImage = imread(imageFileNames{1});
undistortedImage = undistortImage(originalImage, cameraParams);

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('SparseReconstructionExample')
