classdef simulator
    properties(GetAccess = private, SetAccess = private)
        stereoCalibrationPath
        simDir
        
        tracks
        
        nFrames
        pRadius
        pConcentration
        pSinkingRate
        pDirection
       
        trackFilePath
        nParticles
    end
    methods 
        function this = simulator(varargin)
            [stereoCalibrationPath, simConfig, this.simDir] ...
                = validateAndParseInputs(varargin{:});
            this = setup(this, stereoCalibrationPath, simConfig);
            
        end
        
        function this = run(this)
            this.runSimulation();
            this.saveTracks();
        end
    end
    
    methods(Access = private)
        this = runSimulation(this);
        this = saveTracks(this);
        this = projectPoints(this);
    end
end
    
%% Parameter validation
function [stereoCalibrationPath, simConfig, simDir] = ...
    validateAndParseInputs(varargin)

    % Validate and parse inputs
    narginchk(3, 3);
    
    parser = inputParser;
    parser.CaseSensitive = false;

    parser.addRequired('stereoCalibrationPath', @(x)validateattributes(x, {'char'}, {'nonempty'}));
    parser.addRequired('simConfig', @(x)validateattributes(x, {'struct'}, {'nonempty'}));
    parser.addRequired('simDir', @(x)validateattributes(x, {'char'}, {'nonempty'}));
    parser.parse(varargin{:});

    stereoCalibrationPath = parser.Results.stereoCalibrationPath;
    simConfig = parser.Results.simConfig;
    simDir = parser.Results.simDir;
end