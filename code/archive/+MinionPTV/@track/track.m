classdef track
    properties
        
    end
    methods
        function this = track(varargin)
            this.setup();
            this.mainProcess();        
        end
    end
    methods(Access = private)
        this = setup(this);
        this = mainProcess(this);
    end
end
