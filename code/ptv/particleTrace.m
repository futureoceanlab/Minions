classdef particleTrace
    
   properties (GetAccess = private)
        pIdx = 1;
        dIdx2D = 0;
        dIdx3D = 1;
   end
   properties
        predictedTrace
        detectedTrace2D
        detectedTrace3D
   end
   methods
       function obj = addPredicted(obj, centroid)
            obj.predictedTrace(obj.pIdx, :) = centroid;
            obj.pIdx = obj.pIdx + 1;
       end
       function obj = updatePredicted(obj, centroid)
           obj.predictedTrace(obj.pIdx, :) = centroid;
       end
       function obj = addDetected2D(obj, centroid2D)
           obj.dIdx2D = obj.dIdx2D + 1;
            obj.detectedTrace2D(obj.dIdx2D, :) = centroid2D;
       end
      function obj = updateDetected2D(obj, centroid2D)
           obj.detectedTrace2D(obj.dIdx2D, :) = centroid2D;
      end
       function obj = addDetected3D(obj, centroid3D)
            obj.detectedTrace3D(obj.dIdx2D, :) = centroid3D;
%             obj.dIdx3D = obj.dIdx3D + 1;
       end
     function obj = updateDetected3D(obj, centroid3D)
           obj.detectedTrace3D(obj.dIdx2D, :) = centroid3D;
      end
   end
end
