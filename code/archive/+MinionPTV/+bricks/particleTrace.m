classdef particleTrace
    
   properties (GetAccess = private)
        pIdx = 1;
        cIdx = 1;
   end
   properties
        predictedTrace
        detectedTrace
   end
   methods
       function obj = addPredicted(obj, centroid)
            obj.predictedTrace(obj.pIdx, :) = centroid;
            obj.pIdx = obj.pIdx + 1;
       end
       function obj = updatePredicted(obj, centroid)
           obj.predictedTrace(obj.pIdx, :) = centroid;
       end
       function obj = addDetected(obj, centroid)
            obj.detectedTrace(obj.cIdx, :) = centroid;
            obj.cIdx = obj.cIdx + 1;
       end
      function obj = updateDetected(obj, centroid)
           obj.detectedTrace(obj.pIdx, :) = centroid;
       end
   end
end
