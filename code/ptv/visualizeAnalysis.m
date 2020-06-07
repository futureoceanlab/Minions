function visualizeAnalysis(analysis)
    bins = 1:analysis.nBins;
    %% Concentration Outcome
    figure; 
    bar(bins', analysis.concentrationBinCounts./analysis.observedVolume);
    
    %% Speed Calculation

    figure;
    hold on;
    errLow = analysis.avgSpeeds - analysis.stdSpeeds;
    errHigh = analysis.avgSpeeds + analysis.stdSpeeds;
    bar(bins', analysis.avgSpeeds);
    
    er = errorbar(bins, analysis.avgSpeeds, errLow, errHigh);    
    er.Color = [0 0 0];                            
    er.LineStyle = 'none';  
 
    hold off
end