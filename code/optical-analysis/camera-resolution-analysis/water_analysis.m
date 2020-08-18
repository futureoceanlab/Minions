% water_analysis
% Measure and compare the contrast of the target located at different
% region of the imaging frame in water and in air.
% Due to lack of space, we only include data for region 1

trial = 1:9;
c_water = [386 312; 1275 308; 2135 423; 408 944; 1299 983; 2156 976; 428 1537; 1282 1616; 2164 1532;];
c_air_0 = [318, 330; 1279, 325; 2247, 331; 328, 993; 1287, 997; 2312, 995; 364, 1548; 1267, 1550; 2213, 1554;];
lineStyleOrder = ["-", "--", "-."];

for t=1 %trial
    dir_water = sprintf('data/water/%d.png', t);
    dir_air0 = sprintf('data/air/15410110-%d.png', t);

    target_water = (rgb2gray(imread(dir_water)));
    target_air0 = (rgb2gray(imread(dir_air0)));
    
    [cont_water, res_water] = mtf(target_water, c_water(t, 1), c_water(t, 2), 9);
    [cont_air_0, res_air_0] = mtf(target_air0, c_air_0(t, 1), c_air_0(t, 2), 5);
    figure; 
    hold on;
    plot(res_water*1000, cont_water*100, lineStyleOrder(1), 'LineWidth', 2);
    plot(res_air_0*1000, cont_air_0*100, lineStyleOrder(2), 'LineWidth', 2);
    title(sprintf("Contrast vs Resolution (Region - %d)", t), 'FontSize', 12);
    ylabel('Contrast, %');
    xlabel(['Resolution, ',char(181),'m/px'],'Interpreter','tex');
    xlim([0 400]);
    ylim([0 100]);
    legend(["Water", "Air"], 'FontSize', 12);
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    grid on;
    hold off;

end
