trial = 1:9;
%338 1019; 2253 1031; 1265 1028; 
% c_water = [282, 292; 1368, 299; 2299, 304; 335, 906; 1288, 934; 2180, 994; 493, 1517; 1318, 1524; 2291, 1532];
c_water = [386 312; 1275 308; 2135 423; 408 944; 1299 983; 2156 976; 428 1537; 1282 1616; 2164 1532;];
c_air_0 = [318, 330; 1279, 325; 2247, 331; 328, 993; 1287, 997; 2312, 995; 364, 1548; 1267, 1550; 2213, 1554;];
c_air_1 = [317, 337; 1294, 339; 2234, 344; 316, 948; 1286, 952; 2202, 954; 392, 1614; 1322, 1616;  2242, 1619;];
lineStyleOrder = ["-", "--", "-."];

for t=1 %trial
    dir_water = sprintf('data/water/%d.png', t);

%     dir_water = sprintf('data/water_edmund56/15410110-%d.png', t);
    dir_air0 = sprintf('data/15410110_Edmund56/15410110-%d.png', t);
%     dir_air1 = sprintf('data/41810422_Edmund56/41810422-%d.png', t);

    target_water = (rgb2gray(imread(dir_water)));
    target_air0 = (rgb2gray(imread(dir_air0)));
%     target_air1 = (rgb2gray(imread(dir_air1)));
    
    [cont_water, res_water] = mtf(target_water, c_water(t, 1), c_water(t, 2), 9);
    [cont_air_0, res_air_0] = mtf(target_air0, c_air_0(t, 1), c_air_0(t, 2), 5);
%     [cont_air_1, res_air_1] = mtf(target_air1, c_air_1(t, 1), c_air_1(t, 2));
    figure; 
    hold on;
%     subplot(2, 1, 1);
    plot(res_water*1000, cont_water*100, lineStyleOrder(1), 'LineWidth', 2);
    plot(res_air_0*1000, cont_air_0*100, lineStyleOrder(2), 'LineWidth', 2);
    title(sprintf("Contrast vs Resolution (Region - %d)", t), 'FontSize', 12);
    ylabel('Contrast, %');
    xlabel(['Resolution, ',char(181),'m/px'],'Interpreter','tex');
    xlim([0 400]);
%     xticks(linspace(0, 400, 100));
%     yticks(linspace(0.2, 1, 0.1));
    ylim([0 100]);
    legend(["Water", "Air"], 'FontSize', 12);
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    grid on;

%     savefig(sprintf('results/water/scale_%d.fig', t));
%     saveas(gcf, sprintf('results/water/scale_%d.png', t));
%     close all;
%     subplot(2, 1, 2);
%     plot(res_air_0*1000, cont_air_0);
%     title(sprintf("MTF: Air0 - %d", t));
%     ylabel('contrast, %');
%     xlabel('resolution, um/px');
%     xticks(linspace(20, 400, 20));
% %     yticks(linspace(0.2, 1, 0.2));
% 
%     grid on;
%     
%     subplot(3, 1, 3);
%     plot(res_air_1*1000, cont_air_1);
%     title(sprintf("MTF: Air1 - %d", t));
%     ylabel('contrast, %');
%     xlabel('resolution, um/px');
%     xticks(linspace(20, 400, 20));
%     yticks(linspace(0.2, 1, 0.2));
% 
%     grid on;
    
%     figure;
%     imshow(target_air1);
    hold off;

end
