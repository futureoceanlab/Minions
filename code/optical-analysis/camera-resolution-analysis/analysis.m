e = "data/comparison/edmund_2580.png";
c = "data/comparison/cnaico_2580.png";
svl = "data/comparison/svl_2580.png";

t_list = [e; c; svl];
c_target = [1305 946;1297 955; 1294 975];
zoomLen = 20;
contZoom = zeros(20, 3);
lineStyleOrder = ["-", "--", "-."];
figure; 
hold on;
for i=1:3
    target = rgb2gray(imread(t_list(i))); 
    cx = c_target(i, 1); 
    cy= c_target(i, 2); 
    [cont, res] = mtf(target, cx, cy, 5);
    contZoom(:, i) = cont(1:zoomLen);
    plot(res*1000, cont*100, lineStyleOrder(i), 'LineWidth', 2);

end
title("Lens Resolution Comparison", 'FontSize', 12);
ylabel('Contrast, %');
xlabel(['Resolution, ',char(181),'m/px'],'Interpreter','tex');
xlim([0 350]);
ylim([0 100]);
legend(["Edmund Optics", "cnAICO", "ScorpionVision"]);
ax = gca;
ax.XAxis.FontSize = 12;
ax.YAxis.FontSize = 12;
grid on;
hold off;


figure;
hold on;
for i=1:3
    plot(1000*res(1:zoomLen), contZoom(:, i)*100, lineStyleOrder(i),  'LineWidth', 2);
end
xlim([15 50]);
ylabel('Contrast, %');
xlabel(['Resolution, ',char(181),'m/px'],'Interpreter','tex');
grid on;
ax = gca;
ax.XAxis.FontSize = 12;
ax.YAxis.FontSize = 12;
hold off;