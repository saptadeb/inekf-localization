data = load('filteredData.mat');
xgps = load('xgps.mat');
ygps = load('ygps.mat');
[numSteps, ~] = size(data.filteredData);
pauseLen = 0.3;
makeVideo = true;

GLOBAL_FIGURE = 1;

% Beautiful colors
blue      = [0, 0.4470, 0.7410];
orange    = [0.8500, 0.3250, 0.0980];
yellow    = [0.9290, 0.6940, 0.1250];
purple    = [0.4940, 0.1840, 0.5560];
green     = [0.4660, 0.6740, 0.1880];
lightblue = [0.3010, 0.7450, 0.9330];
red       = [0.6350, 0.0780, 0.1840];

if makeVideo
    votype = 'VideoWriter';
    vo = VideoWriter('video.avi', 'Motion JPEG AVI');
    set(vo, 'FrameRate', min(5, 1/pauseLen));
    open(vo);
end

frameRate=100;
for t = 1:frameRate:numSteps
    figure(GLOBAL_FIGURE); clf; hold all;
    title('\textbf{Robot Trajectory}', 'Interpreter', 'latex');
    xlabel('$x$ (m)', 'Interpreter', 'latex')
    ylabel('$y$ (m)', 'Interpreter', 'latex')
    axis([-80 130 -450 150]);
    axis equal

    x = data.filteredData(t, 5);
    y = data.filteredData(t, 6);
    theta = data.filteredData(t, 7);
    plot(data.filteredData(1,5), data.filteredData(1,6), '*', 'Color', yellow, 'MarkerSize', 10);
    plot(data.filteredData(1:t,5), data.filteredData(1:t,6), 'Color', red, 'linewidth', 1.5);
    plot(data.filteredData(1:t,11), data.filteredData(1:t,12), 'Color', blue, 'linewidth', 1.5);
    plot(xgps.xGpsCg(1:t), ygps.yGpsCg(1:t), 'Color', green, 'linewidth', 1.5);
    
    XL = get(gca, 'XLim');
    YL = get(gca, 'YLim');
    radius=10;
    plotrobot(x, y, theta, radius, 'black', 1, 'cyan');
    set(gca, 'fontsize', 14)
    legend({'Starting Location', 'LIEKF', 'Ground truth', 'Consumer grade GPS'}, 'Location','Best', 'Interpreter', 'latex')
    drawnow limitrate
    
    if pauseLen == inf
        pause;
    elseif pauseLen > 0
        pause(pauseLen);
    end
    
    if makeVideo
        F = getframe(gcf);
        switch votype
          case 'avifile'
            vo = addframe(vo, F);
          case 'VideoWriter'
            writeVideo(vo, F);
          otherwise
            error('unrecognized votype');
        end
    end
end

if makeVideo
    fprintf('Writing video...');
    switch votype
      case 'avifile'
        vo = close(vo);
      case 'VideoWriter'
        close(vo);
      otherwise
        error('unrecognized votype');
    end
    fprintf('done\n');
end