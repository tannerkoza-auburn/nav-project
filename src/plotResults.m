function plotResults(data)

    %% PLOT PARAMETERS

    % Colors
    blue = [0, 0.4470, 0.7410];
    orange = [0.8500, 0.3250, 0.0980];
    green = [0.4660, 0.6740, 0.1880];
    yellow = [0.9290, 0.6940, 0.1250];
    lightBlue = [0.3010, 0.7450, 0.9330];

    % Font Sizes
    fontTitle = 25;
    fontLabel = 15;
    fontTick = 10;
    fontLegend = 10;

    % Line Width
    lineWidth = 1.75;

    % Marker Size
    markerSize = 10;

    %% PLOT RESULTS

    % Local Frame Trajectory
    figure("Name", "Local Frame Trajectory")
    plot(data.truth.NED(:, 2), data.truth.NED(:, 1), '--k')
    hold on
    plot(data.gps.NED(:, 2), data.gps.NED(:, 1))
    hold off

    l1 = legend('Truth', 'GPS', 'Location', 'SouthEast');
    t1 = title('2D Trajectory: NED');
    x1 = xlabel('East [m]');
    y1 = ylabel('North [m]');

    grid
    set(t1, 'FontSize', fontTitle);
    set(x1, 'FontSize', fontLabel);
    set(y1, 'FontSize', fontLabel);
    set(l1, 'FontSize', fontLegend);
    % set(gca, 'YTickMode', 'auto', 'FontSize', fontTick);
    axis equal
    axis padded

    % Geoplot Trajectory
    figure("Name", "Geoplot Trajectory")
    geoplot(data.truth.LLA(:, 1), data.truth.LLA(:, 2), '.')
    hold on
    geoplot(data.gps.LLA(:, 1), data.gps.LLA(:, 2), '.')
    hold off

    l2 = legend('Truth', 'GPS', 'Location', 'SouthEast');
    t2 = title('2D Trajectory: LLA');

    grid
    set(t2, 'FontSize', fontTitle);
    set(l2, 'FontSize', fontLegend);
    geobasemap satellite

    % Attitude
    figure("Name", "Attitude: Euler Angles")
    subplot(3, 1, 1)
    plot(data.truth.timeDuration, data.truth.euler(:, 1), '.k')
    hold on
    plot(data.results.timeDuration, data.results.euler(:, 3), '.')
    hold on

    l3 = legend('Truth', 'Mechanized', 'Location', 'SouthEast');
    t3 = title('Attitude: Yaw');
    x3 = xlabel('time [s]');
    y3 = ylabel('yaw [deg]');
    ax3 = gca;

    grid
    set(t3, 'FontSize', fontTitle);
    set(x3, 'FontSize', fontLabel);
    set(y3, 'FontSize', fontLabel);
    set(l3, 'FontSize', fontLegend);

    subplot(3, 1, 2)
    plot(data.truth.timeDuration, data.truth.euler(:, 2), '.k')
    hold on
    plot(data.results.timeDuration, data.results.euler(:, 2), '.')
    hold on

    l4 = legend('Truth', 'Mechanized', 'Location', 'SouthEast');
    t4 = title('Attitude: Pitch');
    x4 = xlabel('time [s]');
    y4 = ylabel('pitch [deg]');
    ax4 = gca;

    grid
    ax4.YLim = ax3.YLim;
    set(t4, 'FontSize', fontTitle);
    set(x4, 'FontSize', fontLabel);
    set(y4, 'FontSize', fontLabel);
    set(l4, 'FontSize', fontLegend);

    subplot(3, 1, 3)
    plot(data.truth.timeDuration, data.truth.euler(:, 3), '.k')
    hold on
    plot(data.results.timeDuration, data.results.euler(:, 1), '.')
    hold on

    l5 = legend('Truth', 'Mechanized', 'Location', 'SouthEast');
    t5 = title('Attitude: Roll');
    x5 = xlabel('time [s]');
    y5 = ylabel('roll [deg]');
    ax5 = gca;

    grid
    ax5.YLim = ax3.YLim;
    set(t5, 'FontSize', fontTitle);
    set(x5, 'FontSize', fontLabel);
    set(y5, 'FontSize', fontLabel);
    set(l5, 'FontSize', fontLegend);

end