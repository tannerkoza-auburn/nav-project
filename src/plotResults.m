function plotResults(data)

%% PLOT PARAMETERS

% Colors
blue    = [0, 0.4470, 0.7410];
orange  = [0.8500, 0.3250, 0.0980];
green   = [0.4660, 0.6740, 0.1880];
yellow  = [0.9290, 0.6940, 0.1250];
lightBlue = [0.3010, 0.7450, 0.9330];

% Font Sizes
fontTitle  = 25;
fontLabel  = 15;
fontTick   = 10;
fontLegend = 10;

% Line Width
lineWidth = 1.75;

% Marker Size
markerSize = 10;

%% PLOT RESULTS

figure("Name","Local Frame Trajectory")
plot(data.truth.NED(:,2), data.truth.NED(:,1),'--k')
hold on
plot(data.gps.NED(:,2), data.gps.NED(:,1))
hold off

l1 = legend('Truth', 'GPS','Location', 'SouthEast');
t1 = title('2D Trajectory: NED');
x1 = xlabel('East [m]');
y1 = ylabel('North [m]');

grid
set(t1,'FontSize', fontTitle);
set(x1,'FontSize', fontLabel);
set(y1,'FontSize', fontLabel);
set(l1,'FontSize', fontLegend);
% set(gca, 'YTickMode', 'auto', 'FontSize', fontTick);
axis equal
axis padded

figure("Name","Geoplot Trajectory")

geoplot(data.truth.LLA(:,1), data.truth.LLA(:,2), '.')
hold on
geoplot(data.gps.LLA(:,1), data.gps.LLA(:,2), '.')
hold off

l2 = legend('Truth', 'GPS','Location', 'SouthEast');
t2 = title('2D Trajectory: LLA');

grid
set(t2,'FontSize', fontTitle);
set(l2,'FontSize', fontLegend);
geobasemap satellite

end