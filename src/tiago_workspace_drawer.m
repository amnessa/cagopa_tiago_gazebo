% Define the path to the URDF file
robot_path = "/home/cago/tiago_public_ws/src/tiago_robot/tiago_description/robots/tiago_arm.urdf";

% Import the robot model
robot = importrobot(robot_path, "urdf", DataFormat = "row");

% Set random seed for reproducibility
rng default;

% Define the end-effector link name
ee_link = "arm_tool_link";

% --- Create plot ---
figure('Name', 'TIAGo Arm Workspace Analysis', 'Position', [100, 100, 1000, 800]); % Create a larger figure window
hold on;

% Show the robot in a visible configuration
config = homeConfiguration(robot);
show(robot, config);

% --- Generate workspace in an empty environment ---
disp("Generating workspace (this may take some time)...");
% Generate workspace using the arm_tool_link as the end effector
% We're using IgnoreSelfCollision="on" to get the theoretical maximum workspace
[ws, configs] = generateRobotWorkspace(robot, {}, ee_link, IgnoreSelfCollision="on", MaxNumSamples=10000);

disp("Workspace generation complete.");
fprintf("Generated %d workspace points\n", size(ws, 1));

% --- Visualize workspace ---
if isempty(ws)
    warning('No workspace points generated. Check robot model or function parameters.');
else
    % Calculate manipulability index for each configuration
    disp("Calculating manipulability index...");
    mIdx = manipulabilityIndex(robot, configs, ee_link);
    
    % Method 1: Plot workspace as points with manipulability color coding
    subplot(1, 2, 1);
    show(robot, config);
    hold on;
    scatter3(ws(:,1), ws(:,2), ws(:,3), 20, mIdx, 'filled');
    colorbar;
    title("TIAGo Arm Workspace - Manipulability");
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    axis equal;
    grid on;
    view(3);
    
    % Method 2: Plot the workspace as an alpha shape
    subplot(1, 2, 2);
    show(robot, config);
    hold on;
    
    % Create an alpha shape from the workspace points
    % Adjust alpha radius for better shape representation
    alphaRadius = 0.1; % Can be adjusted based on point density
    wsAlpha = alphaShape(ws(:,1), ws(:,2), ws(:,3), alphaRadius);
    
    % Plot the alpha shape
    plot(wsAlpha, 'FaceColor', 'cyan', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    
    title("TIAGo Arm Workspace - Volume");
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    axis equal;
    grid on;
    view(3);
    
    % Display workspace volume
    vol = volume(wsAlpha);
    disp(['Workspace volume: ', num2str(vol), ' cubic meters']);
end

% --- Save figure ---
saveas(gcf, 'tiago_arm_workspace.png');
disp("Figure saved as 'tiago_arm_workspace.png'");