clear all; close all; clc;

%% Configuration
COM_PORT = 'COM4';  
BAUD_RATE = 115200;
SEND_INTERVAL = 2;
SIMULATION_SPEED = 1;

fprintf('Connecting to ESP32 on %s...\n', COM_PORT);
try
    delete(instrfind('Port', COM_PORT));
    s = serialport(COM_PORT, BAUD_RATE);
    configureTerminator(s, "LF");
    configureCallback(s, "terminator", @readSerialData);
    fprintf('Connected successfully!\n');
catch ME
    error('Failed to connect to ESP32. Check COM port and try again.\nError: %s', ME.message);
end


battery = 100.0;
solarCharging = true;
voltage = 12.6;
temperature = 28.0;
chargeCurrent = 0.0;

% GPS System
lat = 13.0827;
lng = 80.2707;
alt = 0.0;
heading = 0.0;

modes = {'IDLE', 'PLANNING', 'SCANNING', 'COVERING', 'COLLECTING', 'MOVING', 'DOCKING', 'CHARGING', 'ERROR'};
currentMode = 1;  
lidar = 20.0;
speed = 0.0;
motorTemp = 25.0;

waterTemp = 26.0;
humidity = 65.0;
pressure = 1013.25;
waterQuality = "Good";

rpmLeft = 0;
rpmRight = 0;

areaCovered = 0.0;
trashCollected = 0.0;
cellsCovered = 0;
totalCells = 0;
coveragePercent = 0.0;

operationArea = [];
pathPoints = [];
currentPathIndex = 1;
pathGenerated = false;
cellSize = 5;

% Environmental Simulation
windSpeed = 0;
waterCurrent = 0;
waveHeight = 0;
trashDensity = "Low";

% Time and Mission Management
missionStarted = false;
missionStartTime = datetime;
simulationTime = 0;
timeOfDay = 12;

% Global variables for serial callback
global globalCurrentMode globalBattery globalAreaCovered globalTrashCollected ...
       globalOperationArea globalPathPoints globalPathGenerated ...
       globalCurrentPathIndex globalTotalCells globalCellsCovered;

globalCurrentMode = currentMode;
globalBattery = battery;
globalAreaCovered = areaCovered;
globalTrashCollected = trashCollected;
globalOperationArea = operationArea;
globalPathPoints = pathPoints;
globalPathGenerated = pathGenerated;
globalCurrentPathIndex = currentPathIndex;
globalTotalCells = totalCells;
globalCellsCovered = cellsCovered;

fprintf('\n=== Advanced Water Robot Simulation Started ===\n');
fprintf('Features: Boustrophedon Path Planning, Environmental Simulation\n');
fprintf('Press Ctrl+C to stop\n\n');

%% Serial Data Callback Function
function readSerialData(src, ~)
    global globalCurrentMode globalBattery globalAreaCovered globalTrashCollected ...
           globalOperationArea globalPathPoints globalPathGenerated ...
           globalCurrentPathIndex globalTotalCells globalCellsCovered;
    
    try
        data = readline(src);
        data = strtrim(data);
        
        if isempty(data)
            return;
        end
        
        fprintf('Received from ESP32: %s\n', data);
        
        % Check if it's a command
        if contains(data, 'CMD:')
            command = extractAfter(data, 'CMD:');
            
            switch command
                case 'START_MISSION'
                    globalCurrentMode = 2; % PLANNING
                    fprintf('Command received: Starting mission with path planning\n');
                    
                case 'STOP'
                    globalCurrentMode = 9; % ERROR
                    fprintf('Command received: Emergency stop\n');
                    
                case 'DOCK'
                    globalCurrentMode = 7; % DOCKING
                    fprintf('Command received: Returning to dock\n');
                    
                case 'RESET'
                    globalBattery = 100;
                    globalAreaCovered = 0;
                    globalTrashCollected = 0;
                    globalCurrentMode = 1;
                    globalOperationArea = [];
                    globalPathPoints = [];
                    globalPathGenerated = false;
                    globalCurrentPathIndex = 1;
                    globalCellsCovered = 0;
                    fprintf('Command received: System reset\n');
                    
                case 'PAUSE'
                    if globalCurrentMode == 4 || globalCurrentMode == 3 % COVERING or SCANNING
                        globalCurrentMode = 1; % IDLE
                        fprintf('Command received: Mission paused\n');
                    elseif globalCurrentMode == 1 && ~isempty(globalPathPoints)
                        globalCurrentMode = 4; % COVERING
                        fprintf('Command received: Mission resumed\n');
                    end
                    
                case 'MANUAL:FORWARD'
                    fprintf('Command received: Manual control - FORWARD\n');
                    
                case 'MANUAL:LEFT'
                    fprintf('Command received: Manual control - LEFT\n');
                    
                case 'MANUAL:RIGHT'
                    fprintf('Command received: Manual control - RIGHT\n');
                    
                case 'MANUAL:BACKWARD'
                    fprintf('Command received: Manual control - BACKWARD\n');
            end
            
        % Check if it's JSON data (area confirmation)
        elseif startsWith(data, '{')
            try
                jsonData = jsondecode(data);
                
                if isfield(jsonData, 'type') && strcmp(jsonData.type, 'AREA_CONFIRMATION')
                    % Extract area coordinates
                    coords = jsonData.coordinates;
                    globalOperationArea = [];
                    
                    for i = 1:length(coords)
                        globalOperationArea = [globalOperationArea; 
                                       coords(i).lat, 
                                       coords(i).lng];
                    end
                    
                    globalTotalCells = jsonData.totalCells;
                    globalPathGenerated = false;
                    
                    fprintf('Operation area confirmed: %d points, %d cells\n', ...
                        size(globalOperationArea, 1), globalTotalCells);
                    
                    % Generate path
                    if ~isempty(globalOperationArea)
                        globalPathPoints = generateBoustrophedonPath(globalOperationArea, 5);
                        globalPathGenerated = true;
                        globalCurrentPathIndex = 1;
                        globalCellsCovered = 0;
                        fprintf('Path generated with %d points\n', size(globalPathPoints, 1));
                    end
                end
            catch
                fprintf('Failed to parse JSON data\n');
            end
        end
        
    catch ME
        fprintf('Error reading serial data: %s\n', ME.message);
    end
end

try
    iteration = 0;
    
    pause(3);
    
    while true
        iteration = iteration + 1;
        simulationTime = iteration * SEND_INTERVAL;
        
        timeOfDay = mod(simulationTime / 150, 24);
        
        currentMode = globalCurrentMode;
        battery = globalBattery;
        areaCovered = globalAreaCovered;
        trashCollected = globalTrashCollected;
        operationArea = globalOperationArea;
        pathPoints = globalPathPoints;
        pathGenerated = globalPathGenerated;
        currentPathIndex = globalCurrentPathIndex;
        totalCells = globalTotalCells;
        cellsCovered = globalCellsCovered;
        
        switch modes{currentMode}
            case 'IDLE'
                speed = 0;
                rpmLeft = 0;
                rpmRight = 0;
                lidar = 10 + rand() * 5;
                
            case 'PLANNING'
                speed = 0;
                if ~isempty(operationArea) && ~pathGenerated
                    fprintf('Generating boustrophedon path decomposition...\n');
                    
                    pathPoints = generateBoustrophedonPath(operationArea, cellSize);
                    pathGenerated = true;
                    totalCells = size(pathPoints, 1);
                    cellsCovered = 0;
                    currentPathIndex = 1;
                    
                    fprintf('Path generated with %d cells\n', totalCells);
                    currentMode = 3; % SCANNING
                    globalCurrentMode = 3;
                    globalPathPoints = pathPoints;
                    globalPathGenerated = true;
                    globalTotalCells = totalCells;
                end
                
            case 'SCANNING'
                speed = 0.3;
                rpmLeft = 600;
                rpmRight = 600;
                lidar = 10 + rand() * 40;
                
                if pathGenerated && currentPathIndex <= size(pathPoints, 1)
                    target = pathPoints(currentPathIndex, :);
                    [lat, lng, heading] = moveToPoint(lat, lng, target, speed);
                    
                    distanceToTarget = calculateDistance(lat, lng, target(1), target(2));
                    if distanceToTarget < 2.5
                        currentMode = 4; 
                        globalCurrentMode = 4;
                        fprintf('Reached cell %d/%d\n', currentPathIndex, totalCells);
                    end
                end
                
            case 'COVERING'
                speed = 0.4;
                rpmLeft = 800 + randi([-100, 100]);
                rpmRight = 800 + randi([-100, 100]);
                
                if currentPathIndex <= size(pathPoints, 1)
                    cellCenter = pathPoints(currentPathIndex, :);
                    [lat, lng, heading, coverageComplete] = coverCell(lat, lng, cellCenter, cellSize, iteration);
                    
                    if coverageComplete
                        cellsCovered = currentPathIndex;
                        coveragePercent = (cellsCovered / totalCells) * 100;
                        areaCovered = areaCovered + (cellSize^2);
                        
                        if rand() > 0.7
                            trashCollected = trashCollected + rand() * 2;
                        end
                        
                        currentPathIndex = currentPathIndex + 1;
                        globalCurrentPathIndex = currentPathIndex;
                        globalCellsCovered = cellsCovered;
                        globalAreaCovered = areaCovered;
                        globalTrashCollected = trashCollected;
                        
                        if currentPathIndex > totalCells
                            currentMode = 7; 
                            globalCurrentMode = 7;
                            fprintf('All cells covered - Mission complete!\n');
                        else
                            currentMode = 3; 
                            globalCurrentMode = 3;
                        end
                    end
                end
                
            case 'COLLECTING'
                speed = 0.1;
                rpmLeft = 300;
                rpmRight = 300;
                trashCollected = trashCollected + rand() * 2;
                globalTrashCollected = trashCollected;
                
                pause(2);
                currentMode = 4;
                globalCurrentMode = 4;
                fprintf('Trash collection complete\n');
                
            case 'MOVING'
                speed = 0.8 + rand() * 0.4;
                rpmLeft = 1200 + randi([-100, 100]);
                rpmRight = 1200 + randi([-100, 100]);
                
                lat = lat + (rand() - 0.5) * 0.0001;
                lng = lng + (rand() - 0.5) * 0.0001;
                heading = rand() * 360;
                areaCovered = areaCovered + speed * SEND_INTERVAL * 1.5;
                globalAreaCovered = areaCovered;
                
            case 'DOCKING'
                speed = 0.3;
                rpmLeft = 500;
                rpmRight = 500;
                
                startPoint = [13.0827, 80.2707];
                [lat, lng, heading] = moveToPoint(lat, lng, startPoint, speed);
                
                distanceToStart = calculateDistance(lat, lng, startPoint(1), startPoint(2));
                if distanceToStart < 5
                    pause(3);
                    currentMode = 8;
                    globalCurrentMode = 8;
                    fprintf('Successfully docked\n');
                end
                
            case 'CHARGING'
                speed = 0;
                rpmLeft = 0;
                rpmRight = 0;
                
                if battery < 100
                    battery = min(100, battery + 2);
                    solarCharging = true;
                    chargeCurrent = 5.0;
                    globalBattery = battery;
                else
                    currentMode = 1; % Back to IDLE
                    globalCurrentMode = 1;
                    solarCharging = false;
                    chargeCurrent = 0;
                    fprintf('Charging complete\n');
                end
                
            case 'ERROR'
                speed = 0;
                rpmLeft = 0;
                rpmRight = 0;
                fprintf('Error mode - Waiting for reset command\n');
                pause(2);
        end
        
        % Update battery system
        if solarCharging && currentMode == 8  % Charging at dock
            battery = min(100, battery + 0.5);
            globalBattery = battery;
        else
            battery = max(20, battery - 0.1 * speed);
            globalBattery = battery;
        end
        
        voltage = 11.5 + (battery / 100) * 1.1;
        
        % Solar charging simulation
        solarCharging = (rand() > 0.3);
        
        % Environmental variations
        temperature = 28 + randn() * 2;
        waterTemp = 26 + randn() * 1.5;
        humidity = 65 + randn() * 5;
        pressure = 1013.25 + randn() * 2;
        
        % LIDAR with noise
        lidar = max(5, min(50, lidar + randn() * 3));
        
        % Environmental simulation
        windSpeed = rand() * 10;
        waterCurrent = rand() * 2;
        waveHeight = rand() * 0.3;
        
        densities = {'Low', 'Medium', 'High'};
        if rand() < 0.05
            trashDensity = densities{randi(3)};
        end
        
        % Water quality
        if rand() < 0.02
            qualities = {'Good', 'Fair', 'Poor'};
            waterQuality = qualities{randi(3)};
        end
        
        % Motor temperature
        motorTemp = 25 + speed * 10 + randn();
        
        data = struct();
        
        data.battery = round(battery, 2);
        data.voltage = round(voltage, 2);
        data.temp = round(temperature, 1);
        data.solar = solarCharging;
        data.chargeCurrent = round(chargeCurrent, 2);
        
        % GPS data
        data.lat = round(lat, 6);
        data.lng = round(lng, 6);
        data.alt = round(alt, 1);
        data.heading = round(heading, 1);
        
        % Status data
        data.mode = modes{currentMode};
        data.lidar = round(lidar, 1);
        data.speed = round(speed, 2);
        data.motorTemp = round(motorTemp, 1);
        
        % Sensor readings
        data.waterTemp = round(waterTemp, 1);
        data.humidity = round(humidity, 1);
        data.pressure = round(pressure, 1);
        data.rpmLeft = round(rpmLeft);
        data.rpmRight = round(rpmRight);
        data.waterQuality = waterQuality;
        
        % Mission progress - SIMPLIFIED to match Arduino format
        data.area = round(areaCovered, 1);
        data.trash = round(trashCollected, 1);
        data.cellsCovered = cellsCovered;
        data.totalCells = totalCells;
        data.coveragePercent = round((cellsCovered / max(1, totalCells)) * 100, 1);
        
        % Environmental data - NESTED structure
        data.environment = struct();
        data.environment.windSpeed = round(windSpeed, 1);
        data.environment.waterCurrent = round(waterCurrent, 1);
        data.environment.waveHeight = round(waveHeight, 2);
        data.environment.trashDensity = trashDensity;
        
        jsonStr = jsonencode(data);
        
        % Send to ESP32
        writeline(s, jsonStr);
        
        % Display status
        if mod(iteration, 5) == 0
            if pathGenerated
                fprintf('[%s] %s | Battery: %.1f%% | Cells: %d/%d (%.1f%%) | Trash: %.1fkg\n', ...
                    datestr(now, 'HH:MM:SS'), modes{currentMode}, battery, cellsCovered, totalCells, data.coveragePercent, trashCollected);
            else
                fprintf('[%s] %s | Battery: %.1f%% | Area: %.1fm² | Trash: %.1fkg\n', ...
                    datestr(now, 'HH:MM:SS'), modes{currentMode}, battery, areaCovered, trashCollected);
            end
        end
        
        % Wait before next iteration
        pause(SEND_INTERVAL);
    end
    
catch ME
    fprintf('\nSimulation stopped.\n');
    fprintf('Error: %s\n', ME.message);
end

%% Cleanup
clear s;
fprintf('\nSerial connection closed.\n');

%% Helper Functions

function path = generateBoustrophedonPath(areaPolygon, cellSize)
    % Get bounding box
    minLat = min(areaPolygon(:, 1));
    maxLat = max(areaPolygon(:, 1));
    minLng = min(areaPolygon(:, 2));
    maxLng = max(areaPolygon(:, 2));
    
    % Convert cell size from meters to degrees (approximate)
    cellSizeDeg = cellSize / 111000;
    
    % Create grid
    latGrid = minLat:cellSizeDeg:maxLat;
    lngGrid = minLng:cellSizeDeg*2:maxLng;
    
    path = [];
    direction = 1;
    
    for i = 1:length(latGrid)
        if direction == 1
            for j = 1:length(lngGrid)
                point = [latGrid(i), lngGrid(j)];
                if isPointInPolygon(point, areaPolygon)
                    path = [path; point];
                end
            end
        else
            for j = length(lngGrid):-1:1
                point = [latGrid(i), lngGrid(j)];
                if isPointInPolygon(point, areaPolygon)
                    path = [path; point];
                end
            end
        end
        direction = -direction;
    end
end

function inside = isPointInPolygon(point, polygon)
    x = point(2); y = point(1);
    n = size(polygon, 1);
    inside = false;
    
    p1x = polygon(1, 2); p1y = polygon(1, 1);
    for i = 1:n+1
        p2x = polygon(mod(i, n)+1, 2);
        p2y = polygon(mod(i, n)+1, 1);
        
        if y > min(p1y, p2y)
            if y <= max(p1y, p2y)
                if x <= max(p1x, p2x)
                    if p1y ~= p2y
                        xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x;
                    end
                    if p1x == p2x || x <= xinters
                        inside = ~inside;
                    end
                end
            end
        end
        p1x = p2x; p1y = p2y;
    end
end

function [newLat, newLng, newHeading] = moveToPoint(currentLat, currentLng, target, speed)
    speedDeg = speed / 111000;
    
    dLat = target(1) - currentLat;
    dLng = target(2) - currentLng;
    dist = sqrt(dLat^2 + dLng^2);
    
    newHeading = atan2d(dLng, dLat);
    
    if dist > speedDeg
        newLat = currentLat + (dLat/dist) * speedDeg;
        newLng = currentLng + (dLng/dist) * speedDeg;
    else
        newLat = target(1);
        newLng = target(2);
    end
end

function [newLat, newLng, newHeading, coverageComplete] = coverCell(currentLat, currentLng, center, cellSize, iteration)
    cellSizeDeg = cellSize / 111000;
    halfSize = cellSizeDeg / 2;
    
    patternStep = mod(iteration, 20) / 20;
    
    if mod(floor(iteration/5), 2) == 0
        lngOffset = -halfSize + patternStep * cellSizeDeg;
    else
        lngOffset = halfSize - patternStep * cellSizeDeg;
    end
    
    latOffset = (rand() - 0.5) * halfSize;
    
    newLat = center(1) + latOffset;
    newLng = center(2) + lngOffset;
    
    if mod(floor(iteration/5), 2) == 0
        newHeading = 90;
    else
        newHeading = 270;
    end
    
    coverageComplete = (patternStep > 0.95);
end

function dist = calculateDistance(lat1, lng1, lat2, lng2)
    R = 6371000;
    
    dLat = deg2rad(lat2 - lat1);
    dLng = deg2rad(lng2 - lng1);
    
    a = sin(dLat/2)^2 + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLng/2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    dist = R * c;
end