function plotQuantitative(quantResult, max_iter)
% plotQuantitative plots the success and collision metrics from a cell array of structs
% and calculates the total numbers of success and collision events.

% Initialize arrays to hold success and collision values
successValues = zeros(length(quantResult), 1);
collisionValues = zeros(length(quantResult), 1);

% Iterate through each struct in the cell array to extract data
for i = 1:length(quantResult)
    % Extract success and collision values from the current struct
    successValues(i) = quantResult{i}.success;
    collisionValues(i) = quantResult{i}.collision;
end

% Calculate total numbers of success and collision events
totalSuccess = sum(successValues);
totalCollision = sum(collisionValues);

% Create the bar graph
plot(2, 1);
bar([length(quantResult), totalSuccess, totalCollision]); 
xticks([1, 2, 3]);
xticklabels({'Total rounds', 'Found the best way', 'Collied'});
ylabel('Count');
str = ['Success analysis for iteration times: ', num2str(max_iter)];
title(str);
end