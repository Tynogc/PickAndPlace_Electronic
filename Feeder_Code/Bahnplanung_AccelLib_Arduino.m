%Initialisierung der Anfangs und Endbedingung


geschwindigkeits_log = [1:1005]'; %aktuelle Geschwindigkeit bei jedem timer compare match
timervalue_log = [1:1005]'; %aktueller Wert bei timer compare match
bremsweg_log = [1:1005]'; % Bremsweg nach anderer Formel

currentPos = 0;
targetPos = 1000;
speed = 0.0;
maxSpeed = 300;
acceleration = 30;
stepInterval = 0;
lastStepTime = 0;
actualSpeed = 0;
i = 1;
geschwindigkeits_log(i, 1) = 1;

while (targetPos ~= currentPos)
    i=i+1;

distanceToGo = targetPos - currentPos;
if (distanceToGo == 0)
    geschwindigkeits_log(i, 1) = 0; % return...
elseif (distanceToGo > 0) % clockwise
    requiredSpeed = sqrt(2.0 * distanceToGo * acceleration);
    bremsweg_log(i, 1) = ((actualSpeed^2 / (2*acceleration)));
else  % Anticlockwise
    requiredSpeed = -sqrt(2.0 * -distanceToGo * acceleration);
end

if (requiredSpeed > actualSpeed) % Motor steht oder ist zu langsam
    % Need to accelerate in clockwise direction
    if (actualSpeed == 0)
        requiredSpeed = sqrt(2.0 * acceleration); % anfahren
    else
	    requiredSpeed = actualSpeed + abs(acceleration / actualSpeed); % beschleunigen
    end
    if (requiredSpeed > maxSpeed) % Maximalgeschwindigkeit erreicht
        requiredSpeed = maxSpeed; % Limit
    end
elseif (requiredSpeed < actualSpeed)
    
    if (actualSpeed == 0)
	    requiredSpeed = -sqrt(2.0 * acceleration); % Need to accelerate in anticlockwise direction
    else
	    requiredSpeed = actualSpeed - abs(acceleration / actualSpeed); % bremsen
    end
    if (requiredSpeed < -maxSpeed)
	    requiredSpeed = -maxSpeed;
    end
end

actualSpeed = requiredSpeed;
currentPos = currentPos+1;

geschwindigkeits_log(i, 1) = requiredSpeed;
timervalue_log(i,1) = 1 / requiredSpeed;

end

p1 = subplot(2, 1, 1);
yyaxis(p1, 'left')
plot(p1, geschwindigkeits_log)
ylim([0 220])
ylabel('Geschwindigkeit');
yyaxis(p1, 'right')
plot(p1, bremsweg_log)
ylim([0 55])
xlim([0 1010])
ylabel('Bremsweg aktuell');

xlabel('i');
subplot(2, 1, 2);
plot(timervalue_log)
xlabel('i');
ylabel('1/requiredSpeed');
