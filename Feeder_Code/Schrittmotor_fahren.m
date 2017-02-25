%% Programm um die Verläufe von x, v, a und deren Statusvariablen
%% testen und visualisieren zu können.

% Speicher vorallozieren.

aktX = zeros(102,1);               % Motor steht bei Position 0
sollX = 1000;                      % Motor soll 1000 Schritte machen
aktV = zeros(102,1);               % Motor steht
sollV = 0;                         % Zielgeschwindigkeit am Ende der Fahrt
maxV=30;                           % Maximalgeschwindigkeit beim Verfahren
aktA = zeros(102,1);               % Motor beschleunigt nicht
aMax = 1e-5;                          % Maximal 5 Schritte pro Zeiteinheit mehr als letztes mal
letzterSchritt = zeros(102,1);     % Timerwert für minimale Geschwindigkeit
letzterSchritt(1) = 2^16-1;
restschritte = zeros(102,1);       % minimal nötige Schritte bis zum Ziel
deltaX = zeros(102,1);
deltaV = zeros(102,1);

% TODO: Speicher vor Schleife allozieren.

for j=2:1:100
    i = round(j); % FOR Schleife über FLOAT. WTF.
    
    deltaX(i) = sollX - aktX(i-1);  %Strecke die noch zu fahren ist
	deltaV(i) = sollV - aktV(i-1);  
	
    if (aktV(i) > 0)
        % minimal nötige Schritte zum bremsen berechnen
		restschritte(i) = (deltaV(i)^2) / (2*aktA(i)) + (sollV * deltaV(i)) / aktA(i);
    end
    
    if (deltaX(i) < restschritte(i) + 1) % bremsen anfangen
			aktV(i) =  aktV(i-1) - aMax * letzterSchritt(i-1);

    elseif (aktV(i) < maxV) % beschleunigen
				aktV(i) = aktV(i-1) + aMax * letzterSchritt(i-1);
    end
		% sonst: konstante Fahrtgeschwindigkeit erreicht
		% Schritt erzeugen: v = 1/t -> t = 1/aktV
		letzterSchritt(i) = 1 / (2*aktV(i));
		aktX(i) = aktX(i-1)+1;
end