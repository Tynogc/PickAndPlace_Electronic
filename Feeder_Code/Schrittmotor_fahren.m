%% Programm um die Verläufe von x, v, a und deren Statusvariablen
%% testen und visualisieren zu können.

% Speicher vorallozieren.

aktX = zeros(102);               % Motor steht bei Position 0
sollX = 1000;           % Motor soll 1000 Schritte machen
aktV = zeros(102);               % Motor steht
sollV = 30;             % Maximal 30 Schritte pro Zeiteinheit
aktA = zeros(102);               % Motor beschleunigt nicht
aMax = 5;               % Maximal 5 Schritte pro Zeiteinheit mehr als letztes mal
letzterSchritt = zeros(102);  % Timerwert für minimale Geschwindigkeit
letzterSchritt(1) = 2^16-1;
restschritte = zeros(102);       % minimal nötige Schritte bis zum Ziel
deltaX = zeros(102);
deltaV = zeros(102);

% TODO: speicher vor Schleife allozieren.

for j=1:1:100
    i = round(j); % FOR Schleife über FLOAT. WTF.
    if i == 1
        i = 2;
    end
    
    deltaX(i) = sollX - aktX(i-1);
	deltaV(i) = sollV - aktV(i-1);
	
	
    if (aktV > 0)
		restschritte(i) = (deltaV(i).^2) / (2.*aktA(i)) + (sollV .* deltaV(i)) ./ aktA(i);
    end
    
    if (deltaX(i) < restschritte(i) + 1) % bremsen anfangen
			aktV(i) =  aktV(i-1) - aMax .* letzterSchritt(i-1);

    elseif (aktV < sollV) % beschleunigen
				aktV(i) = aktV(i-1) + aMax .* letzterSchritt(i-1);
    end
		% sonst: konstante Fahrtgeschwindigkeit erreicht
		% Schritt erzeugen: v = 1/t -> t = 1/aktV
		letzterSchritt(i) = 1 / (2.*aktV(i));
		aktX(i) = aktX(i-1)+1;
end