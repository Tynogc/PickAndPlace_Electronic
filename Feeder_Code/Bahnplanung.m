%% Initialisierung der Anfangs und Endbedingung
x_act=single(0);    %Startgeschwindigkeit �ber timervalue anpassen, x_act wird anhand von timervalue berechnet
v_act=single(0);
x_ziel=single(1000);
v_ziel=single(0);
a_max=single(0.001);
v_max=single(0.5); % Bedingt durch den Zusammenhang v = 1/timer
timervalue=single(65000); %TODO evtl mit v_act kombinieren bzw. daraus berechnen lassen
%Einheit für Länge = Schritt
%Einheit für Zeit = Schritt/Sekunde

geschwindigkeits_log = [1:1000]'; %aktuelle Geschwindigkeit bei jedem timer compare match
timervalue_log = [1:1000]'; %aktueller Wert bei timer compare match

while(x_act ~= x_ziel) %solange Zielpunkt nicht erreicht wurde
   % Nur logging, keine Funktion im späteren Programm
   geschwindigkeits_log(x_act+1 , 1) = 1/timervalue;
   timervalue_log(x_act+1 , 1) = timervalue;
   
   
   
   %v_act = 1/timervalue;
   x_diff = x_ziel - x_act; %Weg bis zum Ziel
   v_diff = v_ziel - 1/timervalue;  %Geschwindigkeitsunterschied zur Zielgeschwindigkeit
   %x_B = (1/timervalue^2)/(2*a_max)-(v_ziel^2)/(2*a_max);  %Bremsweg
   x_B = (1/(timervalue^2*2*a_max))-(v_ziel^2)/(2*a_max);  %Bremsweg
   
   if(1/timervalue >= v_ziel)  %Wenn die aktuelle Geschwindigkeit gr��er als die Zielgeschwindigkeit ist
       
       if(x_diff > x_B) %Wenn der Bremsweg noch ausreicht, Beschleunigen TODO evtl x_B+1
          %Beschleunigen
          if(1/timervalue >= v_max)
              %todo evtl. abbremsen
          else
          timervalue = (sqrt((1/timervalue)^2 + 4*a_max) - 1/timervalue)/(2*a_max);
          end

       else             %Wenn der Bremsweg nicht mehr reicht, Bremsen
          %Bremsen
          timervalue = (sqrt((1/timervalue)^2 - 4*a_max) - 1/timervalue)/(-2*a_max);
          
       end              %ggf Totbereich hinzuf�gen      
   else
       %Beschleunigen
       timervalue = (sqrt((1/timervalue)^2 + 4*a_max) - 1/timervalue)/(2*a_max);
   end   
   x_act = x_act+1;
end
subplot(2, 1, 1);
plot(geschwindigkeits_log)
subplot(2, 1, 2);
plot(timervalue_log)

%%

% Wichtig: 
% für alle Operatoren Overflow/Underflow überprüfen


% Datenformat:
% Schritte: uint32
% a, v: 32 bit, Komma an 16. Stelle binär (d.h. 16 Bit für Vor- und Nachkommabereich)
% Timerwert: uint32, muss auf 65535 = 2^16-1 beschränkt werden. 
