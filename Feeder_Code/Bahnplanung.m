%Initialisierung der Anfangs und Endbedingung
x_act=0;    %Startgeschwindigkeit �ber timervalue anpassen, x_act wird anhand von timervalue berechnet
v_act=0;
x_ziel=1000;
v_ziel=30;
a_max=5;
v_max=50; %aktuell nicht verwendet
timervalue=5.0; %TODO evtl mit v_act kombinieren bzw. daraus berechnen lassen
%Einheit f�r L�nge = Schritt
%Einheit f�r Zeit = Schritt/Sekunde

geschwindigkeits_log = [1:1000]'; %aktuelle Geschwindigkeit bei jedem timer compare match
timervalue_log = [1:1000]'; %aktueller Wert bei timer compare match

while(x_act ~= x_ziel) %solange Zielpunkt nicht erreicht wurde
   geschwindigkeits_log(x_act+1 , 1) = 1/timervalue;
   timervalue_log(x_act+1 , 1) = timervalue;
   v_act = 1/timervalue;
   x_diff = x_ziel - x_act; %Weg bis zum Ziel
   v_diff = v_ziel - v_act;  %Geschwindigkeitsunterschied zur Zielgeschwindigkeit
   x_B = (v_act^2) / (2 * a_max) - (v_ziel^2) / (2 * a_max);  %Bremsweg
   if(v_act >= v_ziel)  %Wenn die aktuelle Geschwindigkeit gr��er als die Zielgeschwindigkeit ist
       if(x_diff > x_B) %Wenn der Bremsweg noch ausreicht, Beschleunigen TODO evtl x_B+1
          %Beschleunigen
          if(v_act >= v_max)
              %todo evtl. abbremsen
          else
          timervalue = (-v_act + sqrt(v_act^2 + 4*a_max))/(2*a_max);
          end

       else             %Wenn der Bremsweg nicht mehr reicht, Bremsen
          %Bremsen
          timervalue = (-v_act + sqrt(v_act^2 - 4*a_max))/(-2*a_max);
          
       end              %ggf Totbereich hinzuf�gen      
   else
       %Beschleunigen
       timervalue = (-v_act + sqrt(v_act^2 + 4*a_max))/(2*a_max);
   end   
   x_act = x_act+1;
end
subplot(2, 1, 1);
plot(geschwindigkeits_log)
subplot(2, 1, 2);
plot(timervalue_log)

