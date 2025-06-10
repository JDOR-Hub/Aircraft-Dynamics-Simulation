% plotAircraftSimulationResults.m
% function to plot fixed wing aircraft simulation results obtained with function simulateAircraft
function plotAircraftSimulationResults(t,X,Y,aircraft)
    
    % extract V, alpha, and beta from Y
    V = Y(:,1);
    alpha = Y(:,2);
    beta = Y(:,3);

    % plot airspeed
    figure(1)
    plot(t,V*3600/1852); grid on; xlabel('t (s)'); ylabel('V (kn)'); title(['Airspeed for ', aircraft.aircraftName]);

    % plot angle of attack
    figure(2)
    plot(t,alpha*180/pi); grid on; xlabel('t (s)'); ylabel('\alpha (deg)'); title(['Angle of attack for ', aircraft.aircraftName]);

    % plot angle of sideslip
    figure(3)
    plot(t,beta*180/pi); grid on; xlabel('t (s)'); ylabel('\beta (deg)'); title(['Angle of sideslip for ', aircraft.aircraftName]);

    % plot angle of roll
    figure(4)
    plot(t,X(:,4)*180/pi); grid on; xlabel('t (s)'); ylabel('\phi (deg)'); title(['Roll for ', aircraft.aircraftName]);

    % plot angle of pitch
    figure(5)
    plot(t,X(:,5)*180/pi); grid on; xlabel('t (s)'); ylabel('\theta (deg)'); title(['Pitch for ', aircraft.aircraftName]);

    % plot angle of yaw (heading)
    figure(6)
    plot(t,X(:,6)*180/pi); grid on; xlabel('t (s)'); ylabel('\psi (deg)'); title(['Yaw for ', aircraft.aircraftName]);

     % plot angle of roll rate
    figure(7)
    plot(t,X(:,10)); grid on; xlabel('t (s)'); ylabel('p (rad/s)'); title(['Roll rate for ', aircraft.aircraftName]);

    % plot angle of pitch rate
    figure(8)
    plot(t,X(:,11)); grid on; xlabel('t (s)'); ylabel('q (rad/s)'); title(['Pitch rate for ', aircraft.aircraftName]);

    % plot angle of yaw rate (heading rate)
    figure(9)
    plot(t,X(:,12)); grid on; xlabel('t (s)'); ylabel('r (rad/s)'); title(['Yaw rate for ', aircraft.aircraftName]);

    % plot aircraft position
    figure(10)
    plot(X(:,2),X(:,1)); grid on; axis equal; xlabel('Ye -> E (m)'); ylabel('Xe -> N (m)'); title(['Position for ', aircraft.aircraftName]);

    % plot altitude
    figure(11)
    plot(t,-X(:,3)/0.3048); grid on; xlabel('t (s)'); ylabel('h (ft)'); title(['Altitude for ', aircraft.aircraftName]);

end