% VITOR MACHADO GUILHERME BARROS
% vitorbarros@dcc.ufmg.br
% https://www.youtube.com/watch?v=5XrNWSngeQ4
% https://ieeexplore.ieee.org/document/8588567

function [x1,x2,theta,e_theta,time,v,w,ux1,ux2] = fuzzyAVF(x1,x2,T,N)
    %INPUT: initial (x,y) coordinates, robot direction (theta), 
    %sampling rate (T) and number of iterations (N)
    
    %OUTPUT: (x1,x2) coordinates of the robot,
    %direction of the robot (theta)
    %errors in x, y and theta, linear and angular control actions
    
    %EXAMPLE: 
    %[x1,x2,theta,erro,time,v,w] = fuzzyAVF(0.001,0.001,0.001,14000)
    
    close all;
    % original AVF
    figure
    syms a b;
    %f = a.^2 + b.^2 - 1;
    f = a.^4/16 - 2*a.^2*b.^2/5 + b.^4 - 1;
    %f = a.^4 + 2*a.^2*b.^2 + b.^4 - 8*a.^2 - 1*b.^2;
    %f = a.^4 - a.^2 + b.^2; 
    subplot(2,2,[3,4])
    h = ezplot(f, [-3,3,-3,3]);
    set(h,'LineWidth',3,'Color','c');
    xlabel('x (m)');
    ylabel('y (m)');
    hold on;
    
    x1 = [x1 zeros(1,N-1)];
    x2 = [x2 zeros(1,N-1)];
    theta = zeros(1,N);
    w = zeros(1,N);
    ux1 = zeros(1,N);
    ux2 = zeros(1,N);
    theta_d = zeros(1,N);
    e_theta = zeros(1,N);
    plot(x1(1),x2(1),'*');
    theta(1) = (2*rand-1)*pi; % degree to rad
    
    tic;
    for i=1 : N-1
        %ux1(i) = (-((x1(i)).^4 + (x2(i)).^4 -1)*4*(x1(i)).^3 - 4*(x2(i)).^3);
        %ux2(i) = (-((x1(i)).^4 + (x2(i)).^4 -1)*4*(x2(i)).^3 + 4*(x1(i)).^3);
        
        %ux1(i) = (-((x1(i)).^2 + (x2(i)).^2 -0.7)*2*(x1(i)).^1 - 2*(x2(i)).^1);
        %ux2(i) = (-((x1(i)).^2 + (x2(i)).^2 -0.7)*2*(x2(i)).^1 + 2*(x1(i)).^1);
        
        ux1(i) = (-((x1(i)).^4/16 -(2*x1(i).^2*x2(i).^2)/5 + (x2(i)).^4 -1)*((4*x1(i).^3)/16 - (4*x1(i)*x2(i).^2)/5) - (-4*x1(i).^2*x2(i)/5 + 4*x2(i).^3));
        ux2(i) = (-((x1(i)).^4/16 -(2*x1(i).^2*x2(i).^2)/5 + (x2(i)).^4 -1)*((-4*x1(i).^2*x2(i))/5 + 4*x2(i).^3) + ((4*x1(i).^3)/16 - (4*x1(i)*x2(i).^2)/5));
        
        %ux1(i) = (-(x1(i).^4 + 2*x1(i).^2*x2(i).^2 + x2(i).^4 -8*x1(i).^2 - x2(i).^2)*(4*x1(i).^3 + 4*x1(i)*x2(i).^2 - 16*x1(i)) - (4*x1(i)^2*x2(i) + 4*x2(i).^3 - 2*x2(i)));
        %ux2(i) = (-(x1(i).^4 + 2*x1(i).^2*x2(i).^2 + x2(i).^4 -8*x1(i).^2 - x2(i).^2)*(4*x2(i).^3 + 4*x1(i).^2*x2(i) - 2*x2(i)) + (4*x1(i).^3 + 2*x1(i).^2*x2(i) - 16*x1(i)));                    
        
        %ux1(i) = (-(x1(i).^4 - x1(i).^2 + x2(i).^2)*(4*x1(i).^3 - 2*x1(i)) - 2*x2(i)); 
        %ux2(i) = (-(x1(i).^4 - x1(i).^2 + x2(i).^2)*(2*x2(i)) + (4*x1(i).^3 - 2*x1(i)));

        normux1(i) = ux1(i)/norm([ux1(i) ux2(i)]);
        normux2(i) = ux2(i)/norm([ux1(i) ux2(i)]);
        
        desired_velocity = sqrt(normux1(i).^2 + normux2(i).^2);
        
        theta_d(i) = atan2(ux2(i),ux1(i));
        e_theta(i) = theta_d(i) - theta(i); 
        e_theta(i) = atan2(sin(e_theta(i)),cos(e_theta(i)));

        [w(i), v(i)] = fuzzyControl(desired_velocity,e_theta(i));
        
        x1(i+1) = x1(i)+T*v(i)*cos(theta(i));
        x2(i+1) = x2(i)+T*v(i)*sin(theta(i));
        theta(i+1) = theta(i) + T*w(i);
        if mod(i,500) == 0
            subplot(2,2,[3,4])
            plot(x1(i),x2(i),'or');
            quiver(x1(i),x2(i),normux1(i),normux2(i),'--r');
        end
    end
    
    %subplot(2,2,[3,4])
    time = toc;
    %hold on
    %plot(x1,x2,'-k')
    %xlim([-3.5 3.5])
    %ylim([-3.5 3.5])
    
    
    %title('c) Tracking Simulation','FontSize',14)
    %leg = zeros(3, 1);
    %leg(1) = plot(NaN,NaN,'c');
    %leg(2) = plot(NaN,NaN,'-k');
    %leg(3) = plot(NaN,NaN,'or');
    %ml = legend(leg, 'Real Curve','Tracked Curve','Robot Position');
    %set(ml,'FontSize',14);
    %xlabel('x (m)', 'FontSize', 14);
    %ylabel('y (m)', 'FontSize', 14);
    
    figure; plot(0:T:T*(N-1),e_theta);
    title('Error in \theta', 'FontSize',14)
    xlabel('Time (s)', 'FontSize', 14);
    ylabel('Error (rad/s)', 'FontSize', 14);
    
    %subplot(2,2,1)
    plot(0:T:T*(N-2),v);
    title('a) Linear Velocity Controller', 'FontSize',14)
    xlabel('Time (s)', 'FontSize', 14);
    ylabel('Control Action (m/s)', 'FontSize', 14);
    
    %subplot(2,2,2)
    %plot(0:T:T*(N-1),w);
    %title('b) Angular Velocity Controller', 'FontSize',14)
    %xlabel('Time (s)', 'FontSize', 14);
    %ylabel('Control Action (rad/s)', 'FontSize', 14);
    
    %controladorFuzzy = readfis('controlador.fis');
    
    %figure;
    %[x_x,mf] = plotmf(controladorFuzzy,'input',1);
    %subplot(2,1,1), plot(x_x,mf)
    %xlabel('e_w', 'FontSize', 18)
    %[x_x,mf] = plotmf(controladorFuzzy,'input',2);
    %subplot(2,1,2), plot(x_x,mf)
    %xlabel('e_v', 'FontSize', 18)
    
    %figure;
    %[y_y,mf] = plotmf(controladorFuzzy,'output',1);
    %subplot(2,1,1), plot(y_y,mf)
    %xlabel('e_{f\omega}', 'FontSize', 18)
    %[y_y,mf] = plotmf(controladorFuzzy,'output',2);
    %subplot(2,1,2), plot(y_y,mf)
    %xlabel('e_{fv}', 'FontSize', 18)
    
end

function [control_w, control_v] = fuzzyControl(erro_v, erro_theta)
    fuzzyController = readfis('controlador.fis');
    if erro_theta > 1
        erro_theta = 1;
    elseif erro_theta < -1
        erro_theta = -1;
    end
    
    if erro_v > 1
        erro_v = 1;
    elseif erro_v < -1
        erro_v = -1;
    end
    fuzzyVal = evalfis([erro_v erro_theta], fuzzyController);
    control_w = 10*fuzzyVal(1,1);
    control_v = fuzzyVal(1,2);
end