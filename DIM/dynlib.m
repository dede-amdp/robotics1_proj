classdef dynlib
    methods
        %{
#@
@name: vand
@brief: computes the vector [1 t ... t^i ... t^n]
@inputs:
- t: value used for the computations;
- n: number of elements in the vector;
@outputs:
- M: vector of the computed values;
@#
%}
        function M = vand(obj, t, n)
            M = [1];
            for i = 1:n
                M = [M, t^i];
            end
        end
        
%{
#@
@name: devand
@brief: compues the derivative of the vector computed by `vand` [0
1 ... it^(i-1) ... nt^(n-1)]
@inputs:
- t: value used for the computations;
- n: number of elements in the vector;
@outputs:
- M: vector of the computed values;
@#
%}
        function M = devand(obj, t, n) 
            M = [0 1];
            for i = 2:n
                M = [M i*t^(i-1)];
            end
        end

%{
#@
@name: dedevand
@brief: computes the derivative of the vector computed by `devand` [0 0 2
... i(i-1)t^(i-2) ... n(n-1)t^(n-2)]
@inputs: 
- t: value used for the computations;
- n: number of elements in the vector;
@outputs:
- M: vector of the computed values;
@#
%}    
        function M = dedevand(obj, t, n) 
            M = [0 0 2];
            for i=3:n
                M = [M i*(i-1)*t^(i-2)];
            end
        end

%{
#@
@name: polynomial
@brief: computes a generic polynomial trajectory
@inputs:
- q: values that q has to take;
- q_dot: values that the derivative of q has to take;
- q_ddot: values that the second derivative of q has to take;
- t: instants at which q has to take the values given as input;
- t_dot: instants at which the derivative of q has to take the values
given as input;
- t_ddot: instants at which the second derivative of q has to take the
values given as input;
@outputs:
- A: coefficients of the polynomial;
- V: vandermont matrix;
@#
%}

        function [A, V] = polynomial(obj, q, q_dot, q_ddot, t, t_dot, t_ddot)
            % computes the coefficients of the polynomial that goes through
            % the points given as input
            n = length(q)+length(q_dot)+length(q_ddot);
            A = zeros(n,1);
            B = [q;q_dot;q_ddot];
            V = zeros(n);
            for i=1:length(q)
                V(i,:) = obj.vand(t(i), n-1);
            end
            for i=1:length(q_dot)
                V(i+length(q),:) = obj.devand(t_dot(i), n-1);
            end
            for i=1:length(q_ddot)
                V(i+length(q)+length(q_dot),:) = obj.dedevand(t_ddot(i), n-1);
            end
            A = (V^-1)*B;
        end

%{
#@
@name: criticaltrajectory
@brief: it computes the trajectory that is used during the dimensioning
simulations for a 2Dofs planar robotic arm
@inputs:
- tf: duration of the trajectory;
- amax: maximum acceleration reached at time tf/4;
@outputs:
- A: coefficients of the polynomial trajectory;
- vandermont: vandermont matrix;
@#
%}

        function [A, vandermont] = criticaltrajectory(obj, tf, amax)
            % criticaltrajectory computes the trajectory that is used
            % during the dimensioning simulations for the 2 Dofs planar
            % robotic arm
            [A, vandermont] = obj.polynomial([-pi/2; 0; pi/2], [0; 0], [0; amax; 0], [0, tf/2, tf], [0, tf], [0, tf/4, tf]);
        end




        function [manip, links, q, dq] = init(obj,lx,ly,lz,rho,mm,Im,kr)
            syms('q1', 'q2', 'dq1', 'dq2');
            q = [q1; q2];
            dq = [dq1; dq2];
            
            mass=[rho*lx(1)*ly(1)*lz(1) rho*lx(2)*ly(2)*lz(2)];
            links = [Link([0 0 0 0])];
            
            links(1) = Link('d', 0,'alpha', 0,'a', lx(1));
            links(2) = Link('d', 0,'alpha', 0,'a', lx(2));
            
            links(1).m = mass(1)+2*mm(2);
            links(2).m = mass(2);
            
            % CoG
            links(1).r = [lx(1)/2*cos(q1); lx(1)/2*sin(q1); 0];
            links(2).r = [lx(1)*cos(q1)+lx(2)/2*cos(q1+q2); lx(1)*sin(q1)+lx(2)/2*sin(q1+q2); 0];
            
            for i=1:2
                Cx = links(i).r(1); % x coordinate of CoG
                I = lx(i)*ly(i)*lz(i)*rho*((ly(i)^2+lx(i)^2)/12+Cx);
                links(i).I = [0 0 0; 0 0 0; 0 0 I]; % link inertia
                links(i).Jm = Im(i); % motor inertia
                links(i).G = kr(i); % gear ratio
            end
            
            manip = SerialLink(links, 'name', 'Peter Cock');

            % TODO: motor masses? -> do we add them to the links?
        end

        function [B, C, g, q, dq] = dyn(obj,lx,ly,lz,rho,mm,Im,kr)
            [manip, ~, q, dq] = obj.init(lx,ly,lz,rho,mm,Im,kr);
            B = manip.inertia(q.');
            C = manip.coriolis([q.' dq.']);
            g = manip.gravload(q.',[0;0;9.81].').';
            % TODO: dato che g doveva essere trasposto, non è che anche C
            % deve essere trasposta? B è simmetrico quindi è indifferente
        end
    end
end