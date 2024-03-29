classdef dynlib
%{
#@
@name: dynlib
@brief: support class for the dynamical model of a 2Dofs planar robotic
arm.
@notes: this class exposes all the methods used to compute the dynamical
model of a 2Dofs planar robotic arm so that it may be used as a library.
@#
%}
    

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
@name: symzeros
@brief: returns a nxm symbolic null matrix
@inputs:
- n: number of rows;
- m: number of columns;
@outputs:
- M: symbolic null matrix;
@#
%}
        function M = symzeros(obj, n, m)
            syms("x", "real");
            M = [x];
            M(1,1) = 0;
            for i = 1:n
                for j = 1:m
                    M(i,j) = 0;
                end
            end
        end

%{
#@
@name: centerofmass
@brief: computes the center of mass of a box of size (lx, ly, lz) with
density rho
@notes: Cx = \int_V rho x dV, Cy = \int_V rho y dV, Cz = \int_V rho z dV
@inputs:
- lx: length relative to the x axis;
- ly: length relative to the y axis;
- lz: length relative to the z axis;
- rho: density of the material;
@outputs:
- Cx: x coordinate of the center of mass;
- Cy: y coordinate of the center of mass;
- Cz: z coordinate of the center of mass;
@#
%}

        function [Com] = centerofmass(obj, lx )
            % centerofmass computes the center of a box of size (lx, ly, lz) with constant
            % density rho

            [~,~,p1]=obj.dh([lx(1)/2 ,0]);
            [~,~,p2]=obj.dh([lx(1) , lx(2)/2]);
            Com=[ p1 ,p2];

        end

%{
#@
@name: linkjacobian
@brief: computes the i-th jacobian of the links of a 2 Dofs planar robotic
arm with sizes (lx, ly, lz) with density rho, relative to the center of
mass of the specified link.
@notes: J_prismatic,i = [z(i-1);0], J_rotoidal,i =
[z(i-1)x(p-p(i-1));z(i-1)]
@inputs:
- i: index of the link relative to which the jacobian will be computed;
- lx: array of lengths relative to the x axis of all the links of the
robotic arm;
- ly: array of length relative to the y axis of all the links of the
robotic arm;
- lz: array of length relative to the z axis of all the links of the
robotic arm;
- rho: density of the material of the links;
@outputs:
- J: jacobian matrix;
@#
%}

        function [J]=linkjacobian(obj, i, lx)
            % linkjacobian computes the i-th jacobian of the links of a 2 Dof
            % planar robotic arm with sizes (lx, ly, lz), density rho
            syms("q1", "q2", "real");
            q = [q1,q2];

            Com= obj.centerofmass(lx);
            Cx=Com(1,i);
            Cy=Com(2,i);


            J = obj.symzeros(6,2);
            J(1,1)= -Cy;
            J(2,1)= Cx;
            J(6,1)= 1;
        
            if(i==2)
                J(1,2)=lx(i)*sin(q(1))-Cy;
                J(2,2)=Cx-lx(i)*cos(q(1));
                J(6,2)=1;
            end
        end

%{
#@
@name: linkinertia
@brief: computes the intertia of the i-th link of a 2Dofs robotic arm
@inputs:
- i: index of the link relative to which the intertia will be computed;
- lx: array of lengths relative to the x axis of the links;
- ly: array of lengths relative to the y axis of the links;
- lz: array of lengths relative to the z axis of the links;
- rho: density of the material of the link;
@outputs:
- inertia: computed inertia;
@#
%}
        function [inertia] = linkinertia(obj, i, lx, ly, lz, rho)
            % computes the intertia of the i-th link of a 2 Dof robotic arm
            Com= obj.centerofmass(lx);
            Cx=Com(1,i);
          


            %inertia = lx(i)*ly(i)*lz(i)*rho*((ly(i)^2+lx(i)^2)/12+Cx);
            Iz = lx(i)*ly(i)*lz(i)*rho*((ly(i)^2+lx(i)^2)/12+Cx);
            inertia= [0 0 0; 0 0 0; 0 0 Iz];
            
        end

%{
#@
@name: dh
@brief: computes the rotation and position matrix of the end effector of a
2Dofs planar robotic arm
@notes: computes the rotation and position matrix based on the
Denavit-Hartenberg Convention, also returns the T matrix computed as
follows: T = [R p;0 0 0 1]
@inputs:
- lx: array of lengths of the links relative to the x axis;
@outputs:
- T: Homogeneous Transformation Matrix;
- R: Rotation matrix;
- p: position vector;
@#
%}

        function [T, R, p] = dh(obj, lx)
            % computes the position and rotation matrices by using the
            % dh convention
            syms("q1","q2", "real");
            q=[q1;q2];
            q12=q(1)+q(2);
            c12=cos(q12);
            s12=sin(q12);

            R=[c12, -s12, 0;
               s12 c12 0 ;
               0 0 1 ];
            p =[lx(1)*cos(q(1))+lx(2)*c12 ; lx(1)*sin(q(1))+lx(2)*s12 ; 0];
            T = [R, p; 0 0 0 1];
        end

%{
#@
@name: joint2op
@brief: computes the transformation matrix T(phi)
@inputs: 
NONE
@outputs:
- T: transformation matrix;
@#
%}

        function [T] = joint2op(obj)
            % joint2op computes the matrix needed to convert the jacobian
            % from the joint space to the operational space
            syms("q1","q2", "real");
            q=[q1;q2];
            q12=q(1)+q(2);
            c12=cos(q12);
            s12=sin(q12);
        
            T=[ 0 0 s12 ;
                0 1 0 ;
                1 0 c12];
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


%{
#@
@name: motorjacobian
@brief: computes the jacobian of the motors of a 2Dofs planar robotic arm
@inputs:
- i: index of the motor;
- lx: array of lengths of the links relative to the x axis;
@outputs:
- J: jacobian;
@#
%}

        function [J] = motorjacobian(obj, i, lx, kr)
            % motorjacobian computes the jacobian of the motors of a 2 Dofs
            % planar robotic arm
            syms("q1","q2", "real");
            q=[q1;q2];
            J = obj.symzeros(6,2);
            J(6,1)= kr(i);
            
            if(i==2)
                J(1,1)=-lx(1)*sin(q(1));
                J(2,1)=lx(1)*cos(q(1));
                J(6,1)= 1;
                J(6,2)= kr(i);
            end
        end

%{
#@
@name: dyn
@brief: computes the dynamical model of a 2Dofs planar robotic arm
@notes: the method computes the dynamical model of a 2Dofs planar robotic
arm

B(q)q:+C(q,q.)q.+g(q)+Fq.=u-J'h

More in details, the function returns the matrices B(q), C(q.) and the
value g(q) as symbolic matrices in q1, q2, dq1 and dq2 so that they can be
adapted to any pose the robotic arm may have.
@inputs:
- lx: array of lengths of all the links relative to the x axis;
- ly: array of lengths of all the links relative to the y axis;
- lz: array of lengths of all the links relative to the z axis;
- rho: density of the material;
- mm: array of masses of the motors;
- Im: array of inertias of the motors;
- kr: array of reduction ratios of the motors;
@#
%}

        function [B, C, g] = dyn(obj, lx, ly, lz, rho, mm, Im, kr)
            % dym computes the dynamical model of a 2 Dofs plana robotic
            % arma as matrices B(q), C(q, q_dot) and component g(q) as
            % symbolic matrices

            syms("q1","q2", "dq1", "dq2", "real");
            q = [q1; q2];
            dq = [dq1; dq2];
            B = obj.symzeros(2,2);
            C = obj.symzeros(2,2);
            g = obj.symzeros(1,1);
            Rot = @(q) [cos(q), -sin(q), 0; sin(q), cos(q), 0; 0 0 1];
            %{
            for l=1:2
                % inertia matrix
                % kinetic energy of the link
                Jl = obj.linkjacobian(l,lx);
                J_p = Jl(1:3,:);
                J_o = Jl(4:end,:);
                R = eye(3);
                if(l == 2)
                    R = Rot(q(1)+q(2));
                else
                    R = Rot(q(1));
                end
                m=lx(l)*ly(l)*lz(l)*rho;
                I = obj.linkinertia(l,lx,ly,lz,rho);
                B = simplify(B + m*(J_p.')*J_p + (J_o.')*R*I*(R.')*J_o);
                
                % kinetic energy of the motor
                Jm = obj.motorjacobian(l, lx,kr);
                J_pm = Jm(1:3,:);
                J_om = Jm(4:end,:);
                I_m = diag([0 0 Im(l)]);
                                
                B = simplify(B + mm(l)*(J_pm.')*J_pm+(J_om.')*R*I_m*(R.')*J_om);% *kr(l)^2;
                
                

                % gravity component
                %g = g-m*[0 0 9.81]*J_p(:,l)-mm(l)*[0 0 9.81]*J_pm(:,l);

                
                
                %{
                TODO:
                - facoltativo: aggiungere modellazione dell'attrito
                %}
                
            end 
            %}
            
            for i=1:2
                Jl = obj.linkjacobian(i,lx);
                Jp = Jl(1:3,:);
                Jo = Jl(4:end,:);
                R = eye(3);
                if(i == 2)
                    R = Rot(q(1)+q(2));
                else
                    R = Rot(q(1));
                end
                ml=lx(i)*ly(i)*lz(i)*rho;
                Il = obj.linkinertia(i,lx,ly,lz,rho);
                Jm = obj.motorjacobian(i, lx,kr);
                Jpm = Jm(1:3,:);
                Jom = Jm(4:end,:);
                I_m = diag([0 0 Im(i)]);

                B=B+ml*(Jp.')*Jp+(Jo.')*R*Il*(R.')*Jo+mm(i)*(Jpm.')*Jpm+(Jom.')*R*I_m*(R.')*Jom;
            end
            
            % gravity component
            g=obj.gravity(lx,ly,lz,rho, mm);
            
            % coriolis matrix

            for i=1:2
                for j=1:2
                    C(i,j)=0;
                    for k=1:2
                        C(i,j)=simplify(C(i,j)+0.5*(diff(B(i,j), q(k))+diff(B(i,k),q(j))-diff(B(j,k),q(i)))*dq(k)); %mod
                    end
                end
            end

        end

        function [J] = geometricjacobian(obj, lx)
            syms("q1", "q2", "real");
            q = [q1,q2];
            q12=q(1)+q(2);

            J = obj.symzeros(6,2);
            J(1,1)= -lx(1)*sin(q(1))-lx(2)*sin(q12);
            J(2,1)= lx(1)*cos(q(1))+lx(2)*cos(q12);
            J(6,1)= 1;
        
            J(1,2)=-lx(2)*sin(q12);
            J(2,2)=lx(2)*cos(q12);
            J(6,2)=1;
            

        end

        function [Ja] = analyticaljacobian(obj, lx)
            T = obj.joint2op();
            Ta = [eye(3), zeros(3);zeros(3), T];
            J = obj.geometricjacobian(lx);
            Ja = (Ta^-1)*J;
        end

        function [pos] = dk(obj, lx, q)
            x = lx(1)*cos(q(1))+lx(2)*cos(q(1)+q(2));
            y = lx(1)*sin(q(1))+lx(2)*sin(q(1)+q(2));
            phi = q(1)+q(2);
            pos = [x;y;0;phi;0;0]; % the rotation is only algon the z axis so the other angles are 0
        end

        
        function [ gq ] =gravity(obj, lx ,ly ,lz ,rho, mm)
        
         syms("q1","q2", "real");
         q=[q1 q2];

         q12=q(1)+q(2);

        Com= obj.centerofmass(lx);
        cx1=Com(1,1);
        cx2=Com(1,2);
       
        x1=cx1*cos(q(1));
        x2=lx(1)*cos(q(1))+cx2*cos(q12);

        y1=cx1*sin(q(1));
        y2= lx(1)*sin(q(1))+cx2*sin(q12);
           
        pl1=[ x1 ; y1; 0];
        pl2=[ x2 ; y2; 0];

        pl= {pl1 ; pl2};
       

        pm1=obj.symzeros(3,1);
        pm2=[lx(1)*cos(q(1)); lx(1)*sin(q(1)); 0];

        pm= {pm1 ; pm2};

        gq=obj.symzeros(2,1);
        ml1=lx(1)*ly(1)*lz(1)*rho;
        ml2=lx(2)*ly(2)*lz(2)*rho;
        ml = [ml1;ml2];


        for i=1:2
                
            for j=1:2
                
                gq(i)=gq(i)+ml(j)*[0 0 9.81]*diff(pl{j},q(i))+mm(j)*[0 0 9.81]*diff(pm{j},q(i));

            end
        end

        end 


        function [q] = ik(obj, x, lx )

            c2=(x(1)^2+x(2)^2-lx(1)^2-lx(2)^2)/(2*lx(1)*lx(2));
            s2=sqrt(1-c2^2);


            q2=atan2(s2,c2);
            q1=x(4)+x(6)-q2;

            q=[q1; q2];

        end
    end
end