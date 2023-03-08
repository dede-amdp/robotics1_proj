function [Ja] = jacobianFnc(lx, qt)
    if ~(exist('J', 'var') == 1)
        methods = dynlib;
        syms("q1", "q2");
        q = [q1,q2];
        J = methods.analyticaljacobian(lx);
    end
    Ja = double(subs(J, q, qt'));
end