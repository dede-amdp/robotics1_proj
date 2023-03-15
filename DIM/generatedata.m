function [B, C, g] = generatedata(lx,ly,lz,rho,mm,Im,kr, qt,dqt)
    if exist('B', 'var') == 0
        methods = dynlib;
        [Bq, Cq, gq, q, dq] = methods.dyn(lx,ly,lz,rho,mm,Im,kr);
    end
    B = double(subs(Bq, q.', qt.'));
    C = double(subs(Cq, [q.', dq.'], [qt.' dqt.']));
    g = double(subs(gq, q.', qt.'));
end