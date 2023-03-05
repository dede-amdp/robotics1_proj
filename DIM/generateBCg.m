function [B, C, g] = generateBCg(lx, ly, lz, rho, mm, Im, kr, qt, dqt)
methods = dynlib;
syms("q1", "q2", "dq1", "dq2");
q = [q1, q2];
dq = [dq1, dq2];
%[lx, ly, lz, rho, mm, Im, kr] = deal(links_params{:});
[Bq, Cq, gq] = methods.dyn(lx, ly, lz, rho, mm, Im, kr);
B = double(subs(Bq, q, qt'));
C = double(subs(Cq, [q, dq], [qt', dqt']));
g = double(subs(gq, q, qt'));
end