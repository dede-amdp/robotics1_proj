function [B, C, g] = generateBCg(lx, ly, lz, rho, mm, Im, kr, qt, dqt)
%[lx, ly, lz, rho, mm, Im, kr] = deal(links_params{:});
if ~((exist('Bq','var') == 1) && (exist('Cq','var') == 1) && (exist('gq','var') == 1))
    % evita di ricreare ogni volta le variabili simboliche e le batrici Bq,
    % Cq e lo scalare gq
    methods = dynlib;
    syms("q1", "q2", "dq1", "dq2");
    q = [q1, q2];
    dq = [dq1, dq2];
    [Bq, Cq, gq] = methods.dyn(lx, ly, lz, rho, mm, Im, kr);
end
B = double(subs(Bq, q, qt'));
C = double(subs(Cq, [q, dq], [qt', dqt']));
g = double(subs(gq, q, qt'));
end