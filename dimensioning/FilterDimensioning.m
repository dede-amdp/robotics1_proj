
s=tf('s');
z=tf('z',0.0002);
wc= (1000*pi)/2;

G=(wc/(s+wc))

%G=1/((s+1)^2);
[A,B]=tf2ss(wc,[1 wc]);

bode(G)

Gz=c2d(G,0.0002,'tustin')

syms z

iztrans((0.1358*z + 0.1358)/(z - 0.7285))

