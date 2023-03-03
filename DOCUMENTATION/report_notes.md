# Report Notes

## 03/03/23
Per effettuare il dimensionamento dei bracci del braccio robotico oggetto del tema d'anno sono stati realizzati alcuni metodi di supporto atti a facilitare la scrittura ed esecuzione di simulazioni del modello dinamico del sistema che permettano di avere dei bracci di dimensioni generiche.

In particolare, i metodi scritti generano le matrici B, C ed il valore g(q) **simbolici** così che tramite il metodo `subs` presente nel symbolic toolbox di matlab sia possibile derivare i corretti valori delle matrici B e C e del valore g(q) basandosi sulla posa corrente del manipolatore.

Il dimensionamento sarà effettuato tramite ripetute simulazioni in cui saranno imposti degli specifici valori delle lunghezze lx, ly e lz dei bracci così da valutare il comportamento del modello durante le simulazione e stabilire se le suddette dimensioni possano essere utilizzate per la effettiva costruzione del braccio robotico.

La simulazione prevede di controllare il manipolatore (rappresentato dal suo modello dinamico) in modo tale che esso possa seguire una traiettoria, detta critica, prestabilita. Valutata la fedeltà della effettiva traiettoria rispetto alla traiettoria desiderata, le dimensioni che permettono una maggiore fedeltà saranno selezionate.

riferimenti: <a href='./dynlib_documentation.md'>`./DOCUMENTATION/dynlib_documentation.md`</a>