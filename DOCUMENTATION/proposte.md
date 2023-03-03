# Proposta di Progetto
## Descrizione

L'obiettivo del progetto è realizzare un manipolatore planare a due bracci con controllo centralizzato nello spazio dei giunti che possa eseguire, disegnando con un pennarello su una whiteboard, delle traiettorie specificate dall'utente tramite una semplice interfaccia grafica e l'uso di una serie di primitive geometriche. 

## Requisiti
1. Realizzazione di un software per tracciare le traiettorie (nel piano) che il manipolatore dovrà eseguire (+ GUI);
2. Realizzazione di una interfaccia di comunicazione tra software di trajectory planning e il controller hardware basata sullo scambio di messaggi tramite interfaccia seriale;
3. Modellazione cinematica del manipolatore;
4. Prototipazione e Realizzazione del manipolatore;
5. Realizzazione di uno schema di controllo centralizzato basato su PD con compensazione di gravità (nello spazio dei giunti);
6. Implementazione di più metodologie di calcolo della traiettoria delle grandezze di giunto;
7. Calcolo e visualizzazione dell'errore tra la traiettoria desiderata e quella effettiva in real time;

## Requisiti Non-Funzionali

1. Utilizzo di 2 stepper con encoder;
2. Utilizzo di microcontrollore della famiglia stm32;
3. Definizione ed implementazione di una procedura di Homing;
4. Custom Firmware scritto in C per il controllo del manipolatore;
5. Interfaccia grafica realizzata in HTML e JS;
6. Middleware comunicativo in Python;

> Riferimenti: https://youtube.com/shorts/1ySjyYLnu_4?feature=share