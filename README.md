# NSS
The Night Surveillance System.
Progettazione e realizzazione di un prototipo di dispositivo di sorveglianza notturna composto da:
* Camera termica. Griglia di sensori IR dotata di lente, con la quale acquisire immagini in notturna o mappe di calore dell'ambiente circostante. 
* Motore stepper. Consente l'orientamento della camera a 180° sull'azimuth.
* Coppia di microfoni a elettrete. Posti frontalmente e a distanza reciproca nota, consentono l'acquisizione dell'audio ambientale in stereo tramite conversione ADC.
* Scheda di sviluppo STMicroelectronics STM32F407G-DISC1 (CortexM4).

Il dispositivo ha due modalità di funzionamento: 
* Manuale. Gestisce simultaneamente streaming  real-time dell'immagine termica su porta seriale, acquisizione e playback audio ambientale su uscita analogica in stereo e controllo dell'orientamento manuale della camera tramite joystick analogico.
* Follow. Oltre a provvedere a streaming audio e video, il dispositivo reagisce ad un eventi sonori sopra soglia RMS, orientando la camera verso un target termico, possibile fonte di rumore. 
