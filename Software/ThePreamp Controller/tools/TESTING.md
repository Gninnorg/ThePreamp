# Test af ThePreamp

## Automatisk (web API og MQTT)

```powershell
.\tools\test-preamp.ps1 -Target 192.168.1.145
.\tools\test-preamp.ps1 -Target 192.168.1.145 -IncludePower                 # tester også tænd/sluk (standby)
.\tools\test-preamp.ps1 -Target 192.168.1.145 -MqttHost <broker> -BaseTopic thepreamp
.\tools\test-preamp.ps1 -Target 192.168.1.145 -IncludeDisplay                # webcam + OCR af displaytekst
.\tools\test-preamp.ps1 -Target 192.168.1.145 -IncludeDisplay -CameraIndex 1 # vælg et andet webcam (overstyrer kalibreringen)
```

Kalibrér webcam'et, hver gang det er flyttet (preamp'en skal være tændt). Et live-billede åbnes; placér kameraet, så begge displays er skarpe og uden genskin, tryk `1`/`2` og træk en boks omkring venstre/højre display (piletaster flytter, `+`/`-` ændrer størrelse, `r` nulstiller, `s` gemmer, `q` afslutter):

```powershell
python .github\skills\preamp-webcam-display-test\scripts\calibrate_camera.py --camera-index 0
```

Kalibreringen gemmes i `camera_calibration.json` (plus et referencebillede, begge ikke i git) og bruges automatisk af displaytesten. Ved start af `-IncludeDisplay` kontrolleres kalibreringen automatisk (`calibrate_camera.py --check`): mangler den, er et display ikke tændt i sin boks, ligger der lys uden for boksen, eller er kameraets billede forskudt mere end 1 % siden kalibreringen, åbnes kalibreringsvinduet automatisk, og testen fortsætter, når du har gemt (`s`) og lukket (`q`). Skærmspareren vækkes automatisk via API'et (tester sætter volumen til den nuværende værdi), også mens kalibreringsvinduet er åbent.

Scriptet gendanner input, volumen og mute efter testen. Displaytesten kræver et webcam på test-PC'en, Python med OpenCV og Tesseract OCR på `PATH` (eller angivet med `-TesseractPath`). Installer OpenCV fra projektroden:

```powershell
python -m pip install -r .github\skills\preamp-webcam-display-test\requirements.txt
```

Displaytesten (`-IncludeDisplay`) bruger webcam'et til at kontrollere, hvad de to displays faktisk viser. Den dækker (scenarierne ligger i `tools\display-tests.ps1`):

- Input-navn på venstre display og volumen på højre display for hvert aktivt input (HT passthrough viser ingen volumen)
- Volumen som trin (`DisplayVolume` = 1), som dB (2) og skjult (0)
- Mute: højttaler-ikon i stedet for volumen (fjernbetjening/web og `/MUTE`, `/UNMUTE`)
- WiFi-signal og temperaturindikatorer til/fra (`DisplayTemperature1/2`)
- Skærmsparer: slukker efter `DisplayTimeout`, vågner ved betjening og forbliver tændt, når den er slået fra
- `DisplaySelectedInput` slået fra: venstre display viser intet navn
- Med `-IncludePower` også: displays slukket i standby, "Wait... N"-countdown under trigger-forsinkelse (kun hvis en trigger er aktiv) og korrekt visning efter opstart

Testen ændrer midlertidigt indstillinger (fx `DisplayTimeout`, `DisplayVolume`) og gendanner dem bagefter. Ved fejl gemmes billederne i Windows' midlertidige mappe (stien skrives i outputtet). Kameraet optager i 1920x1080 og skal vise begge displays tydeligt uden genskin; ROI-områderne for displayene sættes med kalibreringen ovenfor, hvis kameraet flyttes. Se også skillen [preamp-webcam-display-test](../.github/skills/preamp-webcam-display-test/SKILL.md).

Ikke dækket af webcam-testen (kan ikke udløses via API'et): balance-skærmen og infoskærmene (IP/QR, build-datoer), samt lysstyrke (`DisplayOnLevel`/`DisplayDimLevel`).

## Manuel tjekliste

Sæt kryds for hver kontrol efter en ny flash.

### Mute
- [ ] Mute-tast på fjernbetjening viser højttaler-ikon med kryds på højre display og slår lyden fra
- [ ] Mute igen fjerner ikonet og lyden kommer tilbage
- [ ] Mute fra webside (`/remote`) opdaterer displayet
- [ ] `/MUTE` og `/UNMUTE` opdaterer displayet
- [ ] Mute fra MQTT / Home Assistant opdaterer displayet
- [ ] Mute-niveau (`MuteLevel`) i indstillinger giver den forventede dæmpning

### Volumen
- [ ] Encoder op/ned ændrer volumen i ét trin ad gangen og tallet vises
- [ ] Volumen stopper ved min/max for det valgte input
- [ ] Volumen under mute kan ikke ændres
- [ ] Visning efter `DisplayVolume` (0 = ingen, 1 = trin, 2 = dB)
- [ ] HT passthrough viser ingen volumen

### Input
- [ ] Alle aktive inputs 1-5 kan vælges med fjernbetjening, encoder og web
- [ ] Inaktive inputs springes over
- [ ] Relæerne skifter uden hørbart klik eller pop
- [ ] Navnet vises på venstre display
- [ ] `RecallSetLevel` husker volumen pr. input

### Balance
- [ ] SELECT-tasten åbner balance, visningen er korrekt og midten er 127
- [ ] Balance gemmes pr. input

### Standby og start
- [ ] Tænd/sluk med fjernbetjening og web
- [ ] Displays slukker i standby
- [ ] Trigger 1 og 2 følger forsinkelser og type
- [ ] Eksternt strømrelæ (`ExtPowerRelayTrigger`) følger tænd/sluk
- [ ] Efter tænd er volumen højst `MaxStartVolume`
- [ ] Countdown "Wait... N" vises under trigger-forsinkelse

### Display
- [ ] Skærmspareren starter efter `DisplayTimeout` og vågner ved betjening
- [ ] Lysstyrke `DisplayOnLevel` og `DisplayDimLevel`
- [ ] WiFi-signalstyrke og temperaturer vises efter indstilling
- [ ] Infoskærm: IP/QR-kode og build-datoer

### Indstillinger og lagring
- [ ] Gem indstillinger på `/settings.html`, genstart, og værdierne er bevaret
- [ ] Sidste input, volumen og balance huskes efter strømafbrydelse
- [ ] Ugyldige værdier afvises eller afgrænses

### IR
- [ ] Alle indlærte IR-koder virker (`/ir.html`)
- [ ] Indlær, anvend og ryd en kode

### Netværk og opdatering
- [ ] WiFi-forbindelse og fallback-AP/QR ved manglende netværk
- [ ] `/update` åbner, og OTA af firmware og filsystem virker
- [ ] MQTT: Home Assistant finder enheden og viser power, mute, volumen og input
