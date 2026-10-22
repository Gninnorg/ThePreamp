# Test af ThePreamp

## Automatisk (web API og MQTT)

```powershell
.\tools\test-preamp.ps1 -Target 192.168.1.145
.\tools\test-preamp.ps1 -Target 192.168.1.145 -IncludePower                 # tester også tænd/sluk (standby)
.\tools\test-preamp.ps1 -Target 192.168.1.145 -MqttHost <broker> -BaseTopic thepreamp
```

Scriptet gendanner input, volumen og mute efter testen. Det kan ikke se displayet eller høre lyd, så brug listen nedenfor til resten.

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
