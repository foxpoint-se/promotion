---
title: Anteckningar Wireguard
description: Helt osorterade anteckningar om arbete med Wireguard o dyl.
tags:
  - Wireguard
  - ROS2
draft: true
date: 2023-09-03
---

====

steg hära

- starta en ubuntu server
  - typ lightsail
  - ge den ett bra namn, typ "wireguard server"
  - gärna statisk IP också. eller domännamn för den delen
  - öppna för UDP på en port (kanske efter du kört wg-quick? för att ta dess port som den säger)
  - se till att ssh port 22 tcp är öppen också
- connecta till den. lägg in privat nyckel från dig själv så du kommer in
- kör på servern

sudo apt update
sudo apt upgrade

- använd wg-quick för enkelhets skull:
  - angristan wireguard install

```bash
curl -O https://raw.githubusercontent.com/angristan/wireguard-install/master/wireguard-install.sh
chmod +x wireguard-install.sh
./wireguard-install.sh
```

- nu kör installeraren
- välj default-värdena. men se till att public ip är samma som du ser i AWS
- i slutet kan man ange en klient
- ge den ett namn. typ: "mycomputer" eller nåt så man fattar vilken klient det är
- det skapar en fil (`ls` i den katalog du står) som heter typ wg0-client-mycomputer.conf
- den ska du kopiera innehållet ifrån. sen kan du ta bort den helt från servern? tror jag
- installera wireguard på din dator
- `sudo apt update`
- `sudo apt upgrade` också kanske
- `sudo apt install wireguard`
- nu kommer du ha en /etc/wireguard mapp
- där ska du placera din klientconf från servern
- skapa en fil /etc/wireguard/wg0.conf
- klistra in innehållet från wg0-client-mycomputer.conf
  - kan lägga till saker från nedan? typ: `PersistentKeepalive`, `PostUp = ping -c1 10.66.66.1`
- NU kan du ta bort den från servern?
- kör `wg-quick up wg0`
- kolla "whats my ip" på google. nu ska du ha samma ip som servern.
- du kan också testa att pinga `10.66.66.1`

(parentes!)
sudo systemctl status wg-quick@wg0 säger dead inactve. men det kanske inte är ett problem på klienten?
kanske måste köra detta på servern? näää, det har nog gjorts av installern.
`sudo systemctl enable wg-quick@wg0.service`

lägg till user:

- kör skript igen `sudo ./wireguard-install.sh`
- välj "add user"
- skriv namn, typ "eel"
- nu är det klart. kolla att fil finns.
- kopiera innehåll från filen
- lägg in i andra datorn under /etc/wireguard/wg0.conf
- starta med `wg-quick up wg0`
- (stoppa sen med `wg-quick down wg0`)
- (här kan vi nog vilja ha en service?!? `sudo systemctl enable wg-quick@wg0.service` och kanske `sudo systemctl start wg-quick@wg0.service`. kolla med `status`)
- prova att pinga från klient till klient

====

saker att göra:

- JAPP kolla på den andra videon duvet: https://www.youtube.com/watch?v=qNdL9iJqyUk
  för att få lite koll på hu man lägger till klienter
- JAPP testa connection mellan data och jevla rpi
  - JAPP pinga
  - JAPP ROSa
- testa samma när jag har datorn på hotspot.
  - strulade när jag bytte mitt i.
  - men om jag provar:
    - starta upp link som vanligt.
    - logga in.
    - sätt skiten som en service, och kolla att den alltid startar.
    - om den gör det, då vet jag att jag borde kunna komma in igen.
    - sätt datan på hotspot. prova att pinga .3 IP
    - ssh:a till den över .3 IP
    - starta upp ROS där. kolla topics
    - sen på lokal burk: kolla topics.
    - kan vara så att det sker över SSH????
- JAPP! testa att ssh:a genom servern. med forwarding på nåt sätt. ska jag ha config på min lokala då? (SKICKA MED -A flagga från första anrop. sen kan jag ssh:a från servern)
- testa med ROS på servern och se hur den uppfattar grejen
- alt. ws på ålen. som servern vidarebefordrar på nåt sätt
- testa med domännamn istället för IP på servern
- kolla den här fogros2 också https://github.com/BerkeleyAutomation/FogROS2
- cdk:a skiten
- etc...

- måste testa från riktiga ålen. dvs över 4g
- kan ju va så att det är min mobilhotspot som ställer till det?
- och då vill man ju veta hur det är med modemet på ålen?
- kan ju förstås testa peer-server, fast från min dator till servern. bara för att se att det funkar

===
såhär kan man få det att funka från Ålen till server (men inte peer-server-peer)

- `sudo apt install ros-foxy-rmw-cyclonedds-cpp` (eller den här kanske också funkar? `sudo apt install ros-rolling-rmw-cyclonedds-cpp`)
- skapa en fil i typ samma mapp `cyclonedds.xml`

```xml
<?xml version="1.0" encoding="UTF-8" ?>
  <CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>wg0</NetworkInterfaceAddress>
        </General>
    </Domain>
</CycloneDDS>
```

- skriv in i interface det som wireguard heter. kolla `ifconfig` för att se.
- `export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml`
- `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `export ROS_DOMAIN_ID=50`
- sen är inte multicast påslaget. slå på med `sudo ip link set 0 multicast on`
- gör det på alla som ska prata med varandra.
- då funkar det! men bara server-peer och vice versa. inte p2p via server.
- sen säger internet att multicast försvinner vid omstart. men kanske går att lösa på nåt sätt: https://forum.openwrt.org/t/wireguard-persistent-multicast-option/134575/4

- MEN: testa det här också. https://answers.ros.org/question/413513/is-it-possible-to-run-ros2-via-wireguard/
- kolla också gihublänken i posten ovan

- om man vill ha peer to peer, skulle man faktiskt kunna ha sin dator som server. eller ålen? fast ålen har ju inte statisk IP... det är det som är hela problemet. kanske inte då

- testa den här exemplet med humble kanske? https://github.com/tuw-robotics/ros2_cyclonedds_wireguard/tree/main/src/topics
- men då måste jag ju installera humble lokalt? om jag inte sätter upp en till lightsail.. kanske enklast att köra på tre olika lightsails först. och sen om det mot förmodan funkar, prova rpi

- också: kanske ska chilla?
- det här kanske är fine.
- om vi har en dedikerad vpn server (den måste dock hållas uppdaterad? men med cdk är det enklare förstås)
- men då kan man alltid connecta genom ssh. få topics därifrån.
- så fort den går igång borde servern vara medveten om topics. vi kan då ha ui för det.
- tänk lite mer här

1022

===
de funkar!

- multicast verkar inte behövas. så gör inget åt det
- ROS_DOMAIN_ID behövs inte heller
- detta behövs:
- `export CYCLONEDDS_URI=file:///$(pwd)/dds.xml`
- `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

```xml
<?xml version="1.0" encoding="UTF-8" ?>
  <CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>wg0</NetworkInterfaceAddress>
        </General>
        <Discovery>
            <Peers>
                <Peer address="10.66.66.3" />
                <Peer address="10.66.66.2" />
                <Peer address="10.66.66.1" />
                <Peer address="10.66.66.4" />
            </Peers>
            <ParticipantIndex>auto</ParticipantIndex>
        </Discovery>
    </Domain>
</CycloneDDS>
```

- kom ihåg att se till att interface och adresser är rätt
- `ros2 run demo_nodes_py listener`
- `ros2 run demo_nodes_py talker`
- då ser dom varandra på varsin maskin, via vpn-servern
- lyckades köra ålen på rpi, gc på datan, öppna appen på localhost med backend på localhost
- MEN: `ros2 topic list` funkar inte.
  - `https://www.google.com/search?q=ros2+topic+list+not+working+dds+cyclone&oq=ros2+topic+list+not+working+dds+cyclone&gs_lcrp=EgZjaHJvbWUyBggAEEUYOTIHCAEQIRigAdIBCTExMTU5ajBqN6gCALACAA&sourceid=chrome&ie=UTF-8`
  - `https://answers.ros.org/question/373070/ros2-topic-empty-with-fastdds-discovery-server/`
  - för att se om det går att fixa.
  - testa med `ROS_DISCOVERY_`
- det funkar att se det på servern
- vafan?!?!? det funkar out of the box??!??!
- `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- med denna ser jag dessutom topics p2p??!
- men nu ser jag inte topics på den andra klienten .4?
- måste jag ha cyclone för det? fast då hittar dom bara varann via noder
- https://medium.com/@arshad.mehmood/setting-up-node-discovery-across-multiple-systems-in-ros2-infrastructure-a1a5c25f052f
- i den länken finns config för fastdds
- kanske prova igen med cyclone, men prova `ros2 daemon stop && ros2 daemon start`
- summa karde: cyclone verkar mest pålitligt? fattar dock inte varfer de funkar rpi till datan?? men inte till molninstans
- prova allt igen men starta om allt först?
- se om jag kan fixa felet med ros-bridge, och sen vore det intressant om man kunde gå till app.foxpoint.se och gå mot localhost. kan ju testa att köra bridge lokalt, bafatt se om app.foxpoint.se funkar med det

====

- chmod go= /etc/wireguard/private.key

- ta bort client-adam.conf på servern?!?!

- ta bort private.key och public.key i /etc/wireguard på datan

  - behovs inte om man jag har min conf från servern???

- på klienten: `wg-quick up wg0` (så heter ju min conf)
- resolvconf måste dock installeras! det har jag inte gjort

`sudo systemctl status wg-quick@wg0`

Client

```bash
[Interface]
Address = 10.66.66.1/24 # same as the client on server

```

```bash
# install wireguard

wg genkey | sudo tee /etc/wireguard/private.key
sudo chmod go= /etc/wireguard/private.key

sudo cat /etc/wireguard/private.key | wg pubkey | sudo tee /etc/wireguard/public.key

```

public IP server `18.157.133.166`
public IP port `55943`

wg0-client-adam.conf

```conf
[Interface]
PrivateKey = REDACTED
Address = 10.66.66.2/32,fd42:42:42::2/128
DNS = 1.1.1.1,1.0.0.1
PostUp = ping -c1 10.66.66.1

[Peer]
PublicKey = oOUkpHQGVvbtP5RDTWC+/HpVic+FSruKJHoB7Og4uTs=
PresharedKey = 7phRTitzG4CuI37tpC8fXW6IL/kvCzaTGpt+YVnHEGI=
Endpoint = 18.157.133.166:55943
AllowedIPs = 0.0.0.0/0,::/0
PersistentKeepalive = 25
```

server /etc/wireguard/wg0.conf

```conf
[Interface]
Address = 10.66.66.1/24,fd42:42:42::1/64
ListenPort = 55943
PrivateKey = REDACTED
PostUp = iptables -I INPUT -p udp --dport 55943 -j ACCEPT
PostUp = iptables -I FORWARD -i ens5 -o wg0 -j ACCEPT
PostUp = iptables -I FORWARD -i wg0 -j ACCEPT
PostUp = iptables -t nat -A POSTROUTING -o ens5 -j MASQUERADE
PostUp = ip6tables -I FORWARD -i wg0 -j ACCEPT
PostUp = ip6tables -t nat -A POSTROUTING -o ens5 -j MASQUERADE
PostDown = iptables -D INPUT -p udp --dport 55943 -j ACCEPT
PostDown = iptables -D FORWARD -i ens5 -o wg0 -j ACCEPT
PostDown = iptables -D FORWARD -i wg0 -j ACCEPT
PostDown = iptables -t nat -D POSTROUTING -o ens5 -j MASQUERADE
PostDown = ip6tables -D FORWARD -i wg0 -j ACCEPT
PostDown = ip6tables -t nat -D POSTROUTING -o ens5 -j MASQUERADE

### Client adam
[Peer]
PublicKey = 1XcZiW7jdUJ6QzIb8iXeAKl5Ys6QAW3xK5fYA23IFnA=
PresharedKey = 7phRTitzG4CuI37tpC8fXW6IL/kvCzaTGpt+YVnHEGI=
AllowedIPs = 10.66.66.2/32,fd42:42:42::2/128
```
