# EV49N51A – imagem do produto (bridge Ethernet ⇄ Wi-Fi): modo Ethernet 10 Mbps

Base do branch: `upstream/master` de 2026-10-10 (inclui o Wi-Fi e o WPA3 do
upstream). Este manual é para quem vai **testar** a imagem do produto com o
IED. Ele explica o workaround de Ethernet, como verificar e controlar o modo
de conexão e quais testes executar. Vale só para o branch
`feat/ev49n51a-bridge-product` do fork; não faz parte do upstream do NuttX.

## 1. Resumo

- Na placa testada (construída a partir do projeto EV49N51A), os quadros
  **transmitidos pela placa se perdem no parceiro de link quando a Ethernet
  opera a 100 Mbps**; a recepção funciona. A 10 Mbps a mesma placa transmite
  e recebe sem perdas.
- A imagem do produto, portanto, **anuncia apenas 10BASE-T** na
  autonegociação (`CONFIG_PIC32MZ_PHY_10MBPS_ONLY=y`). O link sempre sobe a
  10 Mbps (full ou half duplex, conforme negociado) e o MAC acompanha a
  velocidade negociada.
- É um **workaround**, não a correção. A causa raiz (provavelmente analógica,
  no PHY/magnéticos) não foi estabelecida. Ver [Limitações](#8-limitações).

## 2. O que a imagem faz no boot

`boards/mips/pic32mz/ev49n51a/src/etc/init.d/rcS`:

1. `eth0` recebe o MAC localmente administrado `02:e0:de:ad:be:ef` (o W1 não
   tem MAC de fábrica) e sobe sem endereço.
2. `wlan0` vira SoftAP WPA2-PSK: SSID `NuttX-BR`, senha `nuttxbridge`,
   canal 6. Com `EV49N51A_BRIDGE_WPA3=y` (opcional, ver 3.1) o SoftAP é
   WPA3-Personal.
3. `br0` é criada com `eth0` e `wlan0` como portas e recebe `10.0.0.2/24`.
   Só `br0` tem endereço.
4. **A placa não roda servidor DHCP** (`EV49N51A_BRIDGE_DHCPD=n`). O
   servidor DHCP é o **IED**; a placa é uma bridge transparente entre
   `eth0` e o SoftAP, e os clientes Wi-Fi recebem endereço do IED através
   dela. O endereço `10.0.0.2` de `br0` é só de gerenciamento da placa e
   deve ficar **fora da faixa de leases do IED**.

Os valores vêm das opções `EV49N51A_BRIDGE_SSID`, `_PSK`, `_CHANNEL`,
`_IPADDR`, `_DHCPD` e `_WPA3` (Kconfig da placa). O servidor da placa
continua compilado (`dhcpd_start br0` / `dhcpd_stop` no NSH) para uso de
bancada quando **não** há outro servidor na rede; nunca deixe dois
servidores ativos ao mesmo tempo. Atenção: na bancada, `dhcpd_start` mudou o
endereço de `br0` para `10.0.0.1`; confira com `ifconfig br0` depois de
iniciá-lo. `dhcpd_stop` (e não `kill`) é o jeito de pará-lo.

## 3. Compilar e gravar

Precisa do XC32 v6.00 e do `genromfs` no `PATH`:

```sh
export PATH=/opt/microchip/xc32/v6.00/bin:<pasta do genromfs>:$PATH
./tools/configure.sh -l ev49n51a:bridge
make -j$(nproc)
```

Instalação das ferramentas: ver o [Anexo 9](#9-anexo-instalação-das-ferramentas).

O resultado é `nuttx.hex`. Se você mudar uma opção que afeta o `rcS`
(`EV49N51A_BRIDGE_DHCPD`, `_WPA3`, SSID, senha, canal, endereço), rode
`touch boards/mips/pic32mz/ev49n51a/src/etc/init.d/rcS` antes do `make`: o
`/etc` embutido só é regenerado quando o próprio `rcS` muda, não quando o
`.config` muda. O primeiro build precisa de rede (baixa a biblioteca Wi-Fi,
ver 3.1).

Gravação com o PICkit 3 (leva ~2 min):

```sh
cd /opt/microchip/mplabx/v6.20/mplab_platform/mplab_ipe
./ipecmd.sh -P32MZ1025W104132 -TPPK3 -M -F<caminho>/nuttx.hex -Y -OL
```

Reset da placa sem regravar (~11 s):

```sh
./ipecmd.sh -P32MZ1025W104132 -TPPK3 -OK -OL
```

Console: UART de debug (X5: 1 = TX-in RA8, 2 = RX-out RA9, 3 = GND),
115200 8N1.

### 3.1 Wi-Fi: biblioteca da Microchip e segurança (WPA2 / WPA3)

**Biblioteca Wi-Fi (blob).** O `pic32mzw1.a` da Microchip não está no
repositório: o build o baixa sozinho (`context::`) de
`github.com/Microchip-MPLAB-Harmony/wireless_wifi`, tag **v3.13.0**, para
`arch/mips/src/chip/`. O primeiro build precisa de rede; `make distclean`
apaga o download e ele é baixado de novo. Só compila com XC32. Ele fica
sob a licença da Microchip, não Apache: confira se ela cobre a distribuição
da firmware.

**WPA2 (padrão do produto).** SoftAP WPA2-PSK/CCMP. Não usa o acelerador
de criptografia BA414E, portanto **não há microcódigo nem download extra**.

**WPA3-Personal (opcional, a critério do integrador).** O SoftAP passa a
exigir SAE; clientes só WPA2 não conseguem associar. Para habilitar:

```sh
./tools/configure.sh -l ev49n51a:bridge
kconfig-tweak --enable PIC32MZ_W1_BA414E --enable EV49N51A_BRIDGE_WPA3
make olddefconfig
touch boards/mips/pic32mz/ev49n51a/src/etc/init.d/rcS
make -j$(nproc)
```

- O `touch` é necessário: o `rcS` só é reprocessado quando o próprio arquivo
  muda, não quando o `.config` muda.
- O acelerador BA414E precisa do microcódigo. Por padrão
  (`PIC32MZ_W1_BA414E_UCODE_BUILTIN`) o build baixa o `drv_ba414e.c` do
  Harmony `crypto` v3.9.0 e extrai as 810 palavras (confere o sha256;
  se o servidor devolver um erro, o build recusa e apaga o arquivo: basta
  repetir o `make`). Alternativa: `PIC32MZ_W1_BA414E_UCODE_FILE`, que
  carrega o microcódigo de um arquivo (`/etc/ba414e.bin`) no primeiro uso.
- Esse arquivo do Harmony está sob a licença MPLAB Harmony da Microchip, que
  restringe o uso a produtos Microchip e a redistribuição: confira com o
  jurídico antes de distribuir uma imagem com WPA3.
- Validado em bancada: imagem com WPA3 anunciando `NuttX-BR` como WPA3 e um
  laptop associado por SAE (10/10 pings, 0% de perda).

## 4. Controle do modo de conexão Ethernet

### 4.1 Imagem do produto: 10 Mbps fixo (decidido na compilação)

| Opção                                   | Anunciado na autonegociação       |
|-----------------------------------------|-----------------------------------|
| `PIC32MZ_PHY_10MBPS_ONLY=y` (produto)   | 10BASE-T FD e HD                  |
| `PIC32MZ_PHY_10MBPS_ONLY=n`             | 100BASE-TX FD/HD e 10BASE-T FD/HD |

Para gerar a mesma imagem com 100 Mbps habilitado (testes comparativos):

```sh
./tools/configure.sh -l ev49n51a:bridge
kconfig-tweak --disable PIC32MZ_PHY_10MBPS_ONLY
make olddefconfig
make -j$(nproc)
```

A velocidade e o duplex são decididos **uma vez**, na inicialização da
interface, a partir do que a placa anuncia e do que o parceiro anuncia.
Parceiro sem autonegociação (velocidade fixa) não é suportado: o driver
retorna `-ENODEV`.

### 4.2 Comandos disponíveis na imagem do produto

A imagem do produto **não tem** `mdio`, `mw`, `mb` nem `mh`. Tem:

| Comando                | Uso                                              |
|------------------------|--------------------------------------------------|
| `ifconfig`             | Estado e contadores das interfaces               |
| `ifdown eth0`          | Derruba a porta Ethernet                         |
| `ifup eth0`            | Sobe de novo (renegocia o link)                  |
| `ping <ip>`            | Alcançabilidade e perda                          |
| `iperf`                | Vazão TCP/UDP, cliente/servidor                  |
| `brctl addif/delif`    | Adiciona/remove porta da bridge                  |
| `dhcpd_stop`           | Para o servidor DHCP                             |
| `dhcpd_start br0`      | Inicia o servidor DHCP                           |

`ifconfig eth0` mostra os contadores usados nos testes:

```
RX: Received Fragment Errors   Bytes
    000004e1 00000000 00000000 1e66d
    IPv4     ARP      Dropped
    000004a0 0000000a 00000000
TX: Queued   Sent     Errors   Timeouts Bytes
    00000405 00000405 00000000 00000000 18894
```

- `Errors` do RX deve ficar em **0**. RX com erros crescendo é o sintoma de
  PHY e MAC em velocidades diferentes.
- `Queued` e `Sent` do TX devem ser **iguais** com o link ocioso. Uma pequena
  diferença sob carga UDP pesada *originada pela placa* é a fila de TX
  descartando quadros, não erro de link.

`ifdown eth0` / `ifup eth0` numa porta da bridge foi testado repetidamente e
não trava a bridge.

### 4.3 Imagem de diagnóstico: inspecionar e forçar PHY e MAC

Para bancada, compile a configuração do produto com os comandos de memória e
MDIO:

```sh
./tools/configure.sh -l ev49n51a:bridge
kconfig-tweak --enable SYSTEM_MDIO \
              --disable NSH_DISABLE_MW --disable NSH_DISABLE_MB \
              --disable NSH_DISABLE_MH
make olddefconfig
make -j$(nproc)
```

**Essa imagem é só de bancada. Não deve ser entregue.**

`mdio <phyaddr> <reg> [valor]` lê/escreve um registrador do PHY. **Todos os
números são hexadecimais.** O LAN8720A está no endereço 0.

| Reg | Significado               | Valores úteis                                      |
|-----|---------------------------|----------------------------------------------------|
| 0   | Controle básico           | `1200` = reinicia a autonegociação                 |
| 1   | Status básico             | bit 2 = link up, bit 5 = AN concluída; o bit de link é latched, leia duas vezes |
| 4   | Anúncio                   | `61` = só 10BASE-T; `1e1` = 10/100 FD/HD           |
| 5   | Habilidades do parceiro   | `cde1` = parceiro oferece 10/100                   |
| 1f  | Controle/status especial  | bits 4:2 = indicação de velocidade                 |

Indicação de velocidade (reg `1f`, bits 4:2): `001` 10BASE-T HD, `101`
10BASE-T FD, `010` 100BASE-TX HD, `110` 100BASE-TX FD. Na bancada foi lido
`0x1054` com o link em 10BASE-T full duplex (bits 4:2 = `101`). Decodifique
os bits; não compare a palavra inteira.

`mh <endereço> <qtd>` e `mw <endereço>=<valor>` leem/escrevem SFRs de 32
bits. **Use `0x` no valor do `mw`**: sem ele o número é lido como decimal.

O bit de velocidade do RMII é `EMAC1SUPP` (`0xbf843260`) bit 8, `SPEEDRMII`
(1 = 100 Mbps). Alias CLR: `0xbf843264`; alias SET: `0xbf843268`.

Para forçar 10 Mbps em tempo de execução numa imagem que anuncia 100 Mbps:

```
nsh> mdio 0 4 61
nsh> mdio 0 0 1200
nsh> mw bf843264=0x100
```

O `mw` é **obrigatório**: mudar só o PHY deixa o MAC em RMII 100 Mbps; o
link sobe, mas o receptor da placa vê quadros corrompidos (`Errors` do RX
crescem). A imagem do produto não precisa disso porque o driver programa o
MAC a partir do modo negociado.

Voltar a 100 Mbps em runtime (`mdio 0 4 1e1`, `mdio 0 0 1200`,
`mw bf843268=0x100`) **não foi testado**; resete a placa.

## 5. Testes de aceitação com o IED

São os testes executados na bancada com um PC como parceiro Ethernet.
Repita-os com o **IED** ligado em `eth0` (direto ou pelo switch da
instalação) e anote os resultados na tabela da seção 6.

### Preparação

- Placa com a imagem do produto, alimentada como na instalação; console na
  UART de debug.
- IED em `eth0`, **servidor DHCP** da rede `10.0.0.0/24` (endereço do IED
  diferente de `10.0.0.2`, leases fora de `10.0.0.2`).
- Um cliente Wi-Fi conectado ao SoftAP `NuttX-BR`, para o teste através da
  bridge. Ele deve receber o endereço **do IED**.
- Um PC com `iperf` (2.x) e `ping` do lado Ethernet, ou as ferramentas de
  teste do próprio IED. Onde o IED não roda `iperf`, troque esses testes pelo
  tráfego da aplicação do IED e aplique os mesmos critérios (sem perda, sem
  erros).

### T1 – Modo do link após o boot

1. Ciclo de energia da placa com o cabo conectado.
2. Aguarde o prompt do NSH e o SoftAP.
3. `ifconfig eth0`: `RUNNING`, `Errors` de RX e TX em zero.
4. Opcional (imagem de diagnóstico): `mdio 0 1f`, bits 4:2 = `101`
   (10BASE-T FD) ou `001` (HD, se o IED/switch só oferecer isso).

**Aprovado:** link a 10 Mbps, sem erros de RX.

### T2 – Alcançabilidade e perda

Do IED (ou PC) para a placa e para o cliente Wi-Fi:

```sh
ping -c 1000 -i 0.2 10.0.0.2
```

**Aprovado:** 0% de perda; RTT típico de 0,8–1,1 ms. Um pacote perdido logo
no início após o reset (primeiro ARP) não é falha; perdas depois disso são.

Depois, `ifconfig eth0` na placa: TX `Queued` = `Sent`; `Errors` de TX e RX = 0.

### T3 – Vazão

TCP e UDP, nos dois sentidos. O `iperf` da placa deve ser executado **pelo
console serial** e precisa de `-B` com o endereço de `br0`:

```sh
# IED/PC -> placa
placa:  iperf -s -B 10.0.0.2 -t 12
par:    iperf -c 10.0.0.2 -t 10
placa:  iperf -s -u -B 10.0.0.2 -t 12
par:    iperf -c 10.0.0.2 -u -b 8M -t 10

# placa -> IED/PC
par:    iperf -s            (e iperf -s -u)
placa:  iperf -c <ip do par> -B 10.0.0.2 -t 10
placa:  iperf -c <ip do par> -B 10.0.0.2 -u -t 10
```

Resultados de referência na bancada (10 Mbps, PC como par):

| Teste                     | Resultado                  | Critério            |
|---------------------------|----------------------------|---------------------|
| TCP par → placa           | 9,49 Mbit/s                | ≥ 9 Mbit/s          |
| TCP placa → par           | 8,26 Mbit/s                | ≥ 8 Mbit/s          |
| UDP par → placa (8M)      | 8,39 enviado / 8,38 recebido | sem perda        |
| UDP placa → par           | 9,56 Mbit/s                | 0 datagramas perdidos |

### T4 – Caminho pela bridge (cliente Wi-Fi ⇄ IED)

Com um cliente Wi-Fi no SoftAP, repita T2 e T3 entre o cliente e o IED. A
vazão é limitada pelo lado Ethernet (10 Mbps) e pelo AP Wi-Fi (~22 Mbit/s
sozinho).

**Aprovado:** sem perda no ping; vazão no nível de T3; `br0` ainda
alcançável ao final.

### T5 – Ciclos de cabo e de porta

- Desconectar e reconectar o cabo Ethernet **10 vezes**. Após cada uma, o
  tráfego deve voltar sem intervenção (a autonegociação fica habilitada e o
  PHY renegocia).
- `ifdown eth0` / `ifup eth0` **5 vezes** com um ping rodando. **Aprovado:**
  a placa continua viva, o tráfego volta após cada `ifup`, sem travar.
- Ciclo de energia **5 vezes**; cada um deve chegar ao T1.

### T6 – Qualidade do cabo

Repita T2 com o cabo real da instalação (comprimento, qualidade) e, se
houver, um pior. Registre a perda e o contador `Errors` do RX.

### T7 – Soak

Deixe um ping com `-i 1` e um iperf TCP nos dois sentidos por **pelo menos
12 horas**. **Aprovado:** sem perdas além da pontual do início, sem
crescimento do heap da placa (`free` no início e no fim), sem resets.

### T8 – Tráfego não-IP do IED (verificação)

O driver Ethernet do PIC32MZ é do tipo *legacy*, então a bridge só enxerga
quadros IPv4, IPv6 e ARP vindos dele. Outros EtherTypes do IED (por exemplo
GOOSE IEC 61850, LLDP) **não são encaminhados** ao lado Wi-Fi. Se o produto
precisar encaminhá-los, isso deve ser informado: exige uma pequena alteração
no driver (hook da bridge antes do despacho por EtherType) ou a migração para
lowerhalf. Esta análise vem do projeto e não foi testada na bancada.

### T9 – DHCP do IED através da bridge

**Objetivo:** provar que um cliente Wi-Fi associado ao SoftAP da placa
recebe endereço do servidor DHCP do **IED**, com a placa apenas repassando
os quadros (bridge transparente).

**Preparação**

- Placa com a imagem do produto, `eth0` ligada ao IED (direto ou pelo
  switch da instalação) e console serial aberto (115200 8N1).
- O IED é o **único** servidor DHCP da rede. O endereço do IED não pode ser
  `10.0.0.2` (é o da `br0`), e `10.0.0.2` deve ficar fora da faixa de leases
  do IED.
- Um cliente Wi-Fi (laptop ou celular) sem IP fixo, configurado para DHCP.

**Passos**

1. **Confirme que a placa não serve DHCP.** No console da placa:

   ```
   nsh> ps
   ```

   Não pode haver nenhuma linha `dhcpd`. Se houver, `dhcpd_stop`. Com a
   placa e o IED servindo DHCP ao mesmo tempo o teste não vale.
2. **Confirme a bridge:** `ifconfig br0` deve mostrar `10.0.0.2` e
   `RUNNING`; `eth0` e `wlan0` também `RUNNING`.
3. **Associe o cliente ao SoftAP `NuttX-BR`** (senha `nuttxbridge`, WPA2)
   com IP automático. No Linux com NetworkManager:
   `nmcli dev wifi connect NuttX-BR password nuttxbridge`.
4. **Verifique o endereço recebido.** No cliente, `ip -4 addr show <interface>`:
   o IP deve estar na faixa do IED, e o *server identifier* do lease deve ser
   o IP do IED (por exemplo `nmcli -f DHCP4 dev show <interface>`, ou o log do
   `dhclient`). Se o servidor for `10.0.0.2`, é a placa servindo: o passo 1
   falhou.
5. **Ping nos dois sentidos.** Do cliente: `ping -c 100 <ip do IED>` e
   `ping 10.0.0.2` (a `br0`). Do IED, ping para o IP do cliente.
6. **Renove o lease.** No cliente:
   `sudo dhclient -r <interface> && sudo dhclient <interface>` (ou desligue e
   ligue o Wi-Fi). O endereço deve voltar a ser do IED.
7. **Repita a associação 3 vezes** (Linux: `nmcli con down NuttX-BR` e
   `nmcli con up NuttX-BR`). Em cada uma o cliente deve obter endereço do IED.
8. **Contadores no fim:** `ifconfig eth0` e `ifconfig wlan0` na placa com
   `Errors` em 0.

**Aprovado:**

- o cliente obtém endereço do IED nas 3 associações;
- ping sem perda (uma perda no primeiro ARP é aceitável);
- `br0` (`10.0.0.2`) continua alcançável;
- a placa não oferece nenhum lease.

**Se falhar**, anote qual servidor respondeu (IP do *server identifier*), a
saída de `ps` e `ifconfig` e o log serial.

Referência na bancada (2026-10-10, imagem do produto com 10 Mbps e sem
`dhcpd`): um R550 como IED, com servidor DHCP em `10.0.0.1` ligado à `eth0`
pelo switch, e um laptop no SoftAP. Nas 4 associações (a inicial e 3
reassociações) mais uma renovação, o laptop recebeu `10.0.0.100` com
*server identifier* `10.0.0.1` (MAC do R550); `ping` laptop → `br0` com 100
pacotes sem perda (0,8 ms a 9,4 ms); `Errors` de `eth0` e `wlan0` em 0;
`ps` na placa sem `dhcpd`.

Cuidado ao reproduzir com um PC na mesma rede: se o PC tiver o mesmo IP do
IED (`10.0.0.1`), o `ping` do cliente Wi-Fi para esse IP é respondido
localmente pelo próprio PC e não prova nada sobre o IED; confira o MAC do
vizinho (`ip neigh`) ou tire o IP duplicado.

## 6. Tabela de resultados

| Teste | Descrição                                   | Aprovado | Observações |
|-------|---------------------------------------------|----------|-------------|
| T1    | Modo do link após o boot                    | ☐        |             |
| T2    | 1000 pings, 0% de perda                     | ☐        |             |
| T3    | iperf TCP/UDP nos dois sentidos             | ☐        |             |
| T4    | Cliente Wi-Fi ⇄ IED pela bridge             | ☐        |             |
| T5    | Cabo, ifdown/ifup, ciclos de energia        | ☐        |             |
| T6    | Cabo da instalação                          | ☐        |             |
| T7    | Soak de 12 h                                | ☐        |             |
| T8    | Requisito de tráfego não-IP                 | ☐        |             |
| T9    | DHCP do IED através da bridge               | ☐        |             |

## 7. Como relatar um problema

Anexe: saída de `ifconfig` (antes e depois do teste), `uname -a`, qual teste
falhou, tipo e comprimento do cabo, parceiro de link (IED direto ou switch,
modelo) e, se possível, o log do console serial.

## 8. Limitações

- Máximo de 10 Mbps em `eth0`. O lado Wi-Fi sozinho faz ~22 Mbit/s, então
  uma transferência Wi-Fi ⇄ Ethernet é limitada pela porta Ethernet.
- O workaround esconde o problema de TX a 100 Mbps; não o explica. Na placa
  testada, o buck de 3,3 V, o layout dos pares e o roteamento do clock RMII
  foram descartados por experimentos; o principal suspeito é a magnética do
  RJ45 (ainda não confirmado). A correção definitiva é uma revisão de
  hardware.
- Half duplex e parceiro sem autonegociação não foram testados.
- A velocidade é decidida uma vez na inicialização da interface. Mover o
  cabo entre parceiros que negociam velocidades diferentes sem reiniciar a
  porta não é suportado; com o limite de 10 Mbps isso não se aplica, pois
  toda negociação termina em 10 Mbps.

## 9. Anexo: instalação das ferramentas

Versões usadas na bancada: **MPLAB X v6.20**, **XC32 v6.00** (GCC 13.2.1) e o
**DFP `PIC32MZ-W_DFP` 1.12.356**. Para apenas **gravar** a placa basta o
MPLAB IPE (`ipecmd`) do MPLAB X; o XC32 e o DFP são necessários só para
**compilar**.

### 9.1 Baixar

- MPLAB X IDE v6.20 para Linux: `MPLABX-v6.20-linux-installer.sh` (~936 MB).
- MPLAB XC32 v6.00 para Linux x64:
  `xc32-v6.00-full-install-linux-x64-installer.run` (~1,4 GB).
- DFP: `Microchip.PIC32MZ-W_DFP.1.12.356.atpack` em
  `packs.download.microchip.com` (é um arquivo zip).

Os dois instaladores ficam no site da Microchip, nas páginas de downloads do
MPLAB X e do XC32 (versões anteriores). Procure exatamente por
"MPLAB X IDE v6.20" e "MPLAB XC32 v6.00".

### 9.2 MPLAB X

```sh
chmod +x MPLABX-v6.20-linux-installer.sh
sudo ./MPLABX-v6.20-linux-installer.sh
```

Instala em `/opt/microchip/mplabx/v6.20`; o `ipecmd` fica em
`mplab_platform/mplab_ipe/ipecmd.sh`. Marque o MPLAB IPE no instalador (a IDE
completa é opcional). O MPLAB X traz o próprio Java; não é preciso instalar
um JDK.

### 9.3 XC32

```sh
chmod +x xc32-v6.00-full-install-linux-x64-installer.run
sudo ./xc32-v6.00-full-install-linux-x64-installer.run
```

Instala em `/opt/microchip/xc32/v6.00`. O modo de licença gratuito basta.
Confira com `/opt/microchip/xc32/v6.00/bin/xc32-gcc --version`.

### 9.4 DFP

```sh
mkdir -p ~/microchip/WFI32-W_DFP
unzip Microchip.PIC32MZ-W_DFP.1.12.356.atpack -d ~/microchip/WFI32-W_DFP
```

O `Make.defs` da placa procura o DFP em `~/microchip/WFI32-W_DFP`. Em outro
local, passe `WFI32E01_DFP_DIR=<pasta>` ao `make`.

### 9.5 Permissão do PICkit 3 (Linux)

Crie `/etc/udev/rules.d/99-pickit3.rules`:

```
# Microchip PICkit 3
ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="900a", MODE="0666", GROUP="plugdev", TAG+="uaccess"
```

```sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

O usuário deve estar no grupo `plugdev`. Replugue o PICkit 3 depois.

### 9.6 Outras ferramentas do build

`make`, `gcc` do host, `python3`, `kconfig-frontends` (`kconfig-tweak`,
`make menuconfig`) e `genromfs` (gera o `/etc` embutido). Na bancada o
`genromfs` está em `~/nuttx-tools/bin` e vai no `PATH` junto com o XC32.

### 9.7 Teste da instalação

```sh
cd /opt/microchip/mplabx/v6.20/mplab_platform/mplab_ipe
./ipecmd.sh -P32MZ1025W104132 -TPPK3 -M -F<caminho>/nuttx.hex -Y -OL
```

Deve terminar com `Operation Succeeded`. A gravação leva ~2 min. Se o
PICkit 3 não for encontrado, confira a regra udev (9.5) e o cabo ICSP.
