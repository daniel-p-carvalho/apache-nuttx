# EV49N51A – imagem do produto (bridge Ethernet ⇄ Wi-Fi): modo Ethernet 10 Mbps

Este manual é para quem vai **testar** a imagem do produto com o IED. Ele
explica o workaround de Ethernet, como verificar e controlar o modo de
conexão e quais testes executar. Vale só para o branch
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
   canal 6.
3. `br0` é criada com `eth0` e `wlan0` como portas e recebe `10.0.0.2/24`.
   Só `br0` tem endereço.
4. O servidor DHCP roda em `br0`, entregando endereços a partir de
   `10.0.0.100`.

Os valores vêm das opções `EV49N51A_BRIDGE_SSID`, `_PSK`, `_CHANNEL`,
`_IPADDR` e `_DHCPD` (Kconfig da placa).

## 3. Compilar e gravar

Precisa do XC32 v6.00 e do `genromfs` no `PATH`:

```sh
export PATH=/opt/microchip/xc32/v6.00/bin:<pasta do genromfs>:$PATH
./tools/configure.sh -l ev49n51a:bridge
make -j$(nproc)
```

O resultado é `nuttx.hex`. Gravação com o PICkit 3 (leva ~2 min):

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
- IED em `eth0` com IP estático em `10.0.0.0/24` diferente de `10.0.0.2`
  (ou usando o DHCP da placa, se o IED usar DHCP).
- Um cliente Wi-Fi conectado ao SoftAP `NuttX-BR`, para o teste através da
  bridge. Ele recebe endereço do DHCP da placa.
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
