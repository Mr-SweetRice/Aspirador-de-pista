# Aspirador de Pista

Firmware ESP-IDF para ESP32-S3 com arquitetura modular para controle e telemetria de um robo de pista. A base atual limpa o firmware antigo e mantem apenas configuracoes, pinagem e comunicacao BLE inicial.

## Estrutura

- `main/`: inicializacao geral, NVS e controle de servicos/tasks.
- `components/motors`: acionamento PWM dos motores principais, TB6612FNG e motor auxiliar.
- `components/line_sensor`: pinagem do sensor de linha QTR-8A/QTR-8D.
- `components/encoder`: pinagem e parametros dos encoders N30.
- `components/battery_level`: reservado para bateria.
- `components/comms`: comunicacao BLE e protocolo binario.
- `components/odometry`: configuracoes basicas de odometria.
- `components/imu`: configuracao I2C do MPU-9250.
- `components/memory`: configuracao base de NVS.

## Documentação de hardware

Materiais, esquemático funcional, pinagem, ligações do TB6612FNG e checklist de montagem: [docs/HARDWARE.md](docs/HARDWARE.md).

## Requisitos

- ESP-IDF 5.3 instalado.
- Target ESP32-S3.
- PowerShell no Windows.

Neste computador o ESP-IDF esta em:

```powershell
C:\Users\tecni\esp\v5.3\esp-idf
```

## Preparar o terminal

Abra o PowerShell na pasta do projeto e carregue o ambiente ESP-IDF:

```powershell
. C:\Users\tecni\esp\v5.3\esp-idf\export.ps1
```

Se estiver usando outro caminho de instalacao, ajuste o comando acima.

## Configurar o target

Normalmente o projeto ja esta configurado para ESP32-S3. Se precisar refazer:

```powershell
idf.py set-target esp32s3
```

## Compilar

```powershell
idf.py build
```

O binario final sera gerado em:

```text
build/aspirador_de_pista.bin
```

## Gravar no ESP32-S3

Conecte a placa por USB e descubra a porta serial. No Windows ela costuma aparecer como `COM3`, `COM4`, etc.

Grave substituindo `COMx` pela porta correta:

```powershell
idf.py -p COMx flash
```

Para gravar e abrir o monitor serial:

```powershell
idf.py -p COMx flash monitor
```

Para sair do monitor:

```text
Ctrl + ]
```

## BLE atual

O firmware atual inicia um servidor BLE NimBLE.

- Nome anunciado: `AspiradorPista-S3`
- Servico GATT: `5d7a0000-8f5a-4a7d-9d4f-7a6c2b8d0001`
- Auth characteristic: `5d7a0001-8f5a-4a7d-9d4f-7a6c2b8d0001`
- Command characteristic: `5d7a0002-8f5a-4a7d-9d4f-7a6c2b8d0001`
- Telemetry characteristic: `5d7a0003-8f5a-4a7d-9d4f-7a6c2b8d0001`
- Token inicial: `engineering-token`

Antes de enviar comandos, escreva o token na characteristic de autenticacao.

## Protocolo binario inicial

Pacote base:

```text
byte 0: versao do protocolo
byte 1: classe do comando
byte 2: id da mensagem
byte 3: tamanho do payload
byte 4..N: payload
```

Versao atual:

```text
1
```

Classes:

```text
0x01 SAVE
0x02 READ
0x03 SEND
0x04 TELE
```

Exemplo de pacote `SEND STOP` sem payload:

```text
01 03 01 00
```

Neste momento os comandos sao recebidos e validados, mas ainda nao executam controle real dos motores. A ligacao com `control`, `memory` e telemetria sera feita nas proximas etapas.

## Configuracao de motores e encoders

Os pinos dos motores ficam em `components/motors/include/motors_config.h`.

- Esquerdo: `PWMA GPIO1`, `AIN1 GPIO2`, `AIN2 GPIO42`, encoder `GPIO21/GPIO47`.
- Direito: `PWMB GPIO39`, `BIN1 GPIO41`, `BIN2 GPIO40`, encoder `GPIO48/GPIO35`.
- Auxiliar: PWM via SI2300 em `GPIO4`, sem direcao e sem encoder.
- Driver principal: `TB6612FNG`, `STBY GPIO36`.
- PWM: LEDC low speed, timer 0, 10 bits, `25 kHz`, canais 0/1/2.

Os encoders ficam em `components/encoder/include/encoder_config.h`.

- Contagens por volta do motor: `44`.
- Reducao: `10:1`.
- Multiplicador quadratura: `4x`.
- Contagens por volta de saida: `44 * 10 * 4 = 1760`.

A odometria usa esses parametros por `components/odometry/include/odometry_config.h`.

## Observacoes

- O `sdkconfig` esta com Bluetooth/NimBLE habilitado.
- O arquivo `sdkconfig.defaults` tambem guarda as opcoes principais de BLE para futuras reconfiguracoes.
- Os componentes sem logica final foram deixados minimos de proposito.
