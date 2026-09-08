# Aspirador de Pista

Firmware ESP-IDF para o ESP32-S3-DevKitC-1, responsável pelo controle de um robô de pista, motores GA12-N30, turbina, sensores, odometria e telemetria BLE.

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
- `components/portal_sensor` e `components/vl53l0x_driver`: sensor de portal/distância VL53L0X.
- `engineering_ui/`: interface de controle, mapas, gráficos e configuração da telemetria.
- `mobile_app/`: aplicativo móvel BLE.

## Documentação de hardware

Materiais, esquemático funcional, pinagem, ligações do TB6612FNG e checklist de montagem: [docs/HARDWARE.md](docs/HARDWARE.md).

Resumo da alimentação: bateria LiPo 3S de 300 mAh, Mini-360 ajustado para 5 V e divisor de bateria de 10 kΩ/1 kΩ. O circuito da turbina usa pull-down de 10 kΩ no controle e diodo 1N4148 em paralelo com o motor. Consulte [docs/HARDWARE.md](docs/HARDWARE.md).

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

## Comunicação BLE

O firmware atual inicia um servidor BLE NimBLE.

- Nome anunciado: `AspiradorPista-S3`
- Servico GATT: `5d7a0000-8f5a-4a7d-9d4f-7a6c2b8d0001`
- Auth characteristic: `5d7a0001-8f5a-4a7d-9d4f-7a6c2b8d0001`
- Command characteristic: `5d7a0002-8f5a-4a7d-9d4f-7a6c2b8d0001`
- Telemetry characteristic: `5d7a0003-8f5a-4a7d-9d4f-7a6c2b8d0001`
- Token inicial: `engineering-token`

Antes de enviar comandos, escreva o token na characteristic de autenticação. A fila BLE prioriza STOP, serializa as escritas ATT e reduz leituras concorrentes para evitar atrasos e perdas. A aba **Telemetria BLE** permite habilitar ou desabilitar cada pacote periódico.

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

Os comandos são processados por uma tarefa dedicada, com STOP priorizado. A odometria interna permanece em 1000 Hz; somente a frequência de transmissão BLE é configurável.

## Configuracao de motores e encoders

Os pinos dos motores ficam em `components/motors/include/motors_config.h`.

- Esquerdo: `PWMA GPIO39`, `AIN1 GPIO40`, `AIN2 GPIO41`, encoder `GPIO35/GPIO14`.
- Direito: `PWMB GPIO1`, `BIN1 GPIO2`, `BIN2 GPIO42`, encoder `GPIO21/GPIO5`.
- Turbina: PWM via SI2300 em `GPIO4`, sem direção e sem encoder.
- Driver principal: `TB6612FNG`, `STBY GPIO36`.
- PWM: LEDC low speed, timer 0, 10 bits, `25 kHz`, canais 0/1/2.

Os encoders ficam em `components/encoder/include/encoder_config.h`.

- Contagens por volta do motor: `11`.
- Redução: `6:1`.
- Multiplicador quadratura: `4x`.
- Contagens por volta de saída: `11 * 6 * 4 = 264`.

A odometria usa esses parametros por `components/odometry/include/odometry_config.h`.

## Observacoes

- O `sdkconfig` esta com Bluetooth/NimBLE habilitado.
- O arquivo `sdkconfig.defaults` tambem guarda as opcoes principais de BLE para futuras reconfiguracoes.
- Firmware compilado com ESP-IDF e testes da interface Python executados com sucesso.
- Confirme alimentação, polaridade e pinagem antes de energizar os motores.
