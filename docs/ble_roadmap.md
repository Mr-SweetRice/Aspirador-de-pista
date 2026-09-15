# Roadmap BLE estavel

## Necessidades da comunicacao

- Comandos criticos, principalmente `STOP`, precisam ter prioridade sobre qualquer telemetria ou transferencia de mapa.
- Comandos de motor podem chegar em rajadas pela UI; o firmware nao deve formar uma fila longa de PWM antigo.
- Telemetria deve ser continua, mas limitada pelo BLE. A UI nao precisa receber todos os sensores em 1 kHz.
- Transferencias grandes, como mapa, devem ter uma janela dedicada para evitar disputa com telemetria regular.
- A conexao deve sobreviver a MTU menor, reconexao da UI e leitura de mapa completa.

## Arquitetura alvo

- Uma unica task deve chamar `ble_gatts_notify_custom`.
- Todas as notificacoes passam por uma fila TX com prioridade:
  - alta: status, resposta de mapa, respostas de comando;
  - normal: telemetria periodica.
- A task TX espera o evento `BLE_GAP_EVENT_NOTIFY_TX` antes de enviar o proximo pacote. Isso aplica backpressure real do stack BLE.
- O payload deve respeitar `ATT_MTU - 3`. Com MTU alto, a telemetria deve ser agrupada em bundle. Com MTU baixo, deve cair para pacotes pequenos.
- Leituras de mapa ativam modo bulk: pausa telemetria periodica e limpa pacotes normais pendentes antes da resposta.
- O nucleo de controle fica reservado para sensores/controle; BLE, TX e tarefas auxiliares ficam no outro nucleo.
- Logs devem mostrar: MTU ativa, parametros de conexao, notificacoes OK/falha, motivo de desconexao e progresso de mapa.

## Estado implementado

- Fila TX BLE com task dedicada `ble_tx`.
- Backpressure via `BLE_GAP_EVENT_NOTIFY_TX`.
- Telemetria agregada em `COMMS_TELE_BUNDLE`.
- Telemetria rápida limitada nominalmente a 60 Hz; estados completos são enviados em frequências menores e alternados por bundle.
- Pacotes periódicos podem ser habilitados ou desabilitados no **Painel de Controle do Robô**.
- Fallback para pacotes pequenos quando MTU nao comporta bundle.
- Janelas bulk para leitura de lista/chunks de mapa.
- Status enviado como resposta do comando `STOP`.
- Telemetria BLE executada no núcleo 0; comandos BLE e controle executam no núcleo 1 com prioridades distintas.
- Tick do FreeRTOS em 1000 Hz para suportar delays de 1-2 ms.
- Sensor de linha configurado com alvo de 1500 Hz e timeout curto de leitura para evitar bloqueios longos no núcleo de controle.
- Stack da task de bateria aumentada para evitar reset por overflow durante logs/leitura ADC.

## Criterios para considerar estavel

- Scan encontra `AspiradorPista-S3`.
- Conexao autentica, habilita notify e negocia MTU sem erro.
- Durante 5 s de telemetria, nao ocorre desconexao.
- `STOP` retorna `STATUS OK` em ate 2 s.
- Leitura da lista de mapas retorna em ate 4 s.
- Leitura de todos os chunks do primeiro mapa termina sem desconexao.
- Logs do ESP32 nao mostram fila TX cheia de forma recorrente.

## Proximas fases se ainda houver queda

1. Aumentar a janela bulk e bloquear telemetria ate o fim explicito da leitura de mapa.
2. Adicionar ACK de mapa com janela de um chunk em voo por vez e timeout/retry no protocolo.
3. Expor na interface contadores de MTU, RSSI, fila TX, notificações OK/falha e motivo da última desconexão.
4. Medir latência física de STOP e comandos de controle em diferentes celulares/computadores.
5. Ajustar os perfis de telemetria com base nessas medições de bancada.
