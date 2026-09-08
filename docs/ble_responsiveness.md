# Responsividade BLE e preservação da odometria

## Arquitetura

- Desktop: comandos seguem por uma fila limitada a 128 entradas, com uma única escrita ATT com resposta em andamento. Não há mais espera fixa de 80 ms. PWM pendente é substituído pelo valor mais recente dentro do mesmo grupo de comandos manuais, sem ultrapassar configurações ou START.
- Desktop e mobile: STOP invalida comandos pendentes e sequências anteriores. Uma escrita já em andamento termina antes do STOP; falhas por falta de recursos têm retentativas curtas e limitadas. Timeout de escrita de 2 s desconecta sem repetir operações cujo resultado ficou desconhecido. Isso depende da proteção BLE de desconexão configurada no robô para cortar motores.
- Firmware: callback GATT valida/copia/enfileira. Uma tarefa de comandos no núcleo 1, prioridade 5, executa os handlers em ordem, incluindo operações de armazenamento e comandos NUS. A fila tem 16 entradas; quando cheia, a escrita retorna falta de recursos. STOP limpa a fila e executa após a operação corrente. Ele não interrompe uma gravação de flash que já começou.
- Comandos manuais de PWM são rejeitados enquanto a navegação está ativa, evitando que duas origens sobrescrevam os mesmos motores. Parar a navegação antes de usar os sliders. Erros de PWM agora geram STATUS.
- STATUS e respostas de mapas mantêm ordem FIFO. Transferências de mapas não apagam mais respostas já enfileiradas. Telemetria periódica só é produzida quando o TX está livre, evitando acumular amostras antigas.

## Odometria

Não foram alterados os cálculos, fontes, resets ou prioridades de controle, encoders e odometria. O processamento de encoders/odometria tem alvo de 1 kHz e a tarefa de odometria mantém prioridade 20, acima dos comandos BLE (5) e TX (4).

Apenas o transporte de amostras foi ajustado:

| Informação | Alvo nominal de envio |
| --- | --- |
| Pose fundida, IMU rápida e linha rápida | 60 Hz |
| Odometria completa e encoders | 15 Hz, rodízio de quatro slots |
| Estado do controle | 15 Hz |
| Linha completa | Até 30 Hz, conforme espaço no pacote |

A pose é inserida primeiro. Os estados regulares vêm antes da linha completa para evitar que ela ocupe o espaço necessário ao controle/odometria. Mesmo nas janelas curtas de transferência de mapas, há oportunidades de envio da odometria completa. Comandos comuns não suspendem mais os estados regulares por 300 ms.

Essas taxas são alvos, não garantias de entrega. Dependem do MTU, intervalo negociado, disponibilidade de TX e carga. MTU preferido permanece 256; com MTU 23 a pose ainda cabe, mas vários estados completos não cabem. O intervalo solicitado passou a 7,5–15 ms, sem latência de conexão; a central pode escolher outro intervalo.

## Outros serviços e diagnósticos

O log do componente BLE fica em WARN para evitar logs por evento/notificação. A tarefa de comandos informa esperas ou execuções acima de 50 ms. O desktop informa fila+ATT acima de 150 ms; esse tempo mede aceitação ATT, não a atuação física do motor, pois a execução agora ocorre em uma tarefa separada.

O log de contagem do portal foi movido para fora da seção crítica. Nenhuma prioridade de aquisição/odometria foi reduzida. Gravações em flash ainda podem introduzir pausas no sistema; mover os handlers para uma tarefa não elimina esse efeito. STATUS permanece sem identificador de comando no protocolo atual, portanto não é uma confirmação individual correlacionada.

## Verificação

Correção do START/STOP da gravação: `MAP_RECORD_STOP` (0x23) também tem prioridade de parada no desktop e firmware. START/STOP enviam um snapshot da gravação; a interface usa um único timer de consulta e aguarda o estado confirmado. Snapshots antigos não desfazem o último clique. Parar a gravação interrompe a inclusão de pontos no mapa, mas mantém o cálculo da odometria e a visualização da posição ao vivo.

- `python -m unittest discover -s engineering_ui/tests -v`: regressões de STOP, coalescência, saturação, retentativas, desconexão, timeout e falha antes de START.
- `python -m compileall -q engineering_ui`: sintaxe da interface.
- Em `mobile_app`, `npm run validate`: tipos e testes da fila/protocolo, incluindo decodificação de odometria.
- Build ESP-IDF 5.3: `build/aspirador_de_pista.bin`.

Não houve gravação no robô nem medição de rádio nesta alteração. Para validar em bancada, comparar `track_odometry_loop_hz` e `imu_loop_hz` em repouso, ajustando PWM e transferindo mapas; verificar continuidade de pose e odometria completa. Com o robô suspenso, testar STOP durante uma sequência e confirmar ausência de PWM antigo após a parada. Medir latência física separadamente do tempo ATT e registrar MTU/intervalo efetivamente negociados.
