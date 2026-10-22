# Capivara-inator 20000

Robô de sumô autônomo em ESP32, escrito em C com ESP-IDF e FreeRTOS.

Projeto da disciplina de Sistemas Embarcados.

## O que ele faz

1. Aguarda o botão de partida e faz a contagem regressiva de 5 s.
2. Sorteia uma entre quatro estratégias de inicialização, para ficar imprevisível a cada partida.
3. Caça o oponente com dois sensores ultrassônicos (frente e trás).
4. Monitora a borda da arena com dois sensores de linha, que têm prioridade sobre tudo.

## Hardware

| Função | GPIO |
|---|---|
| Botão de partida | 32 |
| Sonar frente — TRIG / ECHO | 5 / 18 |
| Sonar trás — TRIG / ECHO | 4 / 16 |
| Sensor de linha frente / trás | 34 / 35 |
| Ponte H — motor esq. frente / ré | 33 / 13 |
| Ponte H — motor dir. frente / ré | 14 / 15 |
| LEDs de direção | 19, 21, 22, 23 |
| LEDs de faixa de distância | 25, 26, 27 |

Componentes: ESP32 DevKit (CP2102), 2x HC-SR04, 2x módulo LDR, ponte H, 7 LEDs.

### Cuidados de montagem

- O `ECHO` do HC-SR04 entrega 5 V e o ESP32 não é 5 V-tolerante. Cada um precisa de
  divisor resistivo (2,2 kΩ + 3,3 kΩ) antes de chegar ao pino.
- GPIO34 e GPIO35 são input-only e não possuem pull interno. Funcionam porque o módulo
  LDR aciona o `DO` ativamente; um resistor externo de 10 kΩ para o GND protege contra
  fio solto.
- O motor esquerdo usa o **GPIO33**, não o GPIO12. O GPIO12 é strapping pin (MTDI) e em
  nível alto no reset configura a flash para 1,8 V, impedindo o boot.
- Os jumpers de `ENA` e `ENB` da ponte H ficam no lugar: o PWM sai nos próprios IN1–IN4.

## Arquitetura do firmware

Três contextos de execução, para que nada bloqueie a leitura dos sensores:

| Tarefa | Prioridade | Período | Papel |
|---|---|---|---|
| `linha_task` | 8 | 10 ms | Amostra os LDRs com debounce |
| `sonar_task` | 6 | 35 ms | Dispara os sonares alternadamente |
| `app_main` | 1 | 10 ms | Máquina de estados |

Decisões de projeto que valem nota:

- **Medição do ECHO por interrupção.** A ISR marca as duas bordas do pulso e entrega a
  largura por fila. Sem isso, cada leitura prenderia a CPU em espera ocupada por até
  30 ms, deixando o robô cego para a borda da arena.
- **Disparo alternado dos sonares.** Se os dois disparassem juntos, um ouviria o eco do
  outro (cross-talk) e leria distância fantasma.
- **"Sem eco" não é "0 cm".** Um HC-SR04 sadio sempre levanta o ECHO após o disparo,
  mesmo sem obstáculo. Se o pulso vier longo demais, é caminho livre; se nem a subida
  vier, o módulo está mudo. Tratar os dois como zero faria o robô entender "oponente
  colado na frente" e avançar a toda, para fora da arena.
- **Filtro de mediana de 3.** Descarta as leituras absurdas isoladas do HC-SR04 sem
  atrasar a resposta.
- **Teto de tempo em toda manobra de evasão.** Um sensor travado não pode fazer o robô
  girar indefinidamente.
- **Helper `delay_ms()`.** O tick do FreeRTOS é 100 Hz por padrão, então
  `pdMS_TO_TICKS(5)` resulta em zero ticks — e `vTaskDelay(0)` não cede a CPU. Numa
  tarefa de prioridade alta isso trava o núcleo e dispara o task watchdog. O helper
  garante no mínimo 1 tick.

### Requisitos da atividade

| Requisito | Onde está |
|---|---|
| Quatro estratégias de inicialização | `estrategia_inicial()` |
| Evasão pelo sensor frontal | `evasao_frente()` |
| Evasão pelo sensor traseiro | `evasao_tras()` |
| Prioridade na detecção simultânea | `evasao_ambos()` |
| Timer de segurança de 500 ms | `sair_da_linha()` + `T_EMPURRADO_MS` |
| Memória de posição | `lado_oposto()` |
| Modo de sobrevivência | `modo_sobrevivencia()` |

## Compilar e gravar

```bash
pio run            # compila
pio run -t upload  # grava na placa
pio device monitor # monitor serial a 115200
```

O driver CP210x da Silicon Labs precisa estar instalado para a porta COM aparecer.

## Simulação

O projeto inclui `diagram.json` e `wokwi.toml`, então roda no simulador Wokwi sem
hardware nenhum. Instale a extensão *Wokwi Simulator* no VS Code e use
`F1 → Wokwi: Start Simulator`.

Na simulação: clique e segure o botão para armar, arraste o slider de distância de um
HC-SR04 para simular o oponente, e puxe o *illumination* de um módulo LDR até o mínimo
para simular a borda da arena.

## Calibração

Os parâmetros ficam agrupados no bloco "2. PARÂMETROS CALIBRÁVEIS" do `src/main.c`:
velocidades, tempos de manobra e limiares de decisão.

Os tempos de giro (`T_GIRO_90_MS` e `T_GIRO_180_MS`) dependem do motor, da roda e do
peso do robô. Nenhum simulador resolve isso — tem que cronometrar na arena.
