// =============================================================================
//  CAPIVARA-INATOR 20000 - ROBO DE SUMO
//  ESP32 / ESP-IDF / FreeRTOS
//  2x HC-SR04 (frente e tras) + 2x LDR de linha + ponte H com PWM
// -----------------------------------------------------------------------------
//  O QUE MUDOU EM RELACAO A VERSAO ANTERIOR
//
//  1) SENSOR MUDO NAO E MAIS "0 cm"
//     Antes, se o ultrassonico nao respondesse, ler_distancia() devolvia 0.
//     Como 0 < 90, o robo entendia "oponente colado na frente" e avancava a
//     toda - para fora da arena. Agora: um HC-SR04 sadio SEMPRE levanta o ECHO
//     apos o disparo, mesmo sem obstaculo. Entao
//        subiu o ECHO mas o pulso foi longo demais -> DIST_LIVRE
//        nem chegou a subir                        -> DIST_FALHA
//
//  2) NADA MAIS BLOQUEIA O LACO DE CONTROLE
//     As leituras eram busy-wait de ate 30 ms, duas por volta. Durante esses
//     ~120 ms o robo ficava CEGO para os LDRs, ou seja, cego para a borda da
//     arena. Agora o ECHO e medido por interrupcao, em tarefa propria.
//
//  3) TAREFA DE LINHA COM PRIORIDADE MAXIMA
//     Os LDRs sao amostrados a cada 5 ms com debounce, numa tarefa que nunca
//     bloqueia. A borda da arena nunca passa despercebida.
//
//  4) SONARES ALTERNADOS
//     Nunca disparam juntos - senao um ouve o eco do outro (cross-talk).
//
//  5) FILTRO DE MEDIANA (3 amostras)
//     Descarta as leituras absurdas isoladas tipicas do HC-SR04.
//
//  6) TODO LACO DE EVASAO TEM TETO DE TEMPO
//     evasao_ambos() girava enquanto os LDRs vissem linha, sem limite: um fio
//     solto fazia o robo rodar em circulo ate a bateria acabar.
//
//  7) CONTROLE DE VELOCIDADE POR PWM (LEDC)
//     Os 4 pinos da ponte H agora saem por PWM. Ataque com forca total, busca
//     e correcoes de borda em velocidade reduzida (bem mais preciso).
//     >>> Mantenha os jumpers de ENA/ENB do L298N no lugar (enable em nivel
//         alto). O PWM vai nos proprios pinos IN1..IN4.
//
//  8) BOTAO COM DEBOUNCE E LIGA/DESLIGA
//
//  9) MOTOR_ESQ_F MOVIDO DO GPIO12 PARA O GPIO33
//     GPIO12 e strapping pin (MTDI): em nivel alto no reset ele configura a
//     flash para 1,8 V e a placa nao da boot. Com o pull-up da entrada da
//     ponte H, isso ia acontecer toda vez.
// =============================================================================

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"    // esp_rom_delay_us() - substitui o antigo ets_delay_us()
#include "esp_random.h"

// =============================================================================
// 1. PINAGEM
// =============================================================================

#define PINO_BOTAO        GPIO_NUM_32

#define PINO_TRIG_FRENTE  GPIO_NUM_5  //fio vermelho
#define PINO_ECHO_FRENTE  GPIO_NUM_18 //fio vermelho
#define PINO_TRIG_TRAS    GPIO_NUM_4 //fio laranja
#define PINO_ECHO_TRAS    GPIO_NUM_16 //fio laranja 

// GPIO34/35 sao INPUT-ONLY e nao possuem pull interno. Funcionam porque o
// modulo LDR aciona o DO ativamente. Se um fio soltar, o pino flutua: se
// quiser blindagem, coloque 10k externo para o GND em cada um.


#define PINO_LDR_FRENTE   GPIO_NUM_34
#define PINO_LDR_TRAS     GPIO_NUM_35

#define MOTOR_ESQ_F       GPIO_NUM_33   // era GPIO12 (strapping - nao usar) fio marrom
#define MOTOR_ESQ_T       GPIO_NUM_13 //fio marrom
#define MOTOR_DIR_F       GPIO_NUM_14 //fio branco
#define MOTOR_DIR_T       GPIO_NUM_15 //fio branco 





#define LED_FRENTE        GPIO_NUM_19
#define LED_TRAS          GPIO_NUM_21
#define LED_DIREITA       GPIO_NUM_22
#define LED_ESQUERDA      GPIO_NUM_23

#define LED_DIST_VERMELHO GPIO_NUM_25
#define LED_DIST_AZUL     GPIO_NUM_26
#define LED_DIST_LARANJA  GPIO_NUM_27

// =============================================================================
// 2. PARAMETROS CALIBRAVEIS
// =============================================================================

// ---- Sensores de linha ------------------------------------------------------
// Nivel do DO quando o sensor esta sobre a linha.
//
// Medido no modulo real em 23/10: tapado (escuro) o DO vai para 1, exposto
// (claro) vai para 0. Portanto MAIS LUZ = 0. Como a borda da arena e uma
// linha BRANCA, que reflete mais luz que o piso preto, "estou na linha"
// corresponde a 0.
//
// O valor 1 que estava aqui antes veio da calibracao no simulador Wokwi, cujo
// modulo LDR responde ao contrario do modulo fisico.
#define LINHA_NIVEL_ATIVO      0
#define T_AMOSTRA_LINHA_MS     5   // periodo da tarefa de linha
#define DEBOUNCE_LINHA         2   // leituras iguais seguidas para validar

// Permite rodar com um sensor de linha so. Um LDR desligado ou queimado deixa o
// pino flutuando, e pino flutuante tende a ler 1 - que o codigo entende como
// "estou na borda". O robo entao fica preso em evasao para sempre.
// Ponha 0 no sensor que nao existe ainda e ele passa a ser ignorado.
//
// ATENCAO: com um sensor desligado o robo perde a protecao de borda daquele
// lado. Serve para bancada; antes da competicao os dois precisam estar em 1.
#define USAR_LDR_FRENTE        1
#define USAR_LDR_TRAS          0

// ---- Ultrassonicos ----------------------------------------------------------
// T_PING_MS e o intervalo entre disparos ALTERNADOS: cada sensor e lido a cada
// 70 ms, acima do minimo de 60 ms do datasheet, com folga para o eco do outro
// ja ter morrido.
#define T_PING_MS             35
#define TIMEOUT_ECHO_MS       40
#define N_MEDIANA              3
#define DIST_MIN_CM            2
#define DIST_MAX_CM          400
#define DIST_FALHA          (-1)
#define DIST_LIVRE   DIST_MAX_CM

// ---- Decisao ----------------------------------------------------------------
#define D_OPONENTE            90   // cm - abaixo disso considera que achou alguem
#define D_ATAQUE              30   // cm - abaixo disso vai com tudo

// ---- Velocidades (0..100 %) -------------------------------------------------
#define VEL_ATAQUE           100
#define VEL_EVASAO            95
#define VEL_GIRO              80
#define VEL_BUSCA             65
#define VEL_SUAVE             45   // modo sobrevivencia

// ---- Tempos (ms) - calibrar no robo real ------------------------------------
#define T_CONTROLE_MS         10
#define T_EMPURRADO_MS       500   // linha por mais que isso = sendo empurrado
#define T_MAX_NA_LINHA      1500   // teto de qualquer manobra sobre a linha
#define T_MAX_EMERGENCIA    2500   // teto do giro de emergencia
#define T_GIRO_90_MS         220
#define T_GIRO_180_MS        440
#define T_AVANCO_CENTRO_MS   300
#define T_SUAVE_MS           120

// ---- Botao ------------------------------------------------------------------
#define T_DEBOUNCE_MS         30
#define SEG_ARMAR              5

// ---- PWM --------------------------------------------------------------------
// 1 kHz e confortavel para L298N. TB6612 aceita bem mais.
#define PWM_FREQ            1000
#define PWM_MODE   LEDC_LOW_SPEED_MODE
#define PWM_TIMER  LEDC_TIMER_0
#define PWM_RES    LEDC_TIMER_8_BIT

// =============================================================================
// 3. TIPOS E ESTADO
// =============================================================================

typedef enum {
    MOV_PARADO = 0,
    MOV_FRENTE,
    MOV_TRAS,
    MOV_DIREITA,
    MOV_ESQUERDA,
    MOV_CURVA_ESQ,
    MOV_CURVA_DIR
} mov_t;

static const char *MOV_NOME[] = { "PARADO", "FRENTE", "TRAS", "GIRA DIREITA",
                                  "GIRA ESQUERDA", "CURVA ESQUERDA", "CURVA DIREITA" };

#define N_SONARES 2
#define S_FRENTE  0
#define S_TRAS    1

typedef struct {
    gpio_num_t       trig;
    gpio_num_t       echo;
    const char      *nome;
    QueueHandle_t    fila;
    volatile int64_t t_subida;
    volatile bool    subiu;
    long             jan[N_MEDIANA];
    int              idx;
    long             dist;
    uint32_t         seq;
} sonar_t;

static sonar_t g_sonar[N_SONARES] = {
    { .trig = PINO_TRIG_FRENTE, .echo = PINO_ECHO_FRENTE, .nome = "FRENTE" },
    { .trig = PINO_TRIG_TRAS,   .echo = PINO_ECHO_TRAS,   .nome = "TRAS"   },
};

typedef struct {
    bool    frente, tras;
    int64_t frente_desde, tras_desde;   // ms em que entrou na linha; -1 = fora
} linha_t;

static linha_t      g_linha = { false, false, -1, -1 };
static portMUX_TYPE g_lock  = portMUX_INITIALIZER_UNLOCKED;

static bool  g_robo_ativo    = false;
static bool  g_inicio_feito  = false;
static mov_t g_ultimo_lado   = MOV_DIREITA;   // memoria de posicao

static inline int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

// ARMADILHA DO FreeRTOS: o tick do ESP-IDF e 100 Hz por padrao, entao
// pdMS_TO_TICKS(5) da ZERO ticks - e vTaskDelay(0) NAO cede a CPU. Uma tarefa
// de prioridade alta com delay zero trava o nucleo e dispara o task watchdog.
// Este helper garante no minimo 1 tick em qualquer delay.
static inline void delay_ms(int ms)
{
    TickType_t t = pdMS_TO_TICKS(ms);
    vTaskDelay(t ? t : 1);
}

// =============================================================================
// 4. MOTORES (PWM via LEDC)
// =============================================================================

#define CH_EF LEDC_CHANNEL_0
#define CH_ET LEDC_CHANNEL_1
#define CH_DF LEDC_CHANNEL_2
#define CH_DT LEDC_CHANNEL_3

static void pwm_bind(ledc_channel_t ch, gpio_num_t pino)
{
    ledc_channel_config_t c = {
        .gpio_num   = pino,
        .speed_mode = PWM_MODE,
        .channel    = ch,
        .timer_sel  = PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE,
    };
    ledc_channel_config(&c);
}

static void pwm_set(ledc_channel_t ch, int pct)
{
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    ledc_set_duty(PWM_MODE, ch, (pct * 255) / 100);
    ledc_update_duty(PWM_MODE, ch);
}

// esq / dir de -100 (re total) a +100 (frente total)
static void motores(int esq, int dir)
{
    if (esq >  100) esq =  100;
    if (esq < -100) esq = -100;
    if (dir >  100) dir =  100;
    if (dir < -100) dir = -100;

    pwm_set(CH_EF, esq > 0 ?  esq : 0);
    pwm_set(CH_ET, esq < 0 ? -esq : 0);
    pwm_set(CH_DF, dir > 0 ?  dir : 0);
    pwm_set(CH_DT, dir < 0 ? -dir : 0);
}

static void acender_leds(int f, int t, int d, int e)
{
    gpio_set_level(LED_FRENTE,   f);
    gpio_set_level(LED_TRAS,     t);
    gpio_set_level(LED_DIREITA,  d);
    gpio_set_level(LED_ESQUERDA, e);
}

// Unico ponto que comanda o robo. 'vel' e a velocidade em porcentagem.
static void aplicar(mov_t m, int vel)
{
    static mov_t ultimo_mov = MOV_PARADO;
    static int   ultima_vel = -1;

    switch (m) {
        case MOV_FRENTE:    motores( vel,  vel); acender_leds(1,0,0,0); break;
        case MOV_TRAS:      motores(-vel, -vel); acender_leds(0,1,0,0); break;
        case MOV_DIREITA:   motores( vel, -vel); acender_leds(0,0,1,0); break;
        case MOV_ESQUERDA:  motores(-vel,  vel); acender_leds(0,0,0,1); break;
        case MOV_CURVA_ESQ: motores(   0,  vel); acender_leds(1,0,0,1); break;
        case MOV_CURVA_DIR: motores( vel,    0); acender_leds(1,0,1,0); break;
        default:            motores(   0,    0); acender_leds(0,0,0,0); break;
    }

    if (m != ultimo_mov || vel != ultima_vel) {
        printf("  [MOV] %-14s vel=%d%%\n", MOV_NOME[m], (m == MOV_PARADO) ? 0 : vel);
        ultimo_mov = m;
        ultima_vel = vel;
    }
}

static void parar_motores(void) { aplicar(MOV_PARADO, 0); }

// =============================================================================
// AUTOTESTE DOS LEDS
// =============================================================================
// Acende os sete LEDs em sequencia, 300 ms cada, logo apos o boot.
//
// Serve para conferir antes de cada luta que nenhum indicador esta queimado,
// com fio solto ou no furo errado. Como o nome de cada LED sai no serial junto
// com o acendimento, da para identificar na hora qual nao respondeu.
//
// Os LEDs de movimento so acendem durante manobras especificas, e alguns
// (como o de re) podem demorar muito para aparecer sozinhos - este teste
// exercita todos de uma vez, sem depender do que o robo resolver fazer.

static void autoteste_leds(void)
{
    const gpio_num_t pinos[] = {
        LED_FRENTE, LED_TRAS, LED_DIREITA, LED_ESQUERDA,
        LED_DIST_VERMELHO, LED_DIST_AZUL, LED_DIST_LARANJA
    };
    const char *nomes[] = {
        "FRENTE (G19)", "TRAS (G21)", "DIREITA (G22)", "ESQUERDA (G23)",
        "DIST <10 (G25)", "DIST 10-30 (G26)", "DIST 30-60 (G27)"
    };
    const int n = sizeof(pinos) / sizeof(pinos[0]);

    printf("\n--- autoteste de LEDs ---\n");

    for (int i = 0; i < n; i++) gpio_set_level(pinos[i], 0);

    for (int i = 0; i < n; i++) {
        printf("  %d/%d  %s\n", i + 1, n, nomes[i]);
        gpio_set_level(pinos[i], 1);
        delay_ms(300);
        gpio_set_level(pinos[i], 0);
    }

    // Pisca os sete juntos duas vezes para fechar.
    for (int k = 0; k < 2; k++) {
        for (int i = 0; i < n; i++) gpio_set_level(pinos[i], 1);
        delay_ms(200);
        for (int i = 0; i < n; i++) gpio_set_level(pinos[i], 0);
        delay_ms(200);
    }

    printf("--- fim do autoteste ---\n\n");
}

// =============================================================================
// 5. SENSORES DE LINHA (tarefa de prioridade maxima)
// =============================================================================

static void linha_task(void *arg)
{
    (void)arg;
    const gpio_num_t pino[2] = { PINO_LDR_FRENTE, PINO_LDR_TRAS };
    bool     est[2]   = { false, false };
    uint8_t  cnt[2]   = { 0, 0 };
    int64_t  desde[2] = { -1, -1 };

    const bool habilitado[2] = { USAR_LDR_FRENTE, USAR_LDR_TRAS };

    while (true) {
        for (int i = 0; i < 2; i++) {
            if (!habilitado[i]) {        // sensor desligado: nunca acusa linha
                est[i]   = false;
                desde[i] = -1;
                continue;
            }
            bool bruto = (gpio_get_level(pino[i]) == LINHA_NIVEL_ATIVO);
            if (bruto != est[i]) {
                if (++cnt[i] >= DEBOUNCE_LINHA) {
                    est[i]   = bruto;
                    cnt[i]   = 0;
                    desde[i] = bruto ? now_ms() : -1;
                }
            } else {
                cnt[i] = 0;
            }
        }

        portENTER_CRITICAL(&g_lock);
        g_linha.frente = est[0];  g_linha.frente_desde = desde[0];
        g_linha.tras   = est[1];  g_linha.tras_desde   = desde[1];
        portEXIT_CRITICAL(&g_lock);

        // DIAGNOSTICO: imprime o nivel CRU dos pinos uma vez por segundo.
        // Fica aqui dentro da tarefa de linha de proposito - ela nunca bloqueia,
        // entao a leitura aparece mesmo quando o laco principal esta preso numa
        // manobra de evasao. Tape o sensor com a mao: se o numero nao mudar, o
        // problema e o potenciometro do modulo, nao o codigo.
        static int64_t t_dbg = 0;
        if (now_ms() - t_dbg > 1000) {
            printf("LDR cru: G%d=%d  G%d=%d  (linha quando vale %d)   BOTAO G%d=%d\n",
                   PINO_LDR_FRENTE, gpio_get_level(PINO_LDR_FRENTE),
                   PINO_LDR_TRAS,   gpio_get_level(PINO_LDR_TRAS),
                   LINHA_NIVEL_ATIVO,
                   PINO_BOTAO,      gpio_get_level(PINO_BOTAO));
            t_dbg = now_ms();
        }

        delay_ms(T_AMOSTRA_LINHA_MS);
    }
}

static linha_t linha_get(void)
{
    linha_t c;
    portENTER_CRITICAL(&g_lock);
    c = g_linha;
    portEXIT_CRITICAL(&g_lock);
    return c;
}

static bool linha_frente(void) { return linha_get().frente; }
static bool linha_tras(void)   { return linha_get().tras;   }
static bool alguma_linha(void) { linha_t c = linha_get(); return c.frente || c.tras; }

// =============================================================================
// 6. ULTRASSONICOS (medicao por interrupcao)
// =============================================================================

static void IRAM_ATTR echo_isr(void *arg)
{
    sonar_t *s     = &g_sonar[(int)(intptr_t)arg];
    int64_t  agora = esp_timer_get_time();

    if (gpio_get_level(s->echo)) {
        s->t_subida = agora;
        s->subiu    = true;
    } else if (s->subiu) {
        int64_t dur = agora - s->t_subida;
        BaseType_t acordou = pdFALSE;
        xQueueSendFromISR(s->fila, &dur, &acordou);
        if (acordou) portYIELD_FROM_ISR();
    }
}

static long medir(sonar_t *s)
{
    int64_t lixo;
    while (xQueueReceive(s->fila, &lixo, 0) == pdTRUE) { }   // limpa ecos antigos
    s->subiu = false;

    gpio_set_level(s->trig, 0);
    esp_rom_delay_us(3);
    gpio_set_level(s->trig, 1);
    esp_rom_delay_us(10);
    gpio_set_level(s->trig, 0);

    int64_t dur_us;
    if (xQueueReceive(s->fila, &dur_us, pdMS_TO_TICKS(TIMEOUT_ECHO_MS)) != pdTRUE)
        return s->subiu ? DIST_LIVRE : DIST_FALHA;

    long cm = (long)(dur_us / 58);            // ida e volta: ~58 us por cm
    if (cm < DIST_MIN_CM) cm = DIST_MIN_CM;
    if (cm > DIST_MAX_CM) cm = DIST_LIVRE;
    return cm;
}

// DIST_FALHA vale -1 e afunda para o menor valor: uma falha isolada e
// descartada, duas seguidas resultam em DIST_FALHA.
static long mediana3(long a, long b, long c)
{
    if (a > b) { long t = a; a = b; b = t; }
    if (b > c) { long t = b; b = c; c = t; }
    if (a > b) { long t = a; a = b; b = t; }
    return b;
}

static void sonar_task(void *arg)
{
    (void)arg;
    int k = 0;

    while (true) {
        sonar_t *s = &g_sonar[k];

        s->jan[s->idx] = medir(s);
        s->idx = (s->idx + 1) % N_MEDIANA;

        long filtrada = mediana3(s->jan[0], s->jan[1], s->jan[2]);

        portENTER_CRITICAL(&g_lock);
        s->dist = filtrada;
        s->seq++;
        portEXIT_CRITICAL(&g_lock);

        // DIAGNOSTICO: a telemetria de sonar do laco principal so aparece
        // depois que o botao arma o robo. Esta linha sai sempre, inclusive em
        // espera, para dar para conferir a fiacao antes do botao existir.
        // DIST_FALHA (-1) = modulo mudo; 400 = nada dentro do alcance.
        static int64_t t_dbg = 0;
        if (now_ms() - t_dbg > 1000) {
            printf("sonar cru: FRENTE=%ld cm  TRAS=%ld cm\n",
                   g_sonar[S_FRENTE].dist, g_sonar[S_TRAS].dist);
            t_dbg = now_ms();
        }

        k = (k + 1) % N_SONARES;
        delay_ms(T_PING_MS);
    }
}

static long dist_de(int i)
{
    long d;
    portENTER_CRITICAL(&g_lock);
    d = g_sonar[i].dist;
    portEXIT_CRITICAL(&g_lock);
    return d;
}

// Sensor mudo nao deve travar o robo: conta como "nada a vista".
static long util_de(int i)
{
    long d = dist_de(i);
    return (d == DIST_FALHA) ? DIST_LIVRE : d;
}

// =============================================================================
// 7. LEDs DE DISTANCIA
// =============================================================================

static void atualizar_leds_distancia(void)
{
    static bool pisca = false;

    bool falha = (dist_de(S_FRENTE) == DIST_FALHA);
    long d     = util_de(S_FRENTE);

    if (falha) {                       // sensor mudo: vermelho piscando
        pisca = !pisca;
        gpio_set_level(LED_DIST_VERMELHO, pisca);
        gpio_set_level(LED_DIST_AZUL,     0);
        gpio_set_level(LED_DIST_LARANJA,  0);
        return;
    }

    // Faixas sem ambiguidade (a versao antiga usava azul para 10-30 E 60-90)
    gpio_set_level(LED_DIST_VERMELHO, d < 10);
    gpio_set_level(LED_DIST_AZUL,     (d >= 10 && d < 30) || (d >= 60 && d < 90));
    gpio_set_level(LED_DIST_LARANJA,  (d >= 30 && d < 60) || (d >= 60 && d < 90));
}

// =============================================================================
// 8. MOVIMENTO COM TEMPO E INTERRUPCAO
// =============================================================================
// Mantem o movimento por 'ms' sem nunca bloquear a leitura dos sensores.
// Com 'abortar_na_linha', devolve false assim que qualquer LDR ver a borda.

static bool executar(mov_t m, int vel, int ms, bool abortar_na_linha)
{
    aplicar(m, vel);
    int64_t t0 = now_ms();

    while ((now_ms() - t0) < ms) {
        atualizar_leds_distancia();
        if (abortar_na_linha && alguma_linha()) return false;
        delay_ms(T_CONTROLE_MS);
    }
    return true;
}

// MEMORIA DE POSICAO: alterna o lado a cada evasao, para nao repetir o erro.
static mov_t lado_oposto(void)
{
    g_ultimo_lado = (g_ultimo_lado == MOV_DIREITA) ? MOV_ESQUERDA : MOV_DIREITA;
    return g_ultimo_lado;
}

// =============================================================================
// 9. MODO DE SOBREVIVENCIA
// =============================================================================
// Ficou mais de T_EMPURRADO_MS sobre a linha: esta sendo empurrado. Manobra
// brusca aqui costuma jogar o robo para fora - entao usa correcoes curtas.

static void modo_sobrevivencia(bool frente_na_linha)
{
    printf("  [SOBREVIVENCIA] %d ms sobre a linha: sendo empurrado\n", T_EMPURRADO_MS);

    mov_t lado = lado_oposto();
    if (frente_na_linha) {
        executar(MOV_TRAS, VEL_SUAVE, T_SUAVE_MS, false);
        executar(lado,     VEL_SUAVE, T_SUAVE_MS, false);
    } else {
        executar(MOV_FRENTE, VEL_SUAVE, T_SUAVE_MS, false);
        executar(lado,       VEL_SUAVE, T_SUAVE_MS, false);
    }
    parar_motores();
}

// =============================================================================
// 10. MANOBRAS DE EVASAO DA BORDA
// =============================================================================

// Mantem 'm' enquanto o sensor indicado continuar vendo a linha.
// Devolve true se saiu da linha; false se estourou o teto de tempo.
// O teto e o que impede o robo de girar para sempre com um sensor travado.
static bool sair_da_linha(mov_t m, int vel, bool olhar_frente)
{
    aplicar(m, vel);
    int64_t t0 = now_ms();

    while (olhar_frente ? linha_frente() : linha_tras()) {
        int64_t dt = now_ms() - t0;

        if (dt > T_EMPURRADO_MS) {          // timer de seguranca
            modo_sobrevivencia(olhar_frente);
            return false;
        }
        if (dt > T_MAX_NA_LINHA) {          // teto absoluto
            printf("  [AVISO] LDR %s nao liberou em %d ms\n",
                   olhar_frente ? "FRENTE" : "TRAS", T_MAX_NA_LINHA);
            return false;
        }
        delay_ms(T_CONTROLE_MS);
    }
    return true;
}

// FRENTE na linha: recuo rapido + virada brusca e volta ao centro.
static void evasao_frente(void)
{
    printf("\n[LINHA] FRENTE na borda -> recuo + virada\n");

    if (!sair_da_linha(MOV_TRAS, VEL_EVASAO, true)) return;

    mov_t lado = lado_oposto();
    printf("  -> girando para %s\n", MOV_NOME[lado]);
    executar(lado, VEL_GIRO, T_GIRO_90_MS, false);
    executar(MOV_FRENTE, VEL_BUSCA, T_AVANCO_CENTRO_MS, true);
}

// TRAS na linha: avanco rapido + inversao de direcao.
static void evasao_tras(void)
{
    printf("\n[LINHA] TRAS na borda -> avanco + inversao\n");

    if (!sair_da_linha(MOV_FRENTE, VEL_EVASAO, false)) return;

    mov_t lado = lado_oposto();
    printf("  -> girando para %s\n", MOV_NOME[lado]);
    executar(lado, VEL_GIRO, T_GIRO_180_MS, false);
}

// Os dois na linha: para e gira no proprio eixo ate um deles liberar.
static void evasao_ambos(void)
{
    printf("\n[LINHA] AMBOS na borda -> emergencia\n");

    parar_motores();
    delay_ms(80);

    mov_t lado = (esp_random() & 1) ? MOV_DIREITA : MOV_ESQUERDA;
    aplicar(lado, VEL_GIRO);

    int64_t t0 = now_ms();
    while (linha_frente() && linha_tras()) {
        if ((now_ms() - t0) > T_MAX_EMERGENCIA) {
            printf("  [AVISO] emergencia estourou %d ms\n", T_MAX_EMERGENCIA);
            break;
        }
        delay_ms(T_CONTROLE_MS);
    }

    g_ultimo_lado = lado;
    parar_motores();
}

// =============================================================================
// 11. ESTRATEGIAS DE INICIALIZACAO
// =============================================================================

static void estrategia_inicial(void)
{
    uint32_t tatica = esp_random() % 4;
    static const char *NOME[] = { "PADRAO (giro exploratorio)",
                                  "AGRESSIVA (ataque inicial)",
                                  "DEFENSIVA (recuo e sondagem)",
                                  "ALEATORIA" };
    printf("\n>>> ESTRATEGIA INICIAL: %s\n", NOME[tatica]);

    switch (tatica) {
        case 0:
            executar(MOV_CURVA_ESQ, VEL_BUSCA, 200, true);
            executar(MOV_ESQUERDA,  VEL_GIRO,  200, true);
            break;

        case 1:
            executar(MOV_FRENTE,  VEL_ATAQUE, 300, true);
            executar(MOV_DIREITA, VEL_GIRO,   100, true);
            executar(MOV_FRENTE,  VEL_ATAQUE, 400, true);
            break;

        case 2:
            executar(MOV_TRAS,    VEL_BUSCA, 150, true);
            executar(MOV_DIREITA, VEL_GIRO,  T_GIRO_180_MS, true);
            for (int i = 0; i < 3; i++) {
                if (!executar(MOV_CURVA_DIR, VEL_BUSCA, 180, true)) return;
                if (!executar(MOV_CURVA_ESQ, VEL_BUSCA, 180, true)) return;
            }
            break;

        default:
            for (int i = 0; i < 3; i++) {
                uint32_t r  = esp_random();
                int      ms = 120 + (r % 200);
                switch ((r >> 16) % 3) {
                    case 0:  executar((r & 0x100) ? MOV_DIREITA : MOV_ESQUERDA,
                                      VEL_GIRO, ms, true); break;
                    case 1:  executar(MOV_FRENTE, VEL_ATAQUE, ms, true); break;
                    default: executar(MOV_TRAS,   VEL_BUSCA,  ms, true); break;
                }
            }
            break;
    }
    printf(">>> Inicializacao concluida. Cacando o oponente.\n\n");
}

// =============================================================================
// 12. BOTAO
// =============================================================================

static void armar_robo(void)
{
    printf("\n>>> ARMADO. Partida em %d s\n", SEG_ARMAR);
    parar_motores();

    for (int s = SEG_ARMAR; s > 0; s--) {
        printf("    %d...\n", s);
        for (int k = 0; k < 10; k++) {
            gpio_set_level(LED_DIST_VERMELHO, k % 2);
            delay_ms(100);
        }
    }

    gpio_set_level(LED_DIST_VERMELHO, 0);
    g_robo_ativo   = true;
    g_inicio_feito = false;
    printf(">>> VALENDO\n");
}

// Debounce por estabilidade. Cada borda de subida alterna ligado/desligado.
static void tratar_botao(void)
{
    static bool    inicializado = false;
    static bool    estavel;
    static bool    ultimo_raw;
    static int64_t t_mudanca  = 0;

    bool raw = (gpio_get_level(PINO_BOTAO) == 1);

    // Na primeira chamada, adota o nivel ATUAL do pino como estado de partida.
    // Sem isto o codigo assumia "solto" no boot, e um pino que ja estivesse em
    // 1 nesse instante - botao segurado, preso ou travado por defeito - era
    // lido como um aperto e o robo armava sozinho ao ligar. Com motores no
    // lugar isso significa o robo sair andando logo apos um reset.
    if (!inicializado) {
        estavel      = raw;
        ultimo_raw   = raw;
        t_mudanca    = now_ms();
        inicializado = true;
        return;
    }

    if (raw != ultimo_raw) {
        ultimo_raw = raw;
        t_mudanca  = now_ms();
        return;
    }
    if ((now_ms() - t_mudanca) < T_DEBOUNCE_MS) return;
    if (raw == estavel) return;

    estavel = raw;
    if (!estavel) return;               // so age na borda de subida

    if (!g_robo_ativo) {
        armar_robo();
    } else {
        g_robo_ativo = false;
        parar_motores();
        printf("\n>>> PARADO pelo botao\n\n");
    }
}

// =============================================================================
// 13. CONFIGURACAO DE HARDWARE
// =============================================================================

static void configurar_hardware(void)
{
    // ---- Saidas digitais (TRIGs e LEDs) ----
    gpio_config_t saidas = {
        .pin_bit_mask = (1ULL << PINO_TRIG_FRENTE) | (1ULL << PINO_TRIG_TRAS) |
                        (1ULL << LED_FRENTE)       | (1ULL << LED_TRAS)       |
                        (1ULL << LED_DIREITA)      | (1ULL << LED_ESQUERDA)   |
                        (1ULL << LED_DIST_VERMELHO)| (1ULL << LED_DIST_AZUL)  |
                        (1ULL << LED_DIST_LARANJA),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&saidas);

    // ---- ECHOs: interrupcao nas DUAS bordas ----
    gpio_config_t echos = {
        .pin_bit_mask = (1ULL << PINO_ECHO_FRENTE) | (1ULL << PINO_ECHO_TRAS),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_ANYEDGE
    };
    gpio_config(&echos);

    // ---- LDRs: GPIO34/35 sao input-only, sem pull interno possivel ----
    gpio_config_t ldrs = {
        .pin_bit_mask = (1ULL << PINO_LDR_FRENTE) | (1ULL << PINO_LDR_TRAS),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&ldrs);

    // ---- Botao ----
    gpio_config_t botao = {
        .pin_bit_mask = (1ULL << PINO_BOTAO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&botao);

    gpio_set_level(PINO_TRIG_FRENTE, 0);
    gpio_set_level(PINO_TRIG_TRAS,   0);

    // ---- PWM dos motores ----
    ledc_timer_config_t t = {
        .speed_mode      = PWM_MODE,
        .timer_num       = PWM_TIMER,
        .duty_resolution = PWM_RES,
        .freq_hz         = PWM_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&t);
    pwm_bind(CH_EF, MOTOR_ESQ_F);
    pwm_bind(CH_ET, MOTOR_ESQ_T);
    pwm_bind(CH_DF, MOTOR_DIR_F);
    pwm_bind(CH_DT, MOTOR_DIR_T);
    motores(0, 0);

    // ---- ISR dos ECHOs ----
    gpio_install_isr_service(0);
    for (int i = 0; i < N_SONARES; i++) {
        g_sonar[i].fila = xQueueCreate(4, sizeof(int64_t));
        for (int j = 0; j < N_MEDIANA; j++) g_sonar[i].jan[j] = DIST_LIVRE;
        g_sonar[i].dist = DIST_LIVRE;
        gpio_isr_handler_add(g_sonar[i].echo, echo_isr, (void *)(intptr_t)i);
    }
}

// =============================================================================
// 14. LACO PRINCIPAL
// =============================================================================

void app_main(void)
{
    printf("\n=================================================\n");
    printf(" CAPIVARA-INATOR 20000 - robo de sumo\n");
    printf("   Sonar FRENTE: TRIG=GPIO%d ECHO=GPIO%d\n", PINO_TRIG_FRENTE, PINO_ECHO_FRENTE);
    printf("   Sonar TRAS  : TRIG=GPIO%d ECHO=GPIO%d\n", PINO_TRIG_TRAS,   PINO_ECHO_TRAS);
    printf("   LDR linha   : FRENTE=GPIO%d TRAS=GPIO%d\n", PINO_LDR_FRENTE, PINO_LDR_TRAS);
    printf("   Ponte H PWM : %d/%d/%d/%d @ %d Hz\n",
           MOTOR_ESQ_F, MOTOR_ESQ_T, MOTOR_DIR_F, MOTOR_DIR_T, PWM_FREQ);
    printf(" Aperte o botao (GPIO%d) para armar\n", PINO_BOTAO);
    printf("=================================================\n");

    configurar_hardware();
    autoteste_leds();

    // A linha tem prioridade MAIOR que o sonar: sair da arena e derrota certa.
    xTaskCreate(linha_task, "linha", 2560, NULL, 8, NULL);
    xTaskCreate(sonar_task, "sonar", 3072, NULL, 6, NULL);

    int64_t t_log = 0;

    while (true) {
        tratar_botao();
        atualizar_leds_distancia();

        // ---------------- Parado, esperando o botao ----------------
        if (!g_robo_ativo) {
            parar_motores();
            delay_ms(50);
            continue;
        }

        // ---------------- Estrategia de inicializacao ----------------
        if (!g_inicio_feito) {
            estrategia_inicial();
            g_inicio_feito = true;
            continue;
        }

        // ---------------- PRIORIDADE 1: borda da arena ----------------
        linha_t L = linha_get();
        if (L.frente && L.tras) { evasao_ambos();  continue; }
        if (L.frente)           { evasao_frente(); continue; }
        if (L.tras)             { evasao_tras();   continue; }

        // ---------------- PRIORIDADE 2: o oponente ----------------
        long df = util_de(S_FRENTE);
        long dt = util_de(S_TRAS);

        if (now_ms() - t_log > 500) {
            printf("sonar  frente=%ld cm  tras=%ld cm%s\n", df, dt,
                   (dist_de(S_FRENTE) == DIST_FALHA ||
                    dist_de(S_TRAS)   == DIST_FALHA) ? "   (algum sensor mudo)" : "");
            t_log = now_ms();
        }

        if (df < D_OPONENTE) {
            // Perto ataca com tudo; longe se aproxima em velocidade de cruzeiro.
            aplicar(MOV_FRENTE, (df < D_ATAQUE) ? VEL_ATAQUE : VEL_BUSCA);
        } else if (dt < D_OPONENTE) {
            // Algo atras: gira no eixo para trazer o oponente para a frente.
            aplicar(MOV_ESQUERDA, VEL_GIRO);
        } else {
            // Ninguem a vista: sonda a arena girando devagar (mais preciso).
            aplicar(MOV_DIREITA, VEL_BUSCA);
        }

        delay_ms(T_CONTROLE_MS);
    }
}
