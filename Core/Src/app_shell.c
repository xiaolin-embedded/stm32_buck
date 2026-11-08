/*
 * app_shell.c
 *
 * Minimal line-oriented shell on USART2 (PA2/PA3, 115200 8N1).
 * Reception is interrupt driven into a ring buffer; the shell task drains it
 * every 10 ms.  Only the shell task transmits, so no locking is required.
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "cmsis_os.h"
#include "app_config.h"
#include "app_power.h"
#include "app_shell.h"

UART_HandleTypeDef huart2;

#define SHELL_RX_RING_SIZE  256u
#define SHELL_LINE_SIZE     64u
#define SHELL_HISTORY_SIZE  10u

typedef struct
{
    const char *name;
    bool        takes_arg;
} ShellCmd_t;

static const ShellCmd_t s_cmd_table[] =
{
    { "help",      false },
    { "version",   false },
    { "ver",       false },
    { "status",    false },
    { "vset",      true  },
    { "iset",      true  },
    { "poweron",   false },
    { "poweroff",  false },
    { "monitor",   false },
    { "reset",     false },
};

static volatile uint8_t  s_rx_ring[SHELL_RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static char     s_line[SHELL_LINE_SIZE];
static uint16_t s_line_len;
static uint8_t  s_rx_byte;
static bool     s_ignore_lf;
static bool     s_monitor_on;
static uint32_t s_monitor_next;
static char     s_history[SHELL_HISTORY_SIZE][SHELL_LINE_SIZE];
static uint8_t  s_hist_count;
static uint8_t  s_hist_nav;
static uint8_t  s_esc_state;

/* -------------------------------------------------------------------------- */
/* Output                                                                     */
/* -------------------------------------------------------------------------- */
static void shell_write(const char *s, uint32_t len)
{
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)len, 200u);
}

static void shell_puts(const char *s)
{
    shell_write(s, (uint32_t)strlen(s));
}

static void shell_printf(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0)
    {
        if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
        shell_write(buf, (uint32_t)n);
    }
}

static void shell_prompt(void)
{
    shell_puts("\r\nbuck> ");
}

static void shell_redraw(void)
{
    shell_puts("\r\x1b[K");
    shell_puts("buck> ");
    if (s_line_len > 0u)
    {
        shell_write(s_line, s_line_len);
    }
}

static void history_add(const char *line)
{
    if (line[0] == '\0')
    {
        return;
    }

    if ((s_hist_count > 0u) && (strcmp(s_history[s_hist_count - 1u], line) == 0))
    {
        s_hist_nav = s_hist_count;
        return;
    }

    if (s_hist_count < SHELL_HISTORY_SIZE)
    {
        (void)strcpy(s_history[s_hist_count], line);
        s_hist_count++;
    }
    else
    {
        memmove(s_history[0], s_history[1],
                (SHELL_HISTORY_SIZE - 1u) * SHELL_LINE_SIZE);
        (void)strcpy(s_history[SHELL_HISTORY_SIZE - 1u], line);
    }
    s_hist_nav = s_hist_count;
}

static void history_up(void)
{
    if (s_hist_count == 0u)
    {
        return;
    }
    if (s_hist_nav > 0u)
    {
        s_hist_nav--;
    }
    (void)strcpy(s_line, s_history[s_hist_nav]);
    s_line_len = (uint16_t)strlen(s_line);
    shell_redraw();
}

static void history_down(void)
{
    if (s_hist_count == 0u)
    {
        return;
    }
    if (s_hist_nav < s_hist_count)
    {
        s_hist_nav++;
    }
    if (s_hist_nav == s_hist_count)
    {
        s_line_len = 0u;
    }
    else
    {
        (void)strcpy(s_line, s_history[s_hist_nav]);
        s_line_len = (uint16_t)strlen(s_line);
    }
    shell_redraw();
}

static void shell_tab_complete(void)
{
    const ShellCmd_t *matches[16];
    uint8_t n = 0u;
    uint8_t i;

    /* Only the command name is completed, not its arguments. */
    if (memchr(s_line, ' ', s_line_len) != NULL)
    {
        return;
    }

    for (i = 0u; i < (sizeof(s_cmd_table) / sizeof(s_cmd_table[0])); i++)
    {
        if (strncmp(s_cmd_table[i].name, s_line, s_line_len) == 0)
        {
            if (n < (sizeof(matches) / sizeof(matches[0])))
            {
                matches[n++] = &s_cmd_table[i];
            }
        }
    }

    if (n == 0u)
    {
        return;
    }

    if (n == 1u)
    {
        (void)strcpy(s_line, matches[0]->name);
        s_line_len = (uint16_t)strlen(s_line);
        if (matches[0]->takes_arg)
        {
            s_line[s_line_len++] = ' ';
            s_line[s_line_len] = '\0';
        }
        shell_redraw();
        return;
    }

    shell_puts("\r\n");
    for (i = 0u; i < n; i++)
    {
        shell_puts(matches[i]->name);
        shell_puts("  ");
    }
    shell_puts("\r\nbuck> ");
    if (s_line_len > 0u)
    {
        shell_write(s_line, s_line_len);
    }
}

/* -------------------------------------------------------------------------- */
/* Fixed point helpers                                                        */
/* -------------------------------------------------------------------------- */
static void fmt_volt(char *dst, size_t len, uint32_t mv)
{
    (void)snprintf(dst, len, "%lu.%03lu V", (unsigned long)(mv / 1000u), (unsigned long)(mv % 1000u));
}

static void fmt_amp(char *dst, size_t len, uint32_t ma)
{
    (void)snprintf(dst, len, "%lu.%03lu A", (unsigned long)(ma / 1000u), (unsigned long)(ma % 1000u));
}

static void fmt_watt(char *dst, size_t len, uint32_t mw)
{
    (void)snprintf(dst, len, "%lu.%03lu W", (unsigned long)(mw / 1000u), (unsigned long)(mw % 1000u));
}

/* -------------------------------------------------------------------------- */
/* Commands                                                                   */
/* -------------------------------------------------------------------------- */
static void cmd_help(void)
{
    shell_puts("\r\nCommands:");
    shell_puts("\r\n  version             firmware version / build info");
    shell_puts("\r\n  status              one-shot telemetry snapshot");
    shell_puts("\r\n  vset <mV>           set output voltage (1000..40000 mV)");
    shell_puts("\r\n  iset <mA>           set current limit (0..5000 mA)");
    shell_puts("\r\n  poweron             enable the output");
    shell_puts("\r\n  poweroff            disable the output");
    shell_puts("\r\n  monitor             1 Hz telemetry stream ('q' exits silently)");
    shell_puts("\r\n  reset               stop output and clear a latched fault");
    shell_puts("\r\n  help                this text");
    shell_puts("\r\n  (Up/Down: 10-entry history, Tab: command completion)");
}

static void cmd_ver(void)
{
    shell_printf("\r\n%s v%s", FW_NAME, FW_VERSION);
    shell_printf("\r\nBoard    : %s", FW_BOARD);
    shell_printf("\r\nBuild    : %s", FW_BUILD_DATE);
    shell_printf("\r\nPWM      : %lu kHz, control loop %lu kHz",
                 (unsigned long)(PWM_FREQ_HZ / 1000u),
                 (unsigned long)(CONTROL_FREQ_HZ / 1000u));
}

static void cmd_status(void)
{
    PowerTelemetry_t t;
    char vbuf[24];
    char ibuf[24];
    char pbuf[24];

    Power_GetTelemetry(&t);

    fmt_volt(vbuf, sizeof(vbuf), t.vin_mv);
    shell_printf("\r\nVin   : %s", vbuf);

    fmt_volt(vbuf, sizeof(vbuf), t.vout_mv);
    shell_printf("\r\nVout  : %s", vbuf);

    fmt_amp(ibuf, sizeof(ibuf), t.iout_ma);
    shell_printf("\r\nIout  : %s", ibuf);

    fmt_watt(pbuf, sizeof(pbuf), t.pout_mw);
    shell_printf("\r\nPout  : %s", pbuf);

    fmt_volt(vbuf, sizeof(vbuf), t.vset_mv);
    fmt_amp(ibuf, sizeof(ibuf), t.ilim_ma);
    shell_printf("\r\nSet   : %s / %s", vbuf, ibuf);

    {
        const char *mode = "";
        if (t.state == PWR_STATE_RUN)
        {
            mode = (t.cc_active != 0u) ? " (CC)" : " (CV)";
        }
        shell_printf("\r\nState : %s%s   duty %lu.%lu %%",
                     Power_StateName(t.state), mode,
                     (unsigned long)(t.duty_permille / 10u),
                     (unsigned long)(t.duty_permille % 10u));
    }

    if (t.fault != PWR_FAULT_NONE)
    {
        shell_printf("\r\nFault : %s (0x%02lX)", Power_FaultName(t.fault), (unsigned long)t.fault);
    }

    {
        uint32_t gv = (uint32_t)(Power_GetVinGain() * 10000.0f + 0.5f);
        uint32_t go = (uint32_t)(Power_GetVoutGain() * 10000.0f + 0.5f);
        uint32_t gi = (uint32_t)(Power_GetIoutGain() * 10000.0f + 0.5f);
        shell_printf("\r\nGain  : Vin %lu.%04lu  Vout %lu.%04lu  Iout %lu.%04lu",
                     (unsigned long)(gv / 10000u), (unsigned long)(gv % 10000u),
                     (unsigned long)(go / 10000u), (unsigned long)(go % 10000u),
                     (unsigned long)(gi / 10000u), (unsigned long)(gi % 10000u));
    }

    shell_printf("\r\nLoops : %lu", (unsigned long)t.loop_count);
}

static bool parse_uint(const char *text, uint32_t *value)
{
    char *end = NULL;
    unsigned long v;

    if (text == NULL || *text == '\0') return false;

    v = strtoul(text, &end, 10);
    if (end == text || *end != '\0') return false;

    *value = (uint32_t)v;
    return true;
}

static void cmd_vset(const char *arg)
{
    uint32_t mv;
    PowerTelemetry_t t;

    if (!parse_uint(arg, &mv))
    {
        shell_puts("\r\nusage: vset <mV>");
        return;
    }
    if (!Power_SetVout_mV(mv))
    {
        shell_printf("\r\nout of range: %lu..%lu mV", (unsigned long)VSET_MIN_MV, (unsigned long)VSET_MAX_MV);
        return;
    }

    Power_GetTelemetry(&t);
    if (t.vin_mv > 0u && mv + 1000u > t.vin_mv)
    {
        shell_printf("\r\nwarning: setpoint %lu mV is close to Vin %lu mV (buck cannot boost)",
                     (unsigned long)mv, (unsigned long)t.vin_mv);
    }
    shell_printf("\r\nvset = %lu mV", (unsigned long)mv);
}

static void cmd_iset(const char *arg)
{
    uint32_t ma;

    if (!parse_uint(arg, &ma))
    {
        shell_puts("\r\nusage: iset <mA>");
        return;
    }
    if (!Power_SetIlimit_mA(ma))
    {
        shell_printf("\r\nout of range: 0..%lu mA", (unsigned long)ILIM_MAX_MA);
        return;
    }
    shell_printf("\r\nilimit = %lu mA", (unsigned long)ma);
}

static void monitor_print_line(void)
{
    PowerTelemetry_t t;
    char vin[24];
    char vout[24];
    char iout[24];
    char pout[24];

    Power_GetTelemetry(&t);

    fmt_volt(vin, sizeof(vin), t.vin_mv);
    fmt_volt(vout, sizeof(vout), t.vout_mv);
    fmt_amp(iout, sizeof(iout), t.iout_ma);
    fmt_watt(pout, sizeof(pout), t.pout_mw);

    if (t.fault != PWR_FAULT_NONE)
    {
        shell_printf("\r\nVin=%s Vout=%s Iout=%s Pout=%s FAULT:%s",
                     vin, vout, iout, pout, Power_FaultName(t.fault));
    }
    else
    {
        const char *mode = "";
        if (t.state == PWR_STATE_RUN)
        {
            mode = (t.cc_active != 0u) ? "/CC" : "/CV";
        }
        shell_printf("\r\nVin=%s Vout=%s Iout=%s Pout=%s %s%s D=%lu.%lu%%",
                     vin, vout, iout, pout, Power_StateName(t.state), mode,
                     (unsigned long)(t.duty_permille / 10u),
                     (unsigned long)(t.duty_permille % 10u));
    }
}

static void cmd_monitor(void)
{
    if (s_monitor_on)
    {
        return;             /* already streaming */
    }

    s_monitor_on = true;
    s_monitor_next = osKernelGetTickCount();
    shell_puts("\r\nmonitor on: 1 Hz telemetry, 'q' to stop");
}

static void shell_execute(char *line)
{
    char *arg;

    while (*line == ' ') line++;
    if (*line == '\0') return;

    arg = strchr(line, ' ');
    if (arg != NULL)
    {
        *arg++ = '\0';
        while (*arg == ' ') arg++;
    }

    if (strcmp(line, "help") == 0)             { cmd_help(); }
    else if (strcmp(line, "version") == 0)     { cmd_ver(); }
    else if (strcmp(line, "ver") == 0)         { cmd_ver(); }
    else if (strcmp(line, "status") == 0)      { cmd_status(); }
    else if (strcmp(line, "vset") == 0)        { cmd_vset(arg); }
    else if (strcmp(line, "iset") == 0)        { cmd_iset(arg); }
    else if (strcmp(line, "poweron") == 0)
    {
        if (Power_Start()) shell_puts("\r\npower on, output enabled");
        else               shell_puts("\r\nrefused: fault latched, use 'reset'");
    }
    else if (strcmp(line, "poweroff") == 0)
    {
        Power_Stop();
        shell_puts("\r\npower off, output disabled");
    }
    else if (strcmp(line, "monitor") == 0)
    {
        if (arg != NULL) shell_puts("\r\nusage: monitor ('q' stops)");
        else             cmd_monitor();
    }
    else if (strcmp(line, "reset") == 0)
    {
        Power_Stop();
        (void)Power_ClearFault();
        shell_puts("\r\nfault cleared, output off");
    }
    else
    {
        shell_printf("\r\nunknown command: %s (try 'help')", line);
    }
}

/* -------------------------------------------------------------------------- */
/* RX path                                                                    */
/* -------------------------------------------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uint16_t next = (uint16_t)((s_rx_head + 1u) % SHELL_RX_RING_SIZE);
        if (next != s_rx_tail)
        {
            s_rx_ring[s_rx_head] = s_rx_byte;
            s_rx_head = next;
        }
        (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);
    }
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/* -------------------------------------------------------------------------- */
/* Task                                                                       */
/* -------------------------------------------------------------------------- */
static void shell_process_byte(uint8_t c)
{
    /* Swallow ANSI escape sequences (arrows); only Up/Down are used. */
    if (s_esc_state == 1u)
    {
        s_esc_state = ((c == '[') || (c == 'O')) ? 2u : 0u;
        return;
    }
    if (s_esc_state == 2u)
    {
        s_esc_state = 0u;
        if (!s_monitor_on)
        {
            if (c == 'A')      history_up();
            else if (c == 'B') history_down();
        }
        return;
    }
    if (c == 0x1Bu)
    {
        s_esc_state = 1u;
        return;
    }

    /* In monitor mode 'q' is a single-key hotkey: no echo, no Enter needed. */
    if (s_monitor_on && ((c == 'q') || (c == 'Q')))
    {
        s_monitor_on = false;
        s_line_len = 0u;
        shell_puts("\r\n");
        shell_prompt();
        return;
    }

    if (c == '\t')
    {
        if (!s_monitor_on)
        {
            shell_tab_complete();
        }
        return;
    }

    if (c != '\n')
    {
        s_ignore_lf = false;
    }

    if (c == '\r' || c == '\n')
    {
        if (c == '\r')
        {
            s_ignore_lf = true;
        }
        else if (s_ignore_lf)
        {
            s_ignore_lf = false;
            return;
        }

        if (s_line_len > 0u)
        {
            s_line[s_line_len] = '\0';
            shell_puts("\r\n");
            history_add(s_line);
            shell_execute(s_line);
            s_line_len = 0u;
            s_hist_nav = s_hist_count;
        }
        if (!s_monitor_on)
        {
            shell_prompt();
        }
        return;
    }

    if (c == 0x08u || c == 0x7Fu)
    {
        if (s_line_len > 0u)
        {
            s_line_len--;
            shell_puts("\b \b");
        }
        return;
    }

    if (c >= 0x20u && c < 0x7Fu)
    {
        if (s_line_len < (SHELL_LINE_SIZE - 1u))
        {
            s_line[s_line_len++] = (char)c;
            shell_write((const char *)&c, 1u);
        }
    }
}

void Shell_Task(void *argument)
{
    (void)argument;

    shell_puts("\r\n\r\n" FW_NAME " v" FW_VERSION " ready");
    shell_puts("\r\nType 'help' for commands.");
    shell_prompt();

    for (;;)
    {
        while (s_rx_tail != s_rx_head)
        {
            uint8_t c = s_rx_ring[s_rx_tail];
            s_rx_tail = (uint16_t)((s_rx_tail + 1u) % SHELL_RX_RING_SIZE);
            shell_process_byte(c);
        }

        if (s_monitor_on)
        {
            uint32_t now = osKernelGetTickCount();
            if ((int32_t)(now - s_monitor_next) >= 0)
            {
                monitor_print_line();
                s_monitor_next += 1000u;
            }
        }

        osDelay(10u);
    }
}

void Shell_Init(void)
{
    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_line_len = 0u;
    s_ignore_lf = false;
    s_monitor_on = false;
    s_monitor_next = 0u;
    s_hist_count = 0u;
    s_hist_nav = 0u;
    s_esc_state = 0u;

    huart2.Instance                    = USART2;
    huart2.Init.BaudRate               = 115200u;
    huart2.Init.WordLength             = UART_WORDLENGTH_8B;
    huart2.Init.StopBits               = UART_STOPBITS_1;
    huart2.Init.Parity                 = UART_PARITY_NONE;
    huart2.Init.Mode                   = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }

    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);
}
