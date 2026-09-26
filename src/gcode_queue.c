/** \file
  \brief Command queue and host protocol, Marlin compatible.

  Receiving:
    Characters from the serial receive buffer are assembled into lines,
    comments (';' to end of line) and leading spaces are dropped. A line
    with a line number must look like "N<n> <command>*<checksum>", the
    checksum being the XOR of all characters before '*'. It's accepted if
    the checksum matches and n is the last line number + 1 (M110 sets the
    line number). Lines without line number and checksum are accepted, too.

    Bad lines are answered like Marlin does, hosts (OctoPrint, Pronterface,
    Repetier, Cura) resend automatically:

      Error:checksum mismatch, Last Line: 41
      Resend: 42
      ok

  Executing:
    Up to CMD_BUFSIZE lines wait in the queue. Each executed line gets
    exactly one "ok". With ADVANCED_OK the "ok" tells line number, free
    movement queue slots and free command slots, e.g. "ok N42 P7 B3",
    which lets hosts stream ahead.

  Several host ports (UART, USB CDC):
    Every port has its own line assembly and line numbering, so two hosts
    can't disturb each other's lines. Queued lines remember their port.
    While a line executes, output goes to that port only, so the "ok" and
    all answers reach the host which sent the command. Errors of a line
    (Resend) go to the port the line came from.

  Keepalive:
    While a single command takes longer (G4, G28, waiting for temperatures)
    "echo:busy: processing" is sent every HOST_KEEPALIVE_INTERVAL seconds,
    so hosts don't run into their communication timeout.
*/

#include "gcode_queue.h"

#include <string.h>
#include "config_wrapper.h"
#include "serial.h"
#include "sermsg.h"
#include "gcode_parse.h"
#include "dda_queue.h"
#include "sd.h"

#ifndef CMD_BUFSIZE
  /// Number of queued lines.
  #define CMD_BUFSIZE 8
#endif
#ifndef CMD_MAX_LEN
  /// Maximum line length including line number and checksum (Marlin: 96).
  #define CMD_MAX_LEN 96
#endif
#ifndef HOST_KEEPALIVE_INTERVAL
  #define HOST_KEEPALIVE_INTERVAL 2
#endif

uint8_t gcode_ok_sent = 0;

static char cmd_buf[CMD_BUFSIZE][CMD_MAX_LEN];
static int32_t cmd_line_no[CMD_BUFSIZE];    ///< Line number, -1 = none.
static uint8_t cmd_port[CMD_BUFSIZE];       ///< Host port of the line.
static uint8_t cmd_head, cmd_tail, cmd_count;

/// Receive state of one host port.
typedef struct {
  char line[CMD_MAX_LEN];                    ///< Line being received.
  int32_t last_n;                            ///< Last accepted line number.
  uint8_t len;
  uint8_t comment;                           ///< Inside a ';' comment.
  uint8_t overflow;                          ///< Line was too long.
} port_rx_t;

static port_rx_t port_rx[SERIAL_NUM_PORTS];

static uint8_t paused = 0;                  ///< M600 waits for the user.
static uint8_t executing = 0;
static uint8_t keepalive_interval = HOST_KEEPALIVE_INTERVAL;
static uint8_t keepalive_count = 0;

/// Parse a decimal integer, advances *p.
static int32_t parse_int(const char **p) {
  const char *s = *p;
  int32_t v = 0;
  uint8_t neg = 0;

  if (*s == '-') {
    neg = 1;
    s++;
  }
  while (*s >= '0' && *s <= '9')
    v = v * 10 + (*s++ - '0');
  *p = s;
  return neg ? -v : v;
}

/**
  Reject the current line of a port and ask the host to send it again.
  Output goes to the current selection, which is that port.
*/
static void line_error(uint8_t port, const char *msg) {
  port_rx_t *r = &port_rx[port];

  serial_writestr("Error:");
  serial_writestr(msg);
  serial_writestr(", Last Line: ");
  serwrite_int32(r->last_n);
  serial_writechar('\n');

  // Drop whatever follows, the host resends everything from last_n + 1.
  while (serial_port_rxchars(port))
    (void)serial_port_popchar(port);
  r->len = 0;
  r->comment = 0;
  r->overflow = 0;

  serial_writestr("Resend: ");
  serwrite_int32(r->last_n + 1);
  serial_writestr("\nok\n");
}

/// Check a complete line of a port and queue it. Output goes to the port.
static void finish_line(uint8_t port) {
  port_rx_t *r = &port_rx[port];
  char *line = r->line;
  const char *p = line;
  char *star;
  int32_t n = -1;

  // Trailing spaces.
  while (r->len && (line[r->len - 1] == ' ' || line[r->len - 1] == '\t'))
    r->len--;
  line[r->len] = '\0';

  star = strchr(line, '*');

  if (line[0] == 'N' || line[0] == 'n') {
    uint8_t checksum = 0;
    const char *c;
    const char *cs;
    uint8_t is_m110;

    if (r->overflow) {
      line_error(port, "Line too long");
      return;
    }
    if (star == NULL) {
      line_error(port, "No Checksum with line number");
      return;
    }
    for (c = line; c < star; c++)
      checksum ^= (uint8_t)*c;
    cs = star + 1;
    if (*cs < '0' || *cs > '9' || parse_int(&cs) != checksum) {
      line_error(port, "checksum mismatch");
      return;
    }
    *star = '\0';

    p = line + 1;
    n = parse_int(&p);
    while (*p == ' ' || *p == '\t')
      p++;

    is_m110 = (p[0] == 'M' || p[0] == 'm') && p[1] == '1' && p[2] == '1' &&
              p[3] == '0' && (p[4] < '0' || p[4] > '9');

    if (is_m110) {
      // M110 [N<n>]: set the line number, the parameter wins.
      const char *q = p + 4;

      r->last_n = n;
      while (*q) {
        if (*q == 'N' || *q == 'n') {
          q++;
          r->last_n = parse_int(&q);
          break;
        }
        q++;
      }
    }
    else if (n != r->last_n + 1) {
      line_error(port, "Line Number is not Last Line Number+1");
      return;
    }
    else {
      r->last_n = n;
    }
  }
  else if (star != NULL) {
    line_error(port, "No Line Number with checksum");
    return;
  }
  else if (r->overflow) {
    serial_writestr("echo:Line too long, ignored\nok\n");
    return;
  }

  // Empty after removing the line number? Nothing to execute, but the host
  // expects an acknowledgement.
  if (*p == '\0') {
    serial_writestr("ok\n");
    return;
  }

  {
    // Bounded copy, p is shorter than CMD_MAX_LEN (it's part of line[]).
    uint8_t k = 0;

    while (p[k] && k < CMD_MAX_LEN - 1) {
      cmd_buf[cmd_head][k] = p[k];
      k++;
    }
    cmd_buf[cmd_head][k] = '\0';
  }
  cmd_line_no[cmd_head] = n;
  cmd_port[cmd_head] = port;
  cmd_head = (cmd_head + 1) % CMD_BUFSIZE;
  cmd_count++;
}

/// Read one port, until its buffer is empty or the queue is full.
static void read_port(uint8_t port) {
  port_rx_t *r = &port_rx[port];
  uint8_t out_saved = 0xFF;                  ///< 0xFF: output not switched.

  while (cmd_count < CMD_BUFSIZE && serial_port_rxchars(port)) {
    char c = (char)serial_port_popchar(port);

    if (c == '\n' || c == '\r') {
      if (r->len || r->overflow) {
        // Answers of finish_line() (errors, "ok" of empty lines) go to
        // this port, even while a command of another port executes.
        if (out_saved == 0xFF)
          out_saved = serial_set_output(SERIAL_MASK(port));
        finish_line(port);
      }
      r->len = 0;
      r->comment = 0;
      r->overflow = 0;
      continue;
    }

    if (r->comment)
      continue;
    if (c == ';') {
      r->comment = 1;
      continue;
    }
    if (r->len == 0 && (c == ' ' || c == '\t'))
      continue;

    if (r->len < CMD_MAX_LEN - 1)
      r->line[r->len++] = c;
    else
      r->overflow = 1;
  }

  if (out_saved != 0xFF)
    serial_set_output(out_saved);
}

void gcode_queue_read_serial(void) {
  uint8_t port;

  for (port = 0; port < SERIAL_NUM_PORTS; port++)
    read_port(port);
}

uint8_t gcode_queue_free(void) {
  return CMD_BUFSIZE - cmd_count;
}

uint8_t gcode_queue_execute(void) {
  const char *s;
  int32_t n;
  uint8_t out_saved;

  if (cmd_count == 0)
    return 0;

  s = cmd_buf[cmd_tail];
  n = cmd_line_no[cmd_tail];

  // All output of this command, including "ok", to the host which sent it.
  out_saved = serial_set_output(SERIAL_MASK(cmd_port[cmd_tail]));

  executing = 1;
  keepalive_count = 0;
  gcode_ok_sent = 0;
  gcode_active = GCODE_SOURCE_SERIAL;

  #ifdef SD_FLASH
    // Upload (M28): store lines instead of executing them, up to M29.
    if (sd_writing() && ! ((s[0] == 'M' || s[0] == 'm') && s[1] == '2' &&
                           s[2] == '9' && (s[3] < '0' || s[3] > '9'))) {
      sd_write_line(s);
      s = "";
    }
  #endif

  if (*s) {
    while (*s)
      gcode_parse_char((uint8_t)*s++);
    gcode_parse_char('\n');
  }

  gcode_active = GCODE_SOURCE_INIT;
  executing = 0;

  // Free the slot only now: gcode_queue_read_serial() may run while the
  // command executes (from clock_poll()) and must not overwrite it.
  cmd_tail = (cmd_tail + 1) % CMD_BUFSIZE;
  cmd_count--;

  if ( ! gcode_ok_sent) {
    serial_writestr("ok");
    #ifdef ADVANCED_OK
      if (n >= 0) {
        serial_writestr(" N");
        serwrite_int32(n);
      }
      serial_writestr(" P");
      serwrite_uint8(queue_free());
      serial_writestr(" B");
      serwrite_uint8(gcode_queue_free());
    #else
      (void)n;
    #endif
    serial_writechar('\n');
  }
  gcode_ok_sent = 0;
  serial_set_output(out_saved);

  return 1;
}

void gcode_queue_keepalive(void) {
  if ( ! (executing || paused) || keepalive_interval == 0)
    return;

  if (++keepalive_count >= keepalive_interval) {
    keepalive_count = 0;
    serial_writestr(paused ? "echo:busy: paused for user\n"
                           : "echo:busy: processing\n");
  }
}

void gcode_queue_set_paused(uint8_t on) {
  paused = on;
  keepalive_count = 0;
}

void gcode_queue_set_keepalive(uint8_t seconds) {
  keepalive_interval = seconds;
}

uint8_t gcode_queue_get_keepalive(void) {
  return keepalive_interval;
}
