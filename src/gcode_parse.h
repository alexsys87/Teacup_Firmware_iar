#ifndef	_GCODE_PARSE_H
#define	_GCODE_PARSE_H

#include <stdint.h>

#include "dda.h"
#include "sd.h"

// whether to insist on N line numbers
// if not defined, N's are completely ignored
//#define	REQUIRE_LINENUMBER

// whether to insist on a checksum
//#define	REQUIRE_CHECKSUM

/// this is a very crude decimal-based floating point structure.
/// a real floating point would at least have signed exponent.\n
/// resulting value is \f$ mantissa * 10^{-(exponent - 1)} * ((sign * 2) - 1)\f$
typedef struct {
	uint32_t	mantissa;		///< the actual digits of our floating point number
	uint8_t	exponent	:7;	///< scale mantissa by \f$10^{-exponent}\f$
	uint8_t	sign			:1; ///< positive or negative?
} decfloat;

/// this holds all the possible data from a received command
typedef struct {
	struct {
		uint8_t					seen_G	:1;
		uint8_t					seen_M	:1;
		uint8_t					seen_X	:1;
		uint8_t					seen_Y	:1;
		uint8_t					seen_Z	:1;
		uint8_t					seen_E	:1;
		uint8_t					seen_F	:1;
		uint8_t					seen_S	:1;
		uint8_t					seen_P	:1;
		uint8_t					seen_T	:1;
		uint8_t					seen_R	:1;
		uint8_t					seen_I	:1;
		uint8_t					seen_J	:1;
		uint8_t					seen_D	:1;
		uint8_t					seen_C	:1;
		uint8_t					seen_U	:1;
		uint8_t					seen_L	:1;
		uint8_t					seen_K	:1;
		uint8_t					seen_N	:1;
		uint8_t					seen_checksum				:1; ///< seen a checksum?
		uint8_t					seen_semi_comment		:1; ///< seen a semicolon?
		uint8_t					seen_parens_comment	:1; ///< seen an open parenthesis
    uint8_t         read_string         :1; ///< Currently reading a string.
		uint8_t					option_all_relative	:1; ///< relative or absolute coordinates?
		uint8_t					option_e_relative		:1; ///< same for e axis (M82/M83)
		uint8_t					option_inches				:1; ///< inches or millimeters?
	};

  uint32_t          N;          ///< line number
  uint32_t          N_expected; ///< expected line number

  int32_t           S;          ///< S word (various uses)
  int32_t           R;          ///< R word (M109/M190 target, M73 minutes,
                                ///< G2/G3 radius in um)
  int32_t           P_milli;    ///< P word in thousandths (M301/M304 Kp)
  int32_t           I_milli;    ///< I word in thousandths (M301/M304 Ki,
                                ///< G2/G3 center offset X)
  int32_t           J_milli;    ///< J word in thousandths (G2/G3 offset Y)
  int32_t           D_milli;    ///< D word in thousandths (M301/M304 Kd)
  uint16_t          C;          ///< C word (M303 cycles)
  uint8_t           U;          ///< U word (M303 apply)
  int32_t           U_milli;    ///< U word in thousandths (M600 unload length)
  int32_t           L_milli;    ///< L word in thousandths (M600 load length)
  int32_t           K_value;    ///< K word in 1/10000 (M900 linear advance)
  int32_t           F_milli;    ///< F word in thousandths for M593 (Hz)
  uint16_t          P;          ///< P word (various uses)

	uint16_t						G;				///< G command number
	uint16_t						M;				///< M command number
	TARGET						target;		///< target position: X, Y, Z, E and F

	uint8_t						T;				///< T word (tool index)
  int32_t           T_value;    ///< T word as a number (M204 travel acceleration)

	uint8_t						checksum_read;				///< checksum in gcode command
	uint8_t						checksum_calculated;	///< checksum we calculated
} GCODE_COMMAND;

/// Bits for gcode_sources / gcode_active.
enum gcode_source {
  GCODE_SOURCE_INIT    = 0x00,
  GCODE_SOURCE_SERIAL  = 0x01,
  GCODE_SOURCE_SD      = 0x02
};

/// Bitfield of enum gcode_source.
extern uint8_t gcode_sources;
extern uint8_t gcode_active;

/// the command being processed
extern GCODE_COMMAND next_target;

#ifdef SD
  /// For storing incoming strings. Currently the only use is SD card filename.
  extern char gcode_str_buf[];
#endif

/// Maximum length of the M117 message, including the terminating zero.
#define GCODE_MSG_LEN 32
/// Text of M117, the rest of the line after "M117 ".
extern char gcode_msg_buf[GCODE_MSG_LEN];

void gcode_init(void);

/// accept the next character and process it
uint8_t gcode_parse_char(uint8_t c);

// uses the global variable next_target.N

#endif	/* _GCODE_PARSE_H */
