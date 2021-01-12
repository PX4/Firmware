/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file tunes.h
 */

#pragma once

#include <errno.h>
#include <uORB/uORB.h>
#include <uORB/topics/tune_control.h>
#include "tune_definition.h"

#define TUNE_DEFAULT_NOTE_LENGTH 4
#define TUNE_DEFAULT_OCTAVE 4
#define TUNE_DEFAULT_TEMPO 120

#define TUNE_MAX_UPDATE_INTERVAL_US 100000


/**
 *  Library for parsing tunes from melody-strings or dedicated tune messages.
 *  Needs to be instantiated as it keeps track of which tune is to be played
 *  next. Also handles repeated tunes.
 */
class Tunes
{
public:
	enum class NoteMode {NORMAL, LEGATO, STACCATO};

	enum class Status {
		Continue = 1,
		Stop = 0,
		Error = -1,
	};

	enum class ControlResult {
		Success = 0,
		AlreadyPlaying = 1,
		InvalidTune = -EINVAL,
		WouldInterrupt = -EBUSY,
	};

	/**
	 * Constructor with the default parameters set to:
	 * default_tempo: TUNE_DEFAULT_TEMPO
	 * default_octave: TUNE_DEFAULT_OCTAVE
	 * default_note_length: TUNE_DEFAULT_NOTE_LENGTH
	 * default_mode: NORMAL
	 */
	Tunes(unsigned default_note_length = TUNE_DEFAULT_NOTE_LENGTH,
	      NoteMode default_note_mode   = NoteMode::NORMAL,
	      unsigned default_octave      = TUNE_DEFAULT_OCTAVE,
	      unsigned default_tempo       = TUNE_DEFAULT_TEMPO);

	~Tunes() = default;

	/**
	 * Set tune to be played using the message. If a tune is already being played
	 * the call to this function will be ignored, unless the override flag is set
	 * or the tune being already played is a repeated tune.
	 * @param  tune_control struct containig the uORB message
	 * @return              return ControlResult::InvalidTune if the default tune does not exist,
	 * 			ControlResult::WouldInterrupt if tune was already playing and not interruptable,
	 * 			ControlResult::AlreadyPlaying if same tune was already playing,
	 * 			ControlResult::Success if new tune was set.
	 */
	ControlResult set_control(const tune_control_s &tune_control);

	/**
	 * Set tune to be played using a string.
	 * Parses a tune string, formatted with the syntax of the Microsoft GWBasic/QBasic.
	 * Ownership of the string is NOT transferred. The string has to be kept in
	 * memory for the whole duration of the melody.
	 *
	 * @param  string    tune input string
	 */
	void set_string(const char *const string, uint8_t volume);

	/**
	 * Get next note in the current tune, which has been provided by either
	 * set_control or play_string
	 * @param  frequency return frequency value (Hz)
	 * @param  duration  return duration of the tone (us)
	 * @param  silence   return silence duration (us)
	 * @return           -1 for error, 0 for play one tone and 1 for continue a sequence
	 */
	Tunes::Status get_next_note(unsigned &frequency, unsigned &duration, unsigned &silence);

	/**
	 * Get next note in the current tune, which has been provided by either
	 * set_control or play_string
	 * @param  frequency return frequency value (Hz)
	 * @param  duration  return duration of the note (us)
	 * @param  silence   return silence duration (us)
	 * @param  volume    return the volume level of the note (between 0-100)
	 * @return           -1 for no tune available/error, 0 to not play anything and 1 to play
	 */
	Tunes::Status get_next_note(unsigned &frequency, unsigned &duration,
				    unsigned &silence, uint8_t &volume);

	/**
	 *  Get the number of default tunes. This is useful for when a tune is
	 *  requested via its tune ID.
	 *  @return             Number of default tunes accessible via tune ID
	 */
	unsigned int get_default_tunes_size() const {return _default_tunes_size;}

	unsigned int get_maximum_update_interval() {return (unsigned int)TUNE_MAX_UPDATE_INTERVAL_US;}

private:

	/**
	 * Convert note to frequency
	 *
	 * @param  note unsigned value of the semitone from C
	 * @return      frequency (Hz)
	 */
	uint32_t note_to_frequency(unsigned note) const;

	/**
	 * Calculate the duration in microseconds of play and silence for a
	 * note given the current tempo, length and mode and the number of
	 * dots following in the play string.
	 *
	 * @param  silence     return silence duration (us)
	 * @param  note_length note length
	 * @param  dots        extention of the note length
	 * @return             duration of the note (us)
	 */
	unsigned note_duration(unsigned &silence, unsigned note_length, unsigned dots) const;

	/**
	 * Calculate the duration in microseconds of a rest corresponding to
	 * a given note length.
	 *
	 * @param  rest_length rest lenght in fraction of a note
	 * @param  dots        number of extension dots
	 * @return             rest duration (us)
	 */
	unsigned rest_duration(unsigned rest_length, unsigned dots) const;

	/**
	 * Find the next character in the string, discard any whitespace.
	 *
	 * @return uppercase version of the char.
	 */
	int next_char();

	/**
	 * Extract a number from the string, consuming all the digit characters.
	 *
	 * @return extracted number.
	 */
	unsigned next_number();

	/**
	 * Consume dot characters from the string
	 *
	 * @return number of consumed dots
	 */
	unsigned next_dots();

	/**
	 * Reset the tune parameters. This is necessary when for example a tune moved
	 * one or more octaves up or down. reset() should always be called before
	 * (re)-starting a tune.
	 */
	void reset(bool repeat_flag);

	Tunes::Status tune_end();

	Tunes::Status tune_error();

	static const char *const  _default_tunes[];
	static const bool         _default_tunes_interruptable[];
	static const unsigned int _default_tunes_size;
	static const uint8_t      _note_tab[];

	const char *_next_tune      = nullptr; ///< next note in the string
	const char *_tune           = nullptr; ///< current tune string
	const char *_tune_start_ptr = nullptr; ///< pointer to repeat tune

	int _current_tune_id = static_cast<int>(TuneID::NONE);

	bool _repeat = false; ///< if true, tune restarts at end

	unsigned int _default_note_length = TUNE_DEFAULT_NOTE_LENGTH;
	NoteMode     _default_note_mode   = NoteMode::NORMAL;
	unsigned int _default_octave      = TUNE_DEFAULT_OCTAVE;
	unsigned int _default_tempo       = TUNE_DEFAULT_TEMPO;

	unsigned int _note_length         = TUNE_DEFAULT_NOTE_LENGTH;
	NoteMode     _note_mode           = NoteMode::NORMAL;
	unsigned int _octave              = TUNE_DEFAULT_OCTAVE;
	unsigned int _tempo               = TUNE_DEFAULT_TEMPO;

	unsigned int _duration  = 0;
	unsigned int _frequency = 0;
	unsigned int _silence   = 0;
	uint8_t      _volume    = 0;

	bool  _using_custom_msg = false;
};
