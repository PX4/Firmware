/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

class MPU9250;

#pragma pack(push, 1)
struct ak8963_regs {
//	uint8_t		cmd;
	uint8_t st1;
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t st2;
};
#pragma pack(pop)

/**
 * Helper class implementing the magnetometer driver node.
 */
class MPU9250_mag : public device::CDev
{
public:
	MPU9250_mag(MPU9250 *parent, const char *path);
	~MPU9250_mag();

	virtual int init();

	void set_passthrough(uint8_t reg, uint8_t size, uint8_t *out = NULL);
	void passthrough_read(uint8_t reg, uint8_t *buf, uint8_t size);
	void passthrough_write(uint8_t reg, uint8_t val);
	void read_block(uint8_t reg, uint8_t *val, uint8_t count);

	void ak8963_reset(void);
	bool ak8963_setup(void);
	bool ak8963_check_id(void);

protected:
	friend class MPU9250;

	bool measure(struct ak8963_regs data, struct Report &report);

private:
	MPU9250 *_parent;

	bool check_duplicate(uint8_t *mag_data);

	// keep last accel reading for duplicate detection
	uint8_t			_last_mag_data[6];

	/* do not allow to copy this class due to pointer data members */
	MPU9250_mag(const MPU9250_mag &);
	MPU9250_mag operator=(const MPU9250_mag &);
};
