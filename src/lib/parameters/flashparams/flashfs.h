/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file param.h
 *
 * Global flash based parameter store.
 *
 * This provides the mechanisms to interface to the PX4
 * parameter system but replace the IO with non file based flash
 * i/o routines. So that the code my be implemented on a SMALL memory
 * foot print device.
 *
 */

#ifndef _SYSTEMLIB_FLASHPARAMS_NUTTX_PARAM_H
#define _SYSTEMLIB_FLASHPARAMS_NUTTX_PARAM_H

#include <stdint.h>
#include <stdbool.h>



/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/*
 *  PARAMETER_BUFFER_SIZE must be defined larger then the maximum parameter
 * memory needed to commit the recored + ~20 bytes. For the syslib's parameter
 * this would be the size of the bson representations of the data
 */
#if !defined(PARAMETER_BUFFER_SIZE)
#define PARAMETER_BUFFER_SIZE 512
#endif

__BEGIN_DECLS

/*
 *  Define the interface data a flash_file_token_t
 *  is like a file name
 *
 */
typedef uint32_t flash_file_tokens_t;

typedef struct flash_file_token_t {
	union {
		flash_file_tokens_t t;
		uint8_t n[sizeof(flash_file_tokens_t)];
	};
} flash_file_token_t;

/*
 * Define the parameter "file name" Currently there is only
 * and it is hard coded. If more are added the
 * parameter_flashfs_write would need to support a backing buffer
 * for when a sector is erased.
 */
__EXPORT extern const flash_file_token_t parameters_token;

/* Define the elements of the array passed to the
 * parameter_flashfs_init function
 *
 * For example
 * static sector_descriptor_t  sector_map[] = {
 *      {1, 16 * 1024, 0x08004000},
 *      {2, 16 * 1024, 0x08008000},
 *       {0, 0, 0},
 *
 */
typedef struct sector_descriptor_t {
	uint16_t      page;
	uint32_t      size;
	uint32_t      address;
} sector_descriptor_t;


/****************************************************************************
 * Name: parameter_flashfs_init
 *
 * Description:
 *   This helper function advances the flash entry header pointer to the
 *   locations of the next entry.
 *
 * Input Parameters:
 *   fconfig      - A pointer to an null entry terminated array of
 *                  flash_file_sector_t
 *    buffer      - A pointer to a memory to make available to callers
 *                  for write operations. When allocated to the caller
 *                  space is reserved in the front for the
 *                  flash_entry_header_t.
 *                  If this is passes as NULL. The buffer will be
 *                  allocated from the heap on calls to
 *                  parameter_flashfs_alloc and fread on calls
 *                  to parameter_flashfs_free
 *
 *   size         - The size of the buffer in bytes. Should be be 0 if buffer
 *                  is NULL
 *
 * Returned value:
 *                - A pointer to the next file header location
 *
 *
 ****************************************************************************/

__EXPORT int parameter_flashfs_init(sector_descriptor_t *fconfig, uint8_t *buffer, uint16_t size);

/****************************************************************************
 * Name: parameter_flashfs_read
 *
 * Description:
 *   This function returns a pointer to the locations of the data associated
 *   with the file token. On successful return *buffer will be set to Flash
 *   location and *buf_size the length of the user data.
 *
 * Input Parameters:
 *   token       - File Token File to read
 *   buffer      - A pointer to a pointer that will receive the address
 *                 in flash of the data of this "files" data
 *   buf_size    - A pointer to receive the number of bytes in the "file"
 *
 * Returned value:
 *   On success number of bytes read or a negative errno value,
 *
 *
 ****************************************************************************/

__EXPORT int parameter_flashfs_read(flash_file_token_t ft, uint8_t **buffer, size_t *buf_size);

/****************************************************************************
 * Name: parameter_flashfs_write
 *
 * Description:
 *   This function writes user data from the buffer allocated with a previous call
 *   to parameter_flashfs_alloc. flash starting at the given address
 *
 * Input Parameters:
 *   token      - File Token File to read
 *   buffer      - A pointer to a buffer with buf_size bytes to be written
 *                 to the flash. This buffer must be allocated
 *                 with a previous call to parameter_flashfs_alloc
 *   buf_size    - Number of bytes to write
 *
 * Returned value:
 *   On success the number of bytes written On Error a negative value of errno
 *
 ****************************************************************************/

__EXPORT int parameter_flashfs_write(flash_file_token_t ft, uint8_t *buffer, size_t buf_size);

/****************************************************************************
 * Name: parameter_flashfs_erase
 *
 * Description:
 *   This function erases the sectors that were passed to parameter_flashfs_init
 *
 * Input Parameters:
 *
 * Returned value:
 *   On success the number of bytes erased
 *   On Error a negative value of errno
 *
 ****************************************************************************/

__EXPORT int parameter_flashfs_erase(void);

/****************************************************************************
 * Name: parameter_flashfs_alloc
 *
 * Description:
 *   This function is called to get a buffer to use in a subsequent call
 *   to parameter_flashfs_write. The address returned is advanced into the
 *   buffer to reserve space for the flash entry header.
 *
 * Input Parameters:
 *   token      - File Token File to read (not used)
 *   buffer     - Memory of buf_size length suitable for calling
 *                parameter_flashfs_write
 *   buf_size   - The maximum number of bytes that can be written to
 *                the buffer
 *
 * Returned value:
 *   On success the number of bytes written On Error a negative value of errno
 *
 ****************************************************************************/

__EXPORT int parameter_flashfs_alloc(flash_file_token_t ft, uint8_t **buffer, size_t *buf_size);


/****************************************************************************
 * Name: parameter_flashfs_free
 *
 * Description:
 *   Frees  dynamically allocated memory
 *
 *
 ****************************************************************************/

__EXPORT void parameter_flashfs_free(void);

__END_DECLS
#endif /* _SYSTEMLIB_FLASHPARAMS_NUTTX_PARAM_H */
