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
 * @file px4_module.h
 */

#pragma once

#include <pthread.h>
#include <unistd.h>
#include <stdbool.h>

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <systemlib/px4_macros.h>

#ifdef __cplusplus

#include <cstring>

/**
 * @brief This mutex protects against race conditions during startup & shutdown of modules.
 *        There could be one mutex per module instantiation, but to reduce the memory footprint
 *        there is only a single global mutex. This sounds bad, but we actually don't expect
 *        contention here, as module startup is sequential.
 */
extern pthread_mutex_t px4_modules_mutex;

/**
 * @class ModuleBase
 *      Base class for modules, implementing common functionality,
 *      such as 'start', 'stop' and 'status' commands.
 *      Currently does not support modules which allow multiple instances,
 *      such as mavlink.
 *
 *      The class is implemented as curiously recurring template pattern (CRTP).
 *      It allows to have a static object in the base class that is different for
 *      each module type, and call static methods from the base class.
 *
 * @note Required methods for a derived class:
 * When running in its own thread:
 *      static int task_spawn(int argc, char *argv[])
 *      {
 *              // call px4_task_spawn_cmd() with &run_trampoline as startup method
 *              // optional: wait until _object is not null, which means the task got initialized (use a timeout)
 *              // set _task_id and return 0
 *              // on error return != 0 (and _task_id must be -1)
 *      }
 *
 *      static T *instantiate(int argc, char *argv[])
 *      {
 *              // this is called from within the new thread, from run_trampoline()
 *              // parse the arguments
 *              // create a new object T & return it
 *              // or return nullptr on error
 *      }
 *
 *      static int custom_command(int argc, char *argv[])
 *      {
 *              // support for custom commands
 *              // it none are supported, just do:
 *              return print_usage("unrecognized command");
 *      }
 *
 *      static int print_usage(const char *reason = nullptr)
 *      {
 *              // use the PRINT_MODULE_* methods...
 *      }
 *
 * When running on the work queue:
 * - custom_command & print_usage as above
 *      static int task_spawn(int argc, char *argv[]) {
 *              // parse the arguments
 *              // set _object (here or from the work_queue() callback)
 *              // call work_queue() (with a custom cycle trampoline)
 *              // optional: wait until _object is not null, which means the task got initialized (use a timeout)
 *              // set _task_id to task_id_is_work_queue and return 0
 *              // on error return != 0 (and _task_id must be -1)
 *      }
 */
template<class T>
class ModuleBase
{
public:
	ModuleBase() : _task_should_exit{false} {}
	virtual ~ModuleBase() {}

	/**
	 * @brief main Main entry point to the module that should be
	 *        called directly from the module's main method.
	 * @param argc The task argument count.
	 * @param argc Pointer to the task argument variable array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int main(int argc, char *argv[])
	{
		if (argc <= 1 ||
		    strcmp(argv[1], "-h")    == 0 ||
		    strcmp(argv[1], "help")  == 0 ||
		    strcmp(argv[1], "info")  == 0 ||
		    strcmp(argv[1], "usage") == 0) {
			return T::print_usage();
		}

		if (strcmp(argv[1], "start") == 0) {
			// Pass the 'start' argument too, because later on px4_getopt() will ignore the first argument.
			return start_command_base(argc - 1, argv + 1);
		}

		if (strcmp(argv[1], "status") == 0) {
			return status_command();
		}

		if (strcmp(argv[1], "stop") == 0) {
			return stop_command();
		}

		lock_module(); // Lock here, as the method could access _object.
		int ret = T::custom_command(argc - 1, argv + 1);
		unlock_module();

		return ret;
	}

	/**
	 * @brief Entry point for px4_task_spawn_cmd() if the module runs in its own thread.
	 *        It does:
	 *        - instantiate the object
	 *        - call run() on it to execute the main loop
	 *        - cleanup: delete the object
	 * @param argc The task argument count.
	 * @param argc Pointer to the task argument variable array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int run_trampoline(int argc, char *argv[])
	{
		int ret = 0;

		// We don't need the task name at this point.
		argc -= 1;
		argv += 1;

		T *object = T::instantiate(argc, argv);
		_object.store(object);

		if (object) {
			object->run();

		} else {
			PX4_ERR("failed to instantiate object");
			ret = -1;
		}

		exit_and_cleanup();

		return ret;
	}

	/**
	 * @brief Stars the command, ('command start'), checks if if is already
	 *        running and calls T::task_spawn() if it's not.
	 * @param argc The task argument count.
	 * @param argc Pointer to the task argument variable array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int start_command_base(int argc, char *argv[])
	{
		int ret = 0;
		lock_module();

		if (is_running()) {
			ret = -1;
			PX4_ERR("Task already running");

		} else {
			ret = T::task_spawn(argc, argv);

			if (ret < 0) {
				PX4_ERR("Task start failed (%i)", ret);
			}
		}

		unlock_module();
		return ret;
	}

	/**
	 * @brief Stops the command, ('command stop'), checks if it is running and if it is, request the module to stop
	 *        and waits for the task to complete.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int stop_command()
	{
		int ret = 0;
		lock_module();

		if (is_running()) {
			T *object = _object.load();

			if (object) {
				object->request_stop();

				unsigned int i = 0;

				do {
					unlock_module();
					px4_usleep(10000); // 10 ms
					lock_module();

					if (++i > 500 && _task_id != -1) { // wait at most 5 sec
						PX4_ERR("timeout, forcing stop");

						if (_task_id != task_id_is_work_queue) {
							px4_task_delete(_task_id);
						}

						_task_id = -1;

						delete _object.load();
						_object.store(nullptr);

						ret = -1;
						break;
					}
				} while (_task_id != -1);

			} else {
				// In the very unlikely event that can only happen on work queues,
				// if the starting thread does not wait for the work queue to initialize,
				// and inside the work queue, the allocation of _object fails
				// and exit_and_cleanup() is not called, set the _task_id as invalid.
				_task_id = -1;
			}
		}

		unlock_module();
		return ret;
	}

	/**
	 * @brief Handle 'command status': check if running and call print_status() if it is
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int status_command()
	{
		int ret = -1;
		lock_module();

		if (is_running() && _object.load()) {
			T *object = _object.load();
			ret = object->print_status();

		} else {
			PX4_INFO("not running");
		}

		unlock_module();
		return ret;
	}

	/**
	 * @brief Print the status if the module is running. This can be overridden by the module to provide
	 * more specific information.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	virtual int print_status()
	{
		PX4_INFO("running");
		return 0;
	}

	/**
	 * @brief Main loop method for modules running in their own thread. Called from run_trampoline().
	 *        This method must return when should_exit() returns true.
	 */
	virtual void run()
	{
	}

	/**
	 * @brief Returns the status of the module.
	 * @return Returns true if the module is running, false otherwise.
	 */
	static bool is_running()
	{
		return _task_id != -1;
	}

protected:

	/**
	 * @brief Tells the module to stop (used from outside or inside the module thread).
	 */
	virtual void request_stop()
	{
		_task_should_exit.store(true);
	}

	/**
	 * @brief Checks if the module should stop (used within the module thread).
	 * @return Returns True iff successful, false otherwise.
	 */
	bool should_exit() const
	{
		return _task_should_exit.load();
	}

	/**
	 * @brief Exits the module and delete the object. Called from within the module's thread.
	 *        For work queue modules, this needs to be called from the derived class in the
	 *        cycle method, when should_exit() returns true.
	 */
	static void exit_and_cleanup()
	{
		// Take the lock here:
		// - if startup fails and we're faster than the parent thread, it will set
		//   _task_id and subsequently it will look like the task is running.
		// - deleting the object must take place inside the lock.
		lock_module();

		delete _object.load();
		_object.store(nullptr);

		_task_id = -1; // Signal a potentially waiting thread for the module to exit that it can continue.
		unlock_module();
	}

	/**
	 * @brief Waits until _object is initialized, (from the new thread). This can be called from task_spawn().
	 * @return Returns 0 iff successful, -1 on timeout or otherwise.
	 */
	static int wait_until_running()
	{
		int i = 0;

		do {
			/* Wait up to 1s. */
			px4_usleep(2500);

		} while (!_object.load() && ++i < 400);

		if (i == 400) {
			PX4_ERR("Timed out while waiting for thread to start");
			return -1;
		}

		return 0;
	}

	/**
	 * @brief Get the module's object instance, (this is null if it's not running).
	 */
	static T *get_instance()
	{
		return (T *)_object.load();
	}

	/**
	 * @var _object Instance if the module is running.
	 * @note There will be one instance for each template type.
	 */
	static px4::atomic<T *> _object;

	/** @var _task_id The task handle: -1 = invalid, otherwise task is assumed to be running. */
	static int _task_id;

	/** @var task_id_is_work_queue Value to indicate if the task runs on the work queue. */
	static constexpr const int task_id_is_work_queue = -2;

private:
	/**
	 * @brief lock_module Mutex to lock the module thread.
	 */
	static void lock_module()
	{
		pthread_mutex_lock(&px4_modules_mutex);
	}

	/**
	 * @brief unlock_module Mutex to unlock the module thread.
	 */
	static void unlock_module()
	{
		pthread_mutex_unlock(&px4_modules_mutex);
	}

	/** @var _task_should_exit Boolean flag to indicate if the task should exit. */
	px4::atomic_bool _task_should_exit{false};
};

template<class T>
px4::atomic<T *> ModuleBase<T>::_object{nullptr};

template<class T>
int ModuleBase<T>::_task_id = -1;


#endif /* __cplusplus */


__BEGIN_DECLS

/**
 * @brief Module documentation and command usage help methods.
 *        These are extracted with the Tools/px_process_module_doc.py
 *        script and must be kept in sync.
 */

#ifdef __PX4_NUTTX
/**
 * @note Disable module description on NuttX to reduce Flash usage.
 *       There's a GCC bug (https://gcc.gnu.org/bugzilla/show_bug.cgi?id=55971), preventing us to use
 *       a macro, but GCC will remove the string as well with this empty inline method.
 * @param description The provided functionality of the module and potentially the most important parameters.
 */
static inline void PRINT_MODULE_DESCRIPTION(const char *description) {}
#else

/**
 * @brief Prints module documentation (will also be used for online documentation). It uses Markdown syntax
 *        and should include these sections:
 * - ### Description
 *   Provided functionality of the module and potentially the most important parameters.
 * - ### Implementation
 *   High-level implementation overview
 * - ### Examples
 *   Examples how to use the CLI interface (if it's non-trivial)
 *
 * In addition to the Markdown syntax, a line beginning with '$ ' can be used to mark a command:
 * $ module start -p param
 */
__EXPORT void PRINT_MODULE_DESCRIPTION(const char *description);
#endif

/**
 * @brief Prints the command name.
 * @param executable_name: command name used in scripts & CLI
 * @param category one of: driver, estimator, controller, system, communication, command, template
 */
__EXPORT void PRINT_MODULE_USAGE_NAME(const char *executable_name, const char *category);

/**
 * @brief Specify a subcategory (optional).
 * @param subcategory e.g. if the category is 'driver', subcategory can be 'distance_sensor'
 */
__EXPORT void PRINT_MODULE_USAGE_SUBCATEGORY(const char *subcategory);

/**
 * @brief Prints the name for a command without any sub-commands (@see PRINT_MODULE_USAGE_NAME()).
 */
__EXPORT void PRINT_MODULE_USAGE_NAME_SIMPLE(const char *executable_name, const char *category);


/**
 * @brief Prints a command with a short description what it does.
 */
__EXPORT void PRINT_MODULE_USAGE_COMMAND_DESCR(const char *name, const char *description);

#define PRINT_MODULE_USAGE_COMMAND(name) \
	PRINT_MODULE_USAGE_COMMAND_DESCR(name, NULL);

/**
 * @brief Prints the default commands: stop & status.
 */
#define PRINT_MODULE_USAGE_DEFAULT_COMMANDS() \
	PRINT_MODULE_USAGE_COMMAND("stop"); \
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "print status info");

/**
 * Print default params for I2C or SPI drivers
 * @param i2c_support true if the driver supports I2C
 * @param spi_support true if the driver supports SPI
 */
__EXPORT void PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(bool i2c_support, bool spi_support);

/**
 * Configurable I2C address (via -a <address>)
 */
__EXPORT void PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(uint8_t default_address);

/**
 * -k flag
 */
__EXPORT void PRINT_MODULE_USAGE_PARAMS_I2C_KEEP_RUNNING_FLAG(void);

/** @note Each of the PRINT_MODULE_USAGE_PARAM_* methods apply to the previous PRINT_MODULE_USAGE_COMMAND_DESCR(). */

/**
 * @brief Prints an integer parameter.
 * @param option_char The option character.
 * @param default_val The parameter default value (set to -1 if not applicable).
 * @param min_val The parameter minimum value.
 * @param max_val The parameter value.
 * @param description Pointer to the usage description.
 * @param is_optional true if this parameter is optional
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_INT(char option_char, int default_val, int min_val, int max_val,
		const char *description, bool is_optional);

/**
 * @brief Prints a float parameter.
 * @note See PRINT_MODULE_USAGE_PARAM_INT().
 * @param default_val The parameter default value (set to NaN if not applicable).
 * @param min_val The parameter minimum value.
 * @param max_val The parameter maximum value.
 * @param description Pointer to the usage description. Pointer to the usage description.
 * @param is_optional true if this parameter is optional
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_FLOAT(char option_char, float default_val, float min_val, float max_val,
		const char *description, bool is_optional);

/**
 * @brief Prints a flag parameter, without any value.
 * @note See PRINT_MODULE_USAGE_PARAM_INT().
 * @param option_char The option character.
 * @param description Pointer to the usage description.
 * @param is_optional true if this parameter is optional
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_FLAG(char option_char, const char *description, bool is_optional);

/**
 * @brief Prints a string parameter.
 * @param option_char The option character.
 * @param default_val The default value, can be nullptr.
 * @param values The valid values, it has one of the following forms:
 *               - nullptr: leave unspecified, or any value is valid
 *               - "<file>" or "<file:dev>": a file or more specifically a device file (eg. serial device)
 *               - "<topic_name>": uORB topic name
 *               - "<value1> [<value2>]": a list of values
 *               - "on|off": a concrete set of valid strings separated by "|".
 * @param description Pointer to the usage description.
 * @param is_optional True iff this parameter is optional.
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_STRING(char option_char, const char *default_val, const char *values,
		const char *description, bool is_optional);

/**
 * @brief Prints a comment, that applies to the next arguments or parameters. For example to indicate that
 *        a parameter applies to several or all commands.
 * @param comment
 */
__EXPORT void PRINT_MODULE_USAGE_PARAM_COMMENT(const char *comment);


/**
 * @brief Prints the definition for an argument, which does not have the typical -p <val> form,
 *        but for example 'param set <param> <value>'
 * @param values eg. "<file>", "<param> <value>" or "<value1> [<value2>]"
 * @param description Pointer to the usage description.
 * @param is_optional true if this parameter is optional
 */
__EXPORT void PRINT_MODULE_USAGE_ARG(const char *values, const char *description, bool is_optional);


__END_DECLS
