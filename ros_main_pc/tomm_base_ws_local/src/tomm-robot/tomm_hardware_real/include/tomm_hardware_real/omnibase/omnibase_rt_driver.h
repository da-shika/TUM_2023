#ifndef OMNIBASE_RT_DRIVER_H_
#define OMNIBASE_RT_DRIVER_H_

#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

// use boost instead of c++11
#include <boost/thread.hpp>
#include <boost/array.hpp>

#include <tomm_hardware_real/utilities/rt_thread.h>

#define NUMBER_OF_SLAVES 4

namespace omnibase_rt_driver
{

	/*
	 * Holds command registers
	 * Requires PDO config 0x1601 ( see page 20)
	 */
	struct VelocityWrite
	{
		int32_t velocity; // 32
		uint16_t status;	// 16 -> 48 bit ouput reg size
	};
	typedef boost::array<VelocityWrite, NUMBER_OF_SLAVES> VelocityVec;

	/*
	 * Holds status registers
	 * Requires PDO config 0x1A03 ( see page 22)
	 */
	struct VelocityRead
	{
		int32_t position;				 // 32
		uint32_t digital_inputs; // 32
		int32_t velocity;				 // 32
		uint16_t status;				 // 16 -> 112 bit input reg size
	};
	typedef boost::array<VelocityRead, NUMBER_OF_SLAVES> DataVec;

	/**
	 * Realtime Driver for Ombibase
	 *
	 * Ethernet communication done via SOEM
	 * Spawns two rt threads for controlling and ethercat monitoring
	 */
	class OmnibaseRTDriver
	{
	public:
		enum State
		{
			CONSTRUCTED,
			INITIALIZED,
			RUNNING,
			STOPPING
		};

		OmnibaseRTDriver(const std::string &interface = "eth0", int freq = 1000);
		virtual ~OmnibaseRTDriver();

		/*
		 * return true if running
		 */
		bool is_running();

		/*
		 * return true if driver state is ready to read
		 */
		bool is_ready_read();

		/**
		 * initalize ethercat, reset driver registers
		 */
		bool init();

		/**
		 * start omnibase control thread
		 */
		bool start_request();

		/**
		 * stop omnibase control thread
		 */
		void stop_request();

		/**
		 * set ethernet interface name
		 */
		void set_interface(const std::string &interface);

		/**
		 * set maximum velocity
		 */
		void set_vel_limits(int32_t min_vel, int32_t max_vel);
		void set_vel_limits(int32_t vel_limit);

		/**
		 * write new velocity command
		 */
		void write(const VelocityVec &vel_vec);
		void write(const VelocityWrite &vel, size_t i = 0);

		/*
		 * read driver data
		 */
		void read(DataVec &data_vec);
		void read(VelocityRead &data, size_t i = 0);

	private:
		bool ethercat_init();

		bool omnidrive_init();
		bool omnidrive_recover();
		bool omnidrive_poweron();
		bool omnidrive_poweroff();
		bool omnidrive_set_speedcontrol();
		bool omnidrive_stop();

		void *run_control();
		void *run_ecat_check();

	private:
		State state;
		bool ready_read;

		// control send pdo commands, ecatcheck handels ethercat errors
		RTThread<OmnibaseRTDriver, &OmnibaseRTDriver::run_control> control_thread;
		RTThread<OmnibaseRTDriver, &OmnibaseRTDriver::run_ecat_check> ecatcheck_thread;

		// bus state
		bool is_op;
		int expected_wkc;
		int wkc;

		// synch command struct access
		boost::mutex rw_mutex;

		// i/o register mapping
		int32_t min_vel, max_vel;

		// ehtercat
		uint8_t currentgroup;

		std::string interface;
		int delay;
	};

}

#endif