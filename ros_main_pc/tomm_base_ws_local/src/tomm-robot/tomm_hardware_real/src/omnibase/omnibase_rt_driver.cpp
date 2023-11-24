#include <tomm_hardware_real/omnibase/omnibase_rt_driver.h>
#include <tomm_hardware_real/utilities/omnibase_util.h>

#include <limits>

using namespace omnibase_rt_driver;
using namespace omnibase_util;

// keep as global variables, since the allignment of these varibales is critical
char IOMap[4096];
struct VelocityRead* current[NUMBER_OF_SLAVES] = {0x00, 0x00, 0x00, 0x00};
struct VelocityWrite* target[NUMBER_OF_SLAVES] = {0x00, 0x00, 0x00, 0x00};

OmnibaseRTDriver::OmnibaseRTDriver(const std::string& interface, int freq)
    : control_thread(this),
      ecatcheck_thread(this),
      interface(interface),
      delay( int(1000.0 * 1000.0 / double(freq)) ),
      state(CONSTRUCTED),
      is_op(false),
      min_vel(std::numeric_limits<int32_t>::min()),
      max_vel(std::numeric_limits<int32_t>::max()),
      ready_read(false)
{
}

OmnibaseRTDriver::~OmnibaseRTDriver()
{
    stop_request();
}

bool OmnibaseRTDriver::is_running()
{
    return state == RUNNING;
}

bool OmnibaseRTDriver::is_ready_read()
{
    return ready_read;
}

bool OmnibaseRTDriver::init()
{
    // init network, pdo, status reg
    if( !ethercat_init() ) {
        printf("OmnibaseRTDriver::init: error ethercat_init\n");
        return false;
    }
    
    // start driver, set velo to zero
    if( !omnidrive_init() ) {
        printf("OmnibaseRTDriver::init: error omnidrive_init\n");
        return false;
    }
    state = INITIALIZED;
    return true;
}

bool OmnibaseRTDriver::start_request()
{
    if( state != INITIALIZED)
        return false;

    state = RUNNING;
    ready_read = false;

    // start ethercat check rt thread
    if( !ecatcheck_thread.start() ) {
        printf("OmnibaseRTDriver::start_request: error starting ecatcheck thread\n");
        state = CONSTRUCTED;
        return false;
    }
    // start control rt thread
    if( !control_thread.start() ) {
        printf("OmnibaseRTDriver::start_request: error starting control thread\n");
        state = CONSTRUCTED;
        return false;    
    }

    printf("start_request called: start process with id: %d\n", getpid());
    return true;
}

void OmnibaseRTDriver::stop_request()
{
    if( state != RUNNING )
        return;

    state = STOPPING;
    int elapsed = 0;
    while( state != CONSTRUCTED ) {
        usleep(10 * 1000);
        elapsed += 10;
        if( elapsed > 1000 ) {
            control_thread.terminate();
            ecatcheck_thread.terminate();
            usleep(100 * 1000);
            printf("OmnibaseRTDriver::stop_request: unclean termination\n");
            return;
        }
    }
}

void OmnibaseRTDriver::set_interface(const std::string& interface)
{
    this->interface = interface;
}

void OmnibaseRTDriver::set_vel_limits(int32_t min_vel, int32_t max_vel)
{
    this->min_vel = min_vel;
    this->max_vel = max_vel;
}

void OmnibaseRTDriver::set_vel_limits(int32_t vel_limit)
{
    this->min_vel = -vel_limit;
    this->max_vel = vel_limit;
}

void OmnibaseRTDriver::write(const VelocityVec& vel_vec)
{
    // only copy veloctiy
    boost::lock_guard<boost::mutex> lock(rw_mutex);
    for( int i = 0; i < NUMBER_OF_SLAVES; ++i)
        target[i]->velocity = LIMIT_MIN_MAX(vel_vec[i].velocity, min_vel, max_vel);
}

void OmnibaseRTDriver::write(const VelocityWrite& vel, size_t i)
{
    // only copy veloctiy
    boost::lock_guard<boost::mutex> lock(rw_mutex);
    target[i]->velocity = LIMIT_MIN_MAX(vel.velocity, min_vel, max_vel);
}

void OmnibaseRTDriver::read(DataVec& data_vec )
{
    // read all data
    boost::lock_guard<boost::mutex> lock(rw_mutex);
    for( int i = 0; i < NUMBER_OF_SLAVES; ++i)
        data_vec[i] = *current[i];
}

void OmnibaseRTDriver::read(VelocityRead& data, size_t i)
{
    // read all data
    boost::lock_guard<boost::mutex> lock(rw_mutex);
    data = *current[i];
}

void* OmnibaseRTDriver::run_control()
{
    uint8_t buf8;
    bool reached_inital = false;

    while( state == RUNNING )
    {
        // PDO I/O refresh
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        ready_read = true;

        if(wkc >= expected_wkc)
        {
            // if in fault or in the way to normal status
            for( int j = 0; j < NUMBER_OF_SLAVES; ++j) {
                switch(target[j]->status) {
                case 0:
                    target[j]->status = 6;
                    break;
                case 6:
                    target[j]->status = 7;
                    break;
                case 7:
                    target[j]->status = 15;
                    break;
                case 128:
                    target[j]->status = 0;
                    break;
                default:
                    if(current[j]->status >> 3 & 0x01) {
                        read_register(j+1, 0x1001, 0, buf8);
                        printf("Slave[%d] error = %d, check the emergency button!\n", j, buf8);
                        target[j]->status = 128;
                    }
                }
            }
        }
        usleep(delay); 
    }

    // shutdown dirver
    printf("OmnibaseRTDriver::run_control: shutdown driver\n");
    omnidrive_stop();
    omnidrive_poweroff();
    ec_close();

    state = CONSTRUCTED;
    State printState = state;
    is_op = false;
}

bool OmnibaseRTDriver::ethercat_init()
{
    currentgroup = 0;
    char* interface_cstr = &interface[0];

    printf("OmnibaseRTDriver::ethercat_init: on %s\n", interface_cstr);

    /* initialise SOEM, bind socket to ifname */
    if ( ec_init(interface_cstr) ) 
    {
        if( ec_config_init(FALSE) > 0 )
        {
            if( ec_slavecount == NUMBER_OF_SLAVES ) 
            {
                /* CompleteAccess disabled for Elmo driver */
                for(int i = 1; i <= ec_slavecount; ++i)
                    ec_slave[i].CoEdetails ^= ECT_COEDET_SDOCA;

                /* switch to speed velocity mode */
                ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);
                omnidrive_set_speedcontrol();

                /* configure pdo mapping for velocity mode.
                 * Registers PDO_OUTPUT_ASSIGNMENT and PDO_INPUT_ASSIGNMENT
                 * determine which control registers are available for reading/
                 * writing command to the driver
                 */
                int32_t pdo_out = PDO_RECEIVE_OBJ;
                int32_t pdo_in = PDO_TRANSMIT_OBJ;
                int os = sizeof(pdo_in);
                for(int i = 1; i <= ec_slavecount; ++i) {
                    ec_SDOwrite(i, PDO_OUTPUT_ASSIGNMENT, 0, TRUE, os, &pdo_out, EC_TIMEOUTRXM);
                    ec_SDOwrite(i, PDO_INPUT_ASSIGNMENT, 0, TRUE, os, &pdo_in, EC_TIMEOUTRXM);
                }

                /* if CA disable => automapping works */
                ec_config_map(&IOMap);

                /* let DC off for the time being */
                //ec_configdc();

                int expected_Obits = 8*sizeof(VelocityWrite);
                int expected_Ibits = 8*sizeof(VelocityRead);
                printf("Expected: output size: %dbits | input size: %dbits\n", expected_Obits, expected_Ibits);
                for(int i = 1; i <= ec_slavecount; ++i) {
                    printf("Slave:%d | name:%s | output size: %dbits | input size: %dbits\n", i, ec_slave[1].name, ec_slave[1].Obits, ec_slave[1].Ibits);
                }

                /* basic settings */
                uint32_t heartbeat = 1;
                for( int i = 1; i <= ec_slavecount; ++i) {
                    write_register(i, 0x10F1, 2, heartbeat);
                }

                uint8_t time_period = 2;
                uint16_t interpolation_timeout = 2;
                for(int i = 1; i <= ec_slavecount; ++i) {
                    write_register(i, 0x60c2, 1, time_period);
                    write_register(i, 0x2f75, 0, interpolation_timeout);
                }

                /* wait for all slaves to reach SAFE_OP state */
                ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

                /* old SOEM code, inactive */
                int oloop, iloop;
                oloop = ec_slave[0].Obytes;
                if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
                if (oloop > 20) oloop = 8;
                iloop = ec_slave[0].Ibytes;
                if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
                if (iloop > 20) iloop = 8;

                printf("Request operational state for all slaves\n");
                expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

                /* going operational */
                ec_slave[0].state = EC_STATE_OPERATIONAL;

                /* send one valid process data to make outputs in slaves happy*/
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);

                /* request OP state for all slaves */
                ec_writestate(0);

                /* wait for all slaves to reach OP state */
                int chk = 40;
                do {
                    ec_send_processdata();
                    ec_receive_processdata(EC_TIMEOUTRET);
                    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
                }
                while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

                if (ec_slave[0].state == EC_STATE_OPERATIONAL ) 
                {
                    /* Map the registers to input/ouput structs
                     * Important: structs must stay bit alligned
                     * for the PDO mapping to work
                     */
                    printf("OmnibaseRTDriver::ethercat_init: %d slaves mapped.\n", ec_slavecount);
                    is_op = true;
                    for(int i = 1; i <= ec_slavecount; ++i) {
                        target[i-1] = (VelocityWrite*)(ec_slave[i].outputs);
                        current[i-1] = (VelocityRead*)(ec_slave[i].inputs);
                    }

                    /* ethercat is ready to go */
                    return true;
                }
                else {
                    printf("OmnibaseRTDriver::ethercat_init: not all slaves reached operational state.\n");
                    ec_readstate();
                    for(int i = 1; i <= ec_slavecount ; i++) {
                        if(ec_slave[i].state != EC_STATE_OPERATIONAL) {
                            printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                                i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                        }
                    }
                }
                omnidrive_stop();
                printf("OmnibaseRTDriver::ethercat_init: switch back to init state.\n");
                ec_slave[0].state = EC_STATE_INIT;
                ec_writestate(0);
            }
            else {
                printf("OmnibaseRTDriver::ethercat_init: Only %d slaves found.\n", ec_slavecount);
            }
        }
        else {
            printf("OmnibaseRTDriver::ethercat_init: no slaves found.\n");
        }
        printf("OmnibaseRTDriver::ethercat_init: close socket\n");
        ec_close();
    }
    else {
        printf("OmnibaseRTDriver::ethercat_init: failed to init SOEM.\n");
    }
    return false;
}

bool OmnibaseRTDriver::omnidrive_init()
{
    printf("OmnibaseRTDriver::omnidrive_init\n");

    /* set target to zero */
    for( int i = 0; i < NUMBER_OF_SLAVES; ++i) {
        target[i]->velocity = 0;
    }

    /* switch on procedure */
    omnidrive_poweroff();
    usleep(100*1000);
    omnidrive_recover();
    usleep(100*1000);
    omnidrive_poweron();
    usleep(100*1000);

    /* set accerations */
    uint32_t profile_acceleration = 5000000;
    uint32_t profile_deceleration = 5000001;
    for(int i = 1; i <= ec_slavecount; ++i) {
        write_register(i, PROFILE_ACCELERATION, 0, profile_acceleration);
        write_register(i, PROFILE_DECELERATION, 0, profile_deceleration);
    }
    return true;
}

bool OmnibaseRTDriver::omnidrive_recover()
{
   uint16_t val = 0x80;
   uint16_t index = CONTROL_WORD;
   for(int i = 1; i <= ec_slavecount; ++i)
      write_register(i, index, 0, val);
    return true;
}

bool OmnibaseRTDriver::omnidrive_poweron()
{
    uint16_t val;
    uint16_t index = CONTROL_WORD;

    /* shutdown */
    val = 0x06;
    for(int i = 1; i <= ec_slavecount; ++i)
        write_register(i, index, 0, val);
    usleep(100000);

    /* switch on */
    val = 0x07;
    for(int i = 1; i <= ec_slavecount; ++i)
        write_register(i, index, 0, val);
    usleep(100000);

    /* enable operation */
    val = 0x0f;
    for(int i = 1; i <= ec_slavecount; ++i)
        write_register(i, index, 0, val);
    usleep(100000);
    return true;
}

bool OmnibaseRTDriver::omnidrive_poweroff()
{
    uint16_t val = 0x00;
    uint16_t index = CONTROL_WORD;
    for(int i = 1; i <= ec_slavecount; ++i)
        write_register(i, index, 0, val);
    return true;
}

bool OmnibaseRTDriver::omnidrive_set_speedcontrol()
{
    /* puts driver into speed control mode */
    uint8_t val = 3;
    uint16_t index = OPERATION_MODE;
    for(int i = 1; i <= ec_slavecount; ++i)
        write_register(i, index, 0, val);
    return true;
}

bool OmnibaseRTDriver::omnidrive_stop()
{
    int32_t val = 0;
    uint16_t index = TARGET_VELOCITY;
    for (int i = 1; i <= ec_slavecount; i++)
        write_register(i, index, 0x00, val);
    return true;
}

void* OmnibaseRTDriver::run_ecat_check()
{
    printf("OmnibaseRTDriver::run_ecat_check -- RUNNING\n");
    int slave;
    while( state == RUNNING ) 
    {
        if( is_op && ((wkc < expected_wkc) || ec_group[currentgroup].docheckstate))
        {
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);                              
                  }
                  else if(ec_slave[slave].state > 0)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);                           
                     }
                  } 
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (!ec_slave[slave].state)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);                           
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(!ec_slave[slave].state)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);                           
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);                           
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        usleep(10000);
    }
    printf("OmnibaseRTDriver::run_ecat_check: shutdown ecat_check\n");
}