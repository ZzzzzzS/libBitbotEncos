#include "bus/Encos_bus.h"
#include "algorithm"
#include "iostream"
#include "ethercat.h"

namespace bitbot
{
    extern void EncosRegisterDeviceWarper();

    EncosBus::EncosBus()
        : ErrorFlag(false)
    {
    }

    EncosBus::~EncosBus() {}

    void EncosBus::RegisterDevices()
    {
        // isolate specific device and bus, increase flexibility.
        EncosRegisterDeviceWarper();
    }

    void EncosBus::PowerOnDevice(int id)
    {
        if (id == -1)
        {
            for (auto dev : this->devices_)
            {
                if (dev->HasPowerCfg())
                {
                    if (dev->PowerOn())
                    {
                        this->logger_->info("Power on device: {}", dev->Id());
                    }
                    else
                    {
                        this->logger_->error("Power on device: {} failed", dev->Id());
                    }
                }
            }
        }
        else
        {
            if (this->devices_[id]->HasPowerCfg())
            {
                if (this->devices_[id]->PowerOn())
                {
                    this->logger_->info("Power on device: {}", this->devices_[id]->Id());
                }
                else
                {
                    this->logger_->error("Power on device: {} failed", this->devices_[id]->Id());
                }
            }
        }
    }

    void EncosBus::PowerOffDevice(int id)
    {
        if (id == -1)
        {
            for (auto dev : this->devices_)
            {
                if (dev->PowerOff())
                {
                    this->logger_->info("Power off device: {}", dev->Id());
                }
                else
                {
                    this->logger_->error("Power off device: {} failed", dev->Id());
                }
            }
        }
        else
        {
            if (this->devices_[id]->PowerOff())
            {
                this->logger_->info("Power off device: {}", this->devices_[id]->Id());
            }
            else
            {
                this->logger_->error("Power off device: {} failed", this->devices_[id]->Id());
            }
        }
    }

    void EncosBus::Init()
    {
        this->CAN_Device_By_EtherCAT_ID.resize(ec_slavecount);
        this->CAN_BusReadBuffer.resize(ec_slavecount);
        this->CAN_BusWriteBuffer.resize(ec_slavecount);
        for (size_t i = 0; i < ec_slavecount; i++)
        {
            EtherCAT_Msg* slave_dest_o = (EtherCAT_Msg*)(ec_slave[i + 1].outputs);
            EtherCAT_Msg* slave_dest_i = (EtherCAT_Msg*)(ec_slave[i + 1].inputs);
            if (slave_dest_o == nullptr)
            {
                this->logger_->error("Failed to get EtherCAT slave output buffer, check your connection");
                this->ErrorFlag.store(true);
            }
            else
            {
                this->CAN_BusWriteBuffer[i] = slave_dest_o;
            }

            if (slave_dest_i == nullptr)
            {
                this->logger_->error("Failed to get EtherCAT slave input buffer, check your connection");
                this->ErrorFlag.store(true);
            }
            else
            {
                this->CAN_BusReadBuffer[i] = slave_dest_i;
            }
        }

        for (auto dev : this->devices_)
        {
            if (dev->VirtualBusDevice())
            {
                Encos_VirtualBusDevice* VDev = dynamic_cast<Encos_VirtualBusDevice*>(dev);
                if (VDev == nullptr)
                {
                    this->logger_->error("Unknown device type, check your configuration xml.");
                    this->ErrorFlag.store(true);
                }
                this->VirtualBusDevices.push_back(VDev);
            }
            else
            {
                Encos_CANBusDevice* CanDev = dynamic_cast<Encos_CANBusDevice*>(dev);
                if (CanDev == nullptr)
                {
                    this->logger_->error("Unknown device type, check your configuration xml.");
                    this->ErrorFlag.store(true);
                }

                std::vector<size_t> CAN_IDs;
                CanDev->get_CAN_IDs(CAN_IDs);
                size_t SlaveID = CanDev->get_EtherCAT_Slave_ID();
                this->CAN_Device_By_EtherCAT_ID[SlaveID].push_back(CanDev);

                for (size_t i = 0; i < CAN_IDs.size(); i++)
                {

                    if (this->CAN_Device_By_CAN_ID.count(CAN_IDs[i]) == 0)
                    {
                        this->CAN_Device_By_CAN_ID.insert({ CAN_IDs[i], std::vector<Encos_CANBusDevice*>() });
                    }
                    this->CAN_Device_By_CAN_ID[CAN_IDs[i]].push_back(CanDev);
                }
            }
        }

        for (size_t i = 0; i < CAN_Device_By_EtherCAT_ID.size(); i++)
        {
            std::sort(this->CAN_Device_By_EtherCAT_ID[i].begin(), this->CAN_Device_By_EtherCAT_ID[i].end(), [](Encos_CANBusDevice* a, Encos_CANBusDevice* b)
                { return a->Id() < b->Id(); });

            if (this->CAN_Device_By_EtherCAT_ID[i].size() > 6)
            {
                this->logger_->error("CAN devices for each EtherCAT salves can not be greater than 6, check your configuration xml.");
                this->ErrorFlag.store(true);
            }
        }
    }

    void EncosBus::WriteBus()
    {
        for (auto&& dev : this->VirtualBusDevices)
        {
            dev->WriteOnce();
        }

        for (size_t i = 0; i < this->CAN_Device_By_EtherCAT_ID.size(); i++)
        {
            for (size_t j = 0; j < this->CAN_Device_By_EtherCAT_ID[i].size(); j++)
            {
                this->CAN_Device_By_EtherCAT_ID[i][j]->WriteBus(this->CAN_BusWriteBuffer[i]->device[j]);
            }
            this->CAN_BusWriteBuffer[i]->device_number = this->CAN_Device_By_EtherCAT_ID[i].size();
            this->CAN_BusWriteBuffer[i]->can_ide = 0;
        }
        ec_send_processdata();
    }

    void EncosBus::ReadBus()
    {
        for (auto&& dev : this->VirtualBusDevices)
        {
            dev->ReadOnce();
        }

        this->wkc = ec_receive_processdata(EC_TIMEOUTRET);

        for (size_t i = 0; i < this->CAN_BusReadBuffer.size(); i++)
        {
            this->CAN_BusReadBuffer[i] = (EtherCAT_Msg*)(ec_slave[i + 1].inputs);
            // std::cout<<static_cast<int>(this->CAN_BusReadBuffer[i]->device_number)<<std::endl;
            for (size_t j = 0; j < this->CAN_BusReadBuffer[i]->device_number; j++)
            {
                size_t canid = this->CAN_BusReadBuffer[i]->device[j].id;
                // std::cout<<canid<<"rtr"<<static_cast<int>(this->CAN_BusReadBuffer[i]->device[j].rtr)<<std::endl;

                for (auto dev : this->CAN_Device_By_CAN_ID[canid])
                {
                    dev->ReadBus(this->CAN_BusReadBuffer[i]->device[j]);
                }
            }
        }

        if (this->check_cnt % this->check_cycle == 0)
        {
            this->EtherCATStateCheck();
        }
        this->check_cnt++;
    }

    bool EncosBus::InitEtherCAT(const std::string& ifname)
    {
        int i;
        int oloop, iloop, chk;
        needlf = false;
        inOP = false;

        num = 1;

        /* initialise SOEM, bind socket to ifname */
        if (ec_init(ifname.c_str()))
        {
            std::string info = std::string("Launch EtherCAT interface ") + ifname + std::string(" succeeded, looking for slaves...");
            this->logger_->info(info);
            /* find and auto-config slaves */

            if (ec_config_init(false) > 0)
            {
                std::string info = std::to_string(ec_slavecount) + std::string(" slaves found and configured.");
                this->logger_->info(info);

                for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
                    ec_slave[slave_idx + 1].CoEdetails &= ~ECT_COEDET_SDOCA;

                ec_config_map(&IOmap);
                ec_configdc();

                this->logger_->info("Slaves mapped.");
                /* wait for all slaves to reach SAFE_OP state */
                ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * ec_slavecount * 2);

                for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
                {
                    // printf("[SLAVE %d]\n", slave_idx);
                    // printf("  IN  %d bytes, %d bits\n", ec_slave[slave_idx].Ibytes, ec_slave[slave_idx].Ibits);
                    // printf("  OUT %d bytes, %d bits\n", ec_slave[slave_idx].Obytes, ec_slave[slave_idx].Obits);
                    // printf("\n");

                    info = std::string("[SLAVE ") + std::to_string(slave_idx) + std::string("]:");
                    info += std::string(" IN  ") + std::to_string(ec_slave[slave_idx].Ibytes) + std::string(" bytes, ") + std::to_string(ec_slave[slave_idx].Ibits) + std::string(" bits,");
                    info += std::string(" OUT ") + std::to_string(ec_slave[slave_idx].Obytes) + std::string(" bytes, ") + std::to_string(ec_slave[slave_idx].Obits) + std::string(" bits.");
                    this->logger_->debug(info);
                }

                oloop = ec_slave[0].Obytes;
                if ((oloop == 0) && (ec_slave[0].Obits > 0))
                    oloop = 1;
                if (oloop > 8)
                    oloop = 8;
                iloop = ec_slave[0].Ibytes;
                if ((iloop == 0) && (ec_slave[0].Ibits > 0))
                    iloop = 1;
                if (iloop > 8)
                    iloop = 8;

                info = std::string("segments : ") + std::to_string(ec_group[0].nsegments) + " : " + std::to_string(ec_group[0].IOsegment[0]) + " " + std::to_string(ec_group[0].IOsegment[1]) + " " + std::to_string(ec_group[0].IOsegment[2]) + " " + std::to_string(ec_group[0].IOsegment[3]);
                this->logger_->debug(info);

                this->logger_->debug("Requesting operational state for all slaves...");
                expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
                info = std::string("Calculated workcounter ") + std::to_string(expectedWKC);
                this->logger_->debug(info);
                ec_slave[0].state = EC_STATE_OPERATIONAL;
                /* send one valid process data to make outputs in slaves happy*/
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                /* request OP state for all slaves */
                ec_writestate(0);
                chk = 40;
                /* wait for all slaves to reach OP state */
                do
                {
                    ec_send_processdata();
                    ec_receive_processdata(EC_TIMEOUTRET);
                    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
                } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

                if (ec_slave[0].state == EC_STATE_OPERATIONAL)
                {
                    this->logger_->info("Operational state reached for all slaves.");
                    inOP = true;
                    return true;
                }
                else
                {
                    this->logger_->warn("Not all slaves reached operational state.");
                    ec_readstate();
                    for (i = 1; i <= ec_slavecount; i++)
                    {
                        if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                        {
                            char str[1024];
                            sprintf(str, "[EtherCAT Error] Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                                i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                            std::string info(str);
                            this->logger_->debug(info);
                        }
                    }
                }
            }
            else
            {
                this->logger_->error("No slaves found!");
            }
        }
        else
        {
            std::string info = std::string("No socket connection on ") + ifname + " please set capability first or run in root.";
            this->logger_->error(info);
            this->logger_->error("visit https://github.com/OpenEtherCATsociety/SOEM/issues/83 for more information.");
        }
        return false;
    }

    std::vector<Encos_CANBusDevice*> EncosBus::get_CAN_Devices()
    {
        std::vector<Encos_CANBusDevice*> can_devices;
        for (auto dev : this->devices_)
        {
            if (dev->VirtualBusDevice())
            {
                continue;
            }
            Encos_CANBusDevice* CanDev = dynamic_cast<Encos_CANBusDevice*>(dev);
            if (CanDev == nullptr)
            {
                this->logger_->error("Unknown device type, check your configuration xml.");
                this->ErrorFlag.store(true);
            }
            can_devices.push_back(CanDev);
        }
        return can_devices;
    }

    std::vector<Encos_VirtualBusDevice*> EncosBus::get_VirtualBusDevices()
    {
        return this->VirtualBusDevices;
    }

    void EncosBus::EtherCATStateCheck()
    {
        // count errors
        if (err_iteration_count > K_ETHERCAT_ERR_PERIOD)
        {
            err_iteration_count = 0;
            err_count = 0;
        }

        if (err_count > K_ETHERCAT_ERR_MAX)
        {
            // possibly shut down
            this->logger_->error("EtherCAT connection degraded!");
        }
        err_iteration_count++;

        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        char buf[256];
                        sprintf(buf, "EtherCAT Error: Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        this->logger_->error(std::string(buf));

                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                        err_count++;
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        char buf[256];
                        sprintf(buf, "EtherCAT Status: Slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        this->logger_->info(std::string(buf));

                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                        err_count++;
                    }
                    else if (ec_slave[slave].state > 0)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            char buf[256];
                            sprintf(buf, "EtherCAT Status: Slave %d reconfigured\n", slave);
                            this->logger_->info(std::string(buf));
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            char buf[256];
                            sprintf(buf, "EtherCAT Error: Slave %d lost\n", slave);
                            this->logger_->error(std::string(buf));
                            err_count++;
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (!ec_slave[slave].state)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            char buf[256];
                            sprintf(buf, "EtherCAT Status: Slave %d recovered\n", slave);
                            this->logger_->info(std::string(buf));
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        char buf[256];
                        sprintf(buf, "EtherCAT Status: Slave %d found\n", slave);
                        this->logger_->info(std::string(buf));
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
            {
                this->logger_->info("EtherCAT Status: All slaves resumed OPERATIONAL.");
            }
        }
    }

};
