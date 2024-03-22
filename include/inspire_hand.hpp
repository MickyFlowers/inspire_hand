#ifndef __INSPIRE_HAND_HPP__
#define __INSPIRE_HAND_HPP__

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <unordered_map>
#include <vector>
using namespace std;

namespace inspire_hand
{
    typedef enum _cmd_type_e
    {
        SetId = 0x01,
        SetBaudrate = 0x02,
        ClearError = 0x03,
        SaveFlash = 0x04,
        Reset = 0x05,
        ForceCalibration = 0x06,
        SetCurrentLimit = 0x07,
        SetDefaultSpeed = 0x08,
        SetDefaultForceThreshold = 0x09,
        SetPos = 0x0A,
        SetAngle = 0x0B,
        SetForceThreshold = 0x0C,
        SetSpeed = 0x0D,
        GetPos = 0x0E,
        GetAngle = 0x0F,
        GetForce = 0x10,
        GetCurrent = 0x11,
        GetError = 0x12,
        GetStatus = 0x13,
        GetTemperature = 0x14,
        GetPosSet = 0x15,
        GetAngleSet = 0x16,
        GetForceSet = 0x17,

    } CmdType_e;

    class InspireHand
    {
    public:
        struct cmd
        {
            uint8_t set_id;
            int baudrate;
            vector<uint16_t> current_limit;
            vector<uint16_t> default_speed;
            vector<uint16_t> default_force_threshold;
            vector<uint16_t> pos;
            vector<uint16_t> angle;
            vector<uint16_t> force_threshold;
            vector<uint16_t> speed;
        };
        struct fdb
        {
            vector<uint8_t> error_code;
            vector<uint8_t> status;
            vector<uint8_t> temperature;

            vector<uint16_t> speed;
            vector<uint16_t> pos;
            vector<uint16_t> angle;
            vector<uint16_t> force;
            vector<uint16_t> current;

            vector<uint16_t> pos_set;
            vector<uint16_t> angle_set;
            vector<uint16_t> force_set;
        };

        InspireHand(uint8_t id, int baudrate)
        {
            // set default id to 1
            id_ = id;
            boundrate_ = baudrate;

            // initialize feedback data
            fdb_.error_code.resize(6);
            fdb_.status.resize(6);
            fdb_.temperature.resize(6);
            fdb_.speed.resize(6);
            fdb_.pos.resize(6);
            fdb_.angle.resize(6);
            fdb_.force.resize(6);
            fdb_.current.resize(6);

            fdb_.pos_set.resize(6);
            fdb_.angle_set.resize(6);
            fdb_.force_set.resize(6);

            // initialize command data
            cmd_.current_limit.resize(6);
            cmd_.default_speed.resize(6);
            cmd_.default_force_threshold.resize(6);
            cmd_.pos.resize(6);
            cmd_.angle.resize(6);
            cmd_.force_threshold.resize(6);
            cmd_.speed.resize(6);
        }
        vector<uint8_t> encode(CmdType_e command_type)
        {
            vector<uint8_t> data;
            switch (command_type)
            {
            case SetId:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x04);
                data.push_back(0x12);
                data.push_back(0xE8);
                data.push_back(0x03);
                data.push_back(cmd_.set_id);
                data.push_back(checksum(data));
                break;
            case SetBaudrate:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x04);
                data.push_back(0x12);
                data.push_back(0xE9);
                data.push_back(0x03);
                switch (cmd_.baudrate)
                {
                case 115200:
                    data.push_back(0x00);
                    break;
                case 57600:
                    data.push_back(0x01);
                    break;
                case 19200:
                    data.push_back(0x02);
                    break;
                default:
                    data.push_back(0x00);
                    break;
                }
                data.push_back(checksum(data));
                break;
            case ClearError:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x04);
                data.push_back(0x12);
                data.push_back(0xEC);
                data.push_back(0x03);
                data.push_back(0x01);
                data.push_back(checksum(data));
                break;
            case SaveFlash:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x04);
                data.push_back(0x12);
                data.push_back(0xED);
                data.push_back(0x03);
                data.push_back(0x01);
                data.push_back(checksum(data));
                break;
            case Reset:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x04);
                data.push_back(0x12);
                data.push_back(0xEE);
                data.push_back(0x03);
                data.push_back(0x01);
                data.push_back(checksum(data));
                break;
            case ForceCalibration:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x04);
                data.push_back(0x12);
                data.push_back(0xF1);
                data.push_back(0x03);
                data.push_back(0x01);
                data.push_back(checksum(data));
                break;
            case SetCurrentLimit:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x0F);
                data.push_back(0x12);
                data.push_back(0xFC);
                data.push_back(0x03);
                data.push_back(cmd_.current_limit[0] & 0xFF);
                data.push_back(cmd_.current_limit[0] >> 8);
                data.push_back(cmd_.current_limit[1] & 0xFF);
                data.push_back(cmd_.current_limit[1] >> 8);
                data.push_back(cmd_.current_limit[2] & 0xFF);
                data.push_back(cmd_.current_limit[2] >> 8);
                data.push_back(cmd_.current_limit[3] & 0xFF);
                data.push_back(cmd_.current_limit[3] >> 8);
                data.push_back(cmd_.current_limit[4] & 0xFF);
                data.push_back(cmd_.current_limit[4] >> 8);
                data.push_back(cmd_.current_limit[5] & 0xFF);
                data.push_back(cmd_.current_limit[5] >> 8);
                data.push_back(checksum(data));
                break;
            case SetDefaultSpeed:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x0F);
                data.push_back(0x12);
                data.push_back(0x08);
                data.push_back(0x04);
                data.push_back(cmd_.default_speed[0] & 0xFF);
                data.push_back(cmd_.default_speed[0] >> 8);
                data.push_back(cmd_.default_speed[1] & 0xFF);
                data.push_back(cmd_.default_speed[1] >> 8);
                data.push_back(cmd_.default_speed[2] & 0xFF);
                data.push_back(cmd_.default_speed[2] >> 8);
                data.push_back(cmd_.default_speed[3] & 0xFF);
                data.push_back(cmd_.default_speed[3] >> 8);
                data.push_back(cmd_.default_speed[4] & 0xFF);
                data.push_back(cmd_.default_speed[4] >> 8);
                data.push_back(cmd_.default_speed[5] & 0xFF);
                data.push_back(cmd_.default_speed[5] >> 8);
                data.push_back(checksum(data));
                break;
            case SetDefaultForceThreshold:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x0F);
                data.push_back(0x12);
                data.push_back(0x14);
                data.push_back(0x04);
                data.push_back(cmd_.default_force_threshold[0] & 0xFF);
                data.push_back(cmd_.default_force_threshold[0] >> 8);
                data.push_back(cmd_.default_force_threshold[1] & 0xFF);
                data.push_back(cmd_.default_force_threshold[1] >> 8);
                data.push_back(cmd_.default_force_threshold[2] & 0xFF);
                data.push_back(cmd_.default_force_threshold[2] >> 8);
                data.push_back(cmd_.default_force_threshold[3] & 0xFF);
                data.push_back(cmd_.default_force_threshold[3] >> 8);
                data.push_back(cmd_.default_force_threshold[4] & 0xFF);
                data.push_back(cmd_.default_force_threshold[4] >> 8);
                data.push_back(cmd_.default_force_threshold[5] & 0xFF);
                data.push_back(cmd_.default_force_threshold[5] >> 8);
                data.push_back(checksum(data));
                break;
            case SetPos:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x0F);
                data.push_back(0x12);
                data.push_back(0xC2);
                data.push_back(0x05);
                data.push_back(cmd_.pos[0] & 0xFF);
                data.push_back(cmd_.pos[0] >> 8);
                data.push_back(cmd_.pos[1] & 0xFF);
                data.push_back(cmd_.pos[1] >> 8);
                data.push_back(cmd_.pos[2] & 0xFF);
                data.push_back(cmd_.pos[2] >> 8);
                data.push_back(cmd_.pos[3] & 0xFF);
                data.push_back(cmd_.pos[3] >> 8);
                data.push_back(cmd_.pos[4] & 0xFF);
                data.push_back(cmd_.pos[4] >> 8);
                data.push_back(cmd_.pos[5] & 0xFF);
                data.push_back(cmd_.pos[5] >> 8);
                data.push_back(checksum(data));
                break;
            case SetAngle:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x0F);
                data.push_back(0x12);
                data.push_back(0xCE);
                data.push_back(0x05);
                data.push_back(cmd_.angle[0] & 0xFF);
                data.push_back(cmd_.angle[0] >> 8);
                data.push_back(cmd_.angle[1] & 0xFF);
                data.push_back(cmd_.angle[1] >> 8);
                data.push_back(cmd_.angle[2] & 0xFF);
                data.push_back(cmd_.angle[2] >> 8);
                data.push_back(cmd_.angle[3] & 0xFF);
                data.push_back(cmd_.angle[3] >> 8);
                data.push_back(cmd_.angle[4] & 0xFF);
                data.push_back(cmd_.angle[4] >> 8);
                data.push_back(cmd_.angle[5] & 0xFF);
                data.push_back(cmd_.angle[5] >> 8);
                data.push_back(checksum(data));
                break;
            case SetForceThreshold:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x0F);
                data.push_back(0x12);
                data.push_back(0xDA);
                data.push_back(0x05);
                data.push_back(cmd_.force_threshold[0] & 0xFF);
                data.push_back(cmd_.force_threshold[0] >> 8);
                data.push_back(cmd_.force_threshold[1] & 0xFF);
                data.push_back(cmd_.force_threshold[1] >> 8);
                data.push_back(cmd_.force_threshold[2] & 0xFF);
                data.push_back(cmd_.force_threshold[2] >> 8);
                data.push_back(cmd_.force_threshold[3] & 0xFF);
                data.push_back(cmd_.force_threshold[3] >> 8);
                data.push_back(cmd_.force_threshold[4] & 0xFF);
                data.push_back(cmd_.force_threshold[4] >> 8);
                data.push_back(cmd_.force_threshold[5] & 0xFF);
                data.push_back(cmd_.force_threshold[5] >> 8);
                data.push_back(checksum(data));
                break;
            case SetSpeed:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x0F);
                data.push_back(0x12);
                data.push_back(0xF2);
                data.push_back(0x05);
                data.push_back(cmd_.speed[0] & 0xFF);
                data.push_back(cmd_.speed[0] >> 8);
                data.push_back(cmd_.speed[1] & 0xFF);
                data.push_back(cmd_.speed[1] >> 8);
                data.push_back(cmd_.speed[2] & 0xFF);
                data.push_back(cmd_.speed[2] >> 8);
                data.push_back(cmd_.speed[3] & 0xFF);
                data.push_back(cmd_.speed[3] >> 8);
                data.push_back(cmd_.speed[4] & 0xFF);
                data.push_back(cmd_.speed[4] >> 8);
                data.push_back(cmd_.speed[5] & 0xFF);
                data.push_back(cmd_.speed[5] >> 8);
                data.push_back(checksum(data));
                break;
            case GetPos:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x04);
                data.push_back(0x11);
                data.push_back(0xFE);
                data.push_back(0x05);
                data.push_back(0x0C);
                data.push_back(checksum(data));
                break;
            case GetAngle:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x04);
                data.push_back(0x11);
                data.push_back(0x0A);
                data.push_back(0x06);
                data.push_back(0x0C);
                data.push_back(checksum(data));
                break;
            case GetForce:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x04);
                data.push_back(0x11);
                data.push_back(0x2E);
                data.push_back(0x06);
                data.push_back(0x0C);
                data.push_back(checksum(data));
                break;
            case GetCurrent:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x04);
                data.push_back(0x11);
                data.push_back(0x3A);
                data.push_back(0x06);
                data.push_back(0x0C);
                data.push_back(checksum(data));
                break;
            case GetError:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x04);
                data.push_back(0x11);
                data.push_back(0x46);
                data.push_back(0x06);
                data.push_back(0x06);
                data.push_back(checksum(data));
                break;
            case GetStatus:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x04);
                data.push_back(0x11);
                data.push_back(0x4C);
                data.push_back(0x06);
                data.push_back(0x06);
                data.push_back(checksum(data));
                break;
            case GetTemperature:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x04);
                data.push_back(0x11);
                data.push_back(0x52);
                data.push_back(0x06);
                data.push_back(0x06);
                data.push_back(checksum(data));
                break;
            case GetPosSet:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x04);
                data.push_back(0x11);
                data.push_back(0xC2);
                data.push_back(0x05);
                data.push_back(0x0C);
                data.push_back(checksum(data));
                break;
            case GetAngleSet:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x04);
                data.push_back(0x11);
                data.push_back(0xCE);
                data.push_back(0x05);
                data.push_back(0x0C);
                data.push_back(checksum(data));
                break;
            case GetForceSet:
                data.push_back(0xEB);
                data.push_back(0x90);
                data.push_back(id_);
                data.push_back(0x04);
                data.push_back(0x11);
                data.push_back(0xDA);
                data.push_back(0x05);
                data.push_back(0x0C);
                data.push_back(checksum(data));
                break;
            }
            return data;
        }
        bool decode(CmdType_e command_type, vector<uint8_t> data)
        {
            if (data.empty())
            {
                return false;
            }
            uint8_t check_num = data[data.size() - 1];
            data.pop_back();
            if (check_num != checksum(data))
            {
                return false;
            }
            switch (command_type)
            {
            case GetPos:
                fdb_.pos[0] = data[7] | (data[8] << 8);
                fdb_.pos[1] = data[9] | (data[10] << 8);
                fdb_.pos[2] = data[11] | (data[12] << 8);
                fdb_.pos[3] = data[13] | (data[14] << 8);
                fdb_.pos[4] = data[15] | (data[16] << 8);
                fdb_.pos[5] = data[17] | (data[18] << 8);
                break;
            case GetAngle:
                fdb_.angle[0] = data[7] | (data[8] << 8);
                fdb_.angle[1] = data[9] | (data[10] << 8);
                fdb_.angle[2] = data[11] | (data[12] << 8);
                fdb_.angle[3] = data[13] | (data[14] << 8);
                fdb_.angle[4] = data[15] | (data[16] << 8);
                fdb_.angle[5] = data[17] | (data[18] << 8);
                break;
            case GetForce:
                fdb_.force[0] = data[7] | (data[8] << 8);
                fdb_.force[1] = data[9] | (data[10] << 8);
                fdb_.force[2] = data[11] | (data[12] << 8);
                fdb_.force[3] = data[13] | (data[14] << 8);
                fdb_.force[4] = data[15] | (data[16] << 8);
                fdb_.force[5] = data[17] | (data[18] << 8);
                break;
            case GetCurrent:
                fdb_.current[0] = data[7] | (data[8] << 8);
                fdb_.current[1] = data[9] | (data[10] << 8);
                fdb_.current[2] = data[11] | (data[12] << 8);
                fdb_.current[3] = data[13] | (data[14] << 8);
                fdb_.current[4] = data[15] | (data[16] << 8);
                fdb_.current[5] = data[17] | (data[18] << 8);
                break;
            case GetError:
                fdb_.error_code[0] = data[7];
                fdb_.error_code[1] = data[8];
                fdb_.error_code[2] = data[9];
                fdb_.error_code[3] = data[10];
                fdb_.error_code[4] = data[11];
                fdb_.error_code[5] = data[12];
                break;
            case GetStatus:
                fdb_.status[0] = data[7];
                fdb_.status[1] = data[8];
                fdb_.status[2] = data[9];
                fdb_.status[3] = data[10];
                fdb_.status[4] = data[11];
                fdb_.status[5] = data[12];
                break;
            case GetTemperature:
                fdb_.temperature[0] = data[7];
                fdb_.temperature[1] = data[8];
                fdb_.temperature[2] = data[9];
                fdb_.temperature[3] = data[10];
                fdb_.temperature[4] = data[11];
                fdb_.temperature[5] = data[12];
                break;
            case GetPosSet:
                fdb_.pos_set[0] = data[7] | (data[8] << 8);
                fdb_.pos_set[1] = data[9] | (data[10] << 8);
                fdb_.pos_set[2] = data[11] | (data[12] << 8);
                fdb_.pos_set[3] = data[13] | (data[14] << 8);
                fdb_.pos_set[4] = data[15] | (data[16] << 8);
                fdb_.pos_set[5] = data[17] | (data[18] << 8);
                break;
            case GetAngleSet:
                fdb_.angle_set[0] = data[7] | (data[8] << 8);
                fdb_.angle_set[1] = data[9] | (data[10] << 8);
                fdb_.angle_set[2] = data[11] | (data[12] << 8);
                fdb_.angle_set[3] = data[13] | (data[14] << 8);
                fdb_.angle_set[4] = data[15] | (data[16] << 8);
                fdb_.angle_set[5] = data[17] | (data[18] << 8);
                break;
            case GetForceSet:
                fdb_.force_set[0] = data[7] | (data[8] << 8);
                fdb_.force_set[1] = data[9] | (data[10] << 8);
                fdb_.force_set[2] = data[11] | (data[12] << 8);
                fdb_.force_set[3] = data[13] | (data[14] << 8);
                fdb_.force_set[4] = data[15] | (data[16] << 8);
                fdb_.force_set[5] = data[17] | (data[18] << 8);
                break;
            case SetId:
                id_ = data[2];
                break;
            }
            return true;
        }
        uint8_t checksum(vector<uint8_t> data)
        {
            uint16_t check_bit = 0;
            for (int i = 2; i < data.size(); i++)
            {
                check_bit += data[i];
            }
            return check_bit & 0xFF;
        }

        int boundrate_;
        uint8_t id_;
        fdb fdb_;
        cmd cmd_;
    };

    class InspireHandSerial
    {
    public:
        InspireHandSerial(string port, int baudrate) : port_(port), baudrate_(baudrate)
        {
            cout << "Inspire hand serial port: " << port_.c_str() << endl;
            cout << "Inspire hand serial baudrate: " << baudrate_ << endl;
            // open port
            if ((fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY)) < 0)
            {
                cout << "Failed to open the port" << endl;
                exit(1);
            }

            // set fd parameter
            struct termios options;
            tcgetattr(fd_, &options);
            switch (baudrate_)
            {
            case 19200:
                cfsetispeed(&options, B19200);
                cfsetospeed(&options, B19200);
                break;
            case 57600:
                cfsetispeed(&options, B57600);
                cfsetospeed(&options, B57600);
                break;
            case 115200:
                cfsetispeed(&options, B115200);
                cfsetospeed(&options, B115200);
                break;
            default:
                cfsetispeed(&options, B115200);
                cfsetospeed(&options, B115200);
                break;
            }
            options.c_cflag |= (CLOCAL | CREAD);

            options.c_cflag &= ~CRTSCTS;

            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS8;

            options.c_cflag &= ~PARENB;

            options.c_cflag &= ~CSTOPB;

            options.c_cc[VMIN] = 0;
            options.c_cc[VTIME] = 1;
            tcflush(fd_, TCIFLUSH);
            if (tcsetattr(fd_, TCSANOW, &options) != 0)
            {
                cout << "Failed to set the port" << endl;
                exit(1);
            }
            cout << "Serial port is set up successfully" << endl;
            serachHands();
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                cout << "****************************************Inspire hand id: " << int(inspire_hands_[i].id_) << "********************************" << endl;
                getAngle(inspire_hands_[i].id_);
                for (int j = 0; j < inspire_hands_[i].fdb_.angle.size(); j++)
                {
                    cout << "||Angle " << j << ":  " << inspire_hands_[i].fdb_.angle[j] << " ";
                }
                cout << endl;
                cout << "------------------------------------------------------------------------------------------" << endl;
                getError(inspire_hands_[i].id_);
                for (int j = 0; j < inspire_hands_[i].fdb_.error_code.size(); j++)
                {
                    cout << "||Error " << j << ":    " << int(inspire_hands_[i].fdb_.error_code[j]) << " ";
                }
                cout << endl;
                cout << "------------------------------------------------------------------------------------------" << endl;
                getForceSet(inspire_hands_[i].id_);
                for (int j = 0; j < inspire_hands_[i].fdb_.force_set.size(); j++)
                {
                    cout << "||Force " << j << ": " << inspire_hands_[i].fdb_.force_set[j] << " ";
                }
                cout << endl;
            }
            cout << "******************************************************************************************" << endl;
        }
        ~InspireHandSerial()
        {
            close(fd_);
        }

        void serachHands(void)
        {
            uint8_t recv_len = 14;
            for (int id = 1; id <= 10; id++)
            {
                InspireHand hand(id, baudrate_);
                vector<uint8_t> data = hand.encode(GetError);
                uint8_t send_data[data.size()];
                for (int i = 0; i < data.size(); i++)
                {
                    send_data[i] = data[i];
                }
                if (write(fd_, send_data, data.size()) < 0)
                {
                    cout << "Failed to write data" << endl;
                    return;
                }
                uint8_t recv_data[recv_len];
                if (read(fd_, recv_data, recv_len) <= 0)
                {
                    cout << "Failed to read data from id: " << id << endl;
                    continue;
                }

                vector<uint8_t> recv_data_vec(recv_len);
                for (int i = 0; i < recv_len; i++)
                {
                    recv_data_vec[i] = recv_data[i];
                }
                if (hand.decode(GetError, recv_data_vec))
                {
                    inspire_hands_.push_back(hand);
                }
            }
            cout << "There are " << inspire_hands_.size() << " inspire hands" << endl;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                cout << "Inspire hand id: " << int(inspire_hands_[i].id_) << endl;
            }
        }

        void setId(uint8_t id, uint8_t set_id)
        {
            uint8_t recv_len = 9;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    inspire_hands_[i].cmd_.set_id = set_id;
                    vector<uint8_t> data = inspire_hands_[i].encode(SetId);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(SetId, recv_data_vec))
                    {
                        cout << "Failed to set id" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when set id" << endl;
                }
            }
        }
        void setAngle(uint8_t id, vector<uint16_t> angle)
        {
            uint8_t recv_len = 9;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    inspire_hands_[i].cmd_.angle = angle;
                    vector<uint8_t> data = inspire_hands_[i].encode(SetAngle);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << int(id) << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(SetAngle, recv_data_vec))
                    {
                        cout << "Failed to set angle" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when set angle" << endl;
                }
            }
        }
        void setBaudrate(uint8_t id, int baudrate)
        {
            uint8_t recv_len = 9;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    inspire_hands_[i].cmd_.baudrate = baudrate;
                    vector<uint8_t> data = inspire_hands_[i].encode(SetBaudrate);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(SetBaudrate, recv_data_vec))
                    {
                        cout << "Failed to set baudrate" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when set baudrate" << endl;
                }
            }
        }
        void clearError(uint8_t id)
        {
            uint8_t recv_len = 9;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    vector<uint8_t> data = inspire_hands_[i].encode(ClearError);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(ClearError, recv_data_vec))
                    {
                        cout << "Failed to clear error" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when clear error" << endl;
                }
            }
        }
        void saveFlash(uint8_t id)
        {
            uint8_t recv_len = 9;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    vector<uint8_t> data = inspire_hands_[i].encode(SaveFlash);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(SaveFlash, recv_data_vec))
                    {
                        cout << "Failed to save flash" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when save flash" << endl;
                }
            }
        }
        void reset(uint8_t id)
        {
            uint8_t recv_len = 9;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    vector<uint8_t> data = inspire_hands_[i].encode(Reset);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(Reset, recv_data_vec))
                    {
                        cout << "Failed to reset" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when reset" << endl;
                }
            }
        }
        void forceCalibration(uint8_t id)
        {
            uint8_t recv_len = 9;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    vector<uint8_t> data = inspire_hands_[i].encode(ForceCalibration);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(ForceCalibration, recv_data_vec))
                    {
                        cout << "Failed to force calibration" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when force calibration" << endl;
                }
            }
        }
        void setCurrentLimit(uint8_t id, vector<uint16_t> current_limit)
        {
            uint8_t recv_len = 9;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    inspire_hands_[i].cmd_.current_limit = current_limit;
                    vector<uint8_t> data = inspire_hands_[i].encode(SetCurrentLimit);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(SetCurrentLimit, recv_data_vec))
                    {
                        cout << "Failed to set current limit" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when set current limit" << endl;
                }
            }
        }
        void setDefaultSpeed(uint8_t id, vector<uint16_t> default_speed)
        {
            uint8_t recv_len = 9;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    inspire_hands_[i].cmd_.default_speed = default_speed;
                    vector<uint8_t> data = inspire_hands_[i].encode(SetDefaultSpeed);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(SetDefaultSpeed, recv_data_vec))
                    {
                        cout << "Failed to set default speed" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when set default speed" << endl;
                }
            }
        }
        void setDefaultForceThreshold(uint8_t id, vector<uint16_t> default_force_threshold)
        {
            uint8_t recv_len = 9;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    inspire_hands_[i].cmd_.default_force_threshold = default_force_threshold;
                    vector<uint8_t> data = inspire_hands_[i].encode(SetDefaultForceThreshold);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(SetDefaultForceThreshold, recv_data_vec))
                    {
                        cout << "Failed to set default force threshold" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when set default force threshold" << endl;
                }
            }
        }
        void setPos(uint8_t id, vector<uint16_t> pos)
        {
            uint8_t recv_len = 9;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    inspire_hands_[i].cmd_.pos = pos;
                    vector<uint8_t> data = inspire_hands_[i].encode(SetPos);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(SetPos, recv_data_vec))
                    {
                        cout << "Failed to set pos" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when set pos" << endl;
                }
            }
        }
        void setForceThreshold(uint8_t id, vector<uint16_t> force_threshold)
        {
            uint8_t recv_len = 9;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    inspire_hands_[i].cmd_.force_threshold = force_threshold;
                    vector<uint8_t> data = inspire_hands_[i].encode(SetForceThreshold);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(SetForceThreshold, recv_data_vec))
                    {
                        cout << "Failed to set force threshold" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when set force threshold" << endl;
                }
            }
        }
        void setSpeed(uint8_t id, vector<uint16_t> speed)
        {
            uint8_t recv_len = 9;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    inspire_hands_[i].cmd_.speed = speed;
                    vector<uint8_t> data = inspire_hands_[i].encode(SetSpeed);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(SetSpeed, recv_data_vec))
                    {
                        cout << "Failed to set speed" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when set speed" << endl;
                }
            }
        }
        void getPos(uint8_t id)
        {
            uint8_t recv_len = 20;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    vector<uint8_t> data = inspire_hands_[i].encode(GetPos);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(GetPos, recv_data_vec))
                    {
                        cout << "Failed to get position" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when get pos" << endl;
                }
            }
        }
        void getAngle(uint8_t id)
        {
            uint8_t recv_len = 20;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    vector<uint8_t> data = inspire_hands_[i].encode(GetAngle);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(GetAngle, recv_data_vec))
                    {
                        cout << "Failed to get angle" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when get angle" << endl;
                }
            }
        }
        void getForce(uint8_t id)
        {
            uint8_t recv_len = 20;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    vector<uint8_t> data = inspire_hands_[i].encode(GetForce);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(GetForce, recv_data_vec))
                    {
                        cout << "Failed to get force" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when get force" << endl;
                }
            }
        }
        void getCurrent(uint8_t id)
        {
            uint8_t recv_len = 20;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    vector<uint8_t> data = inspire_hands_[i].encode(GetCurrent);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(GetCurrent, recv_data_vec))
                    {
                        cout << "Failed to get current" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when get current" << endl;
                }
            }
        }
        void getError(uint8_t id)
        {
            uint8_t recv_len = 14;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    vector<uint8_t> data = inspire_hands_[i].encode(GetError);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(GetError, recv_data_vec))
                    {
                        cout << "Failed to get error" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when get error" << endl;
                }
            }
        }
        void getStatus(uint8_t id)
        {
            uint8_t recv_len = 14;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    vector<uint8_t> data = inspire_hands_[i].encode(GetStatus);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(GetStatus, recv_data_vec))
                    {
                        cout << "Failed to get status" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when get status" << endl;
                }
            }
        }
        void getTemperature(uint8_t id)
        {
            uint8_t recv_len = 14;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    vector<uint8_t> data = inspire_hands_[i].encode(GetTemperature);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(GetTemperature, recv_data_vec))
                    {
                        cout << "Failed to get temperature" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when get temperature" << endl;
                }
            }
        }
        void getPosSet(uint8_t id)
        {
            uint8_t recv_len = 20;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    vector<uint8_t> data = inspire_hands_[i].encode(GetPosSet);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(GetPosSet, recv_data_vec))
                    {
                        cout << "Failed to get pos set" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when get pos set" << endl;
                }
            }
        }
        void getAngleSet(uint8_t id)
        {
            uint8_t recv_len = 20;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    vector<uint8_t> data = inspire_hands_[i].encode(GetAngleSet);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(GetAngleSet, recv_data_vec))
                    {
                        cout << "Failed to get angle set" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when get angle set" << endl;
                }
            }
        }
        void getForceSet(uint8_t id)
        {
            uint8_t recv_len = 20;
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    vector<uint8_t> data = inspire_hands_[i].encode(GetForceSet);
                    uint8_t send_data[data.size()];
                    for (int i = 0; i < data.size(); i++)
                    {
                        send_data[i] = data[i];
                    }
                    if (write(fd_, send_data, data.size()) < 0)
                    {
                        cout << "Failed to write data" << endl;
                        return;
                    }
                    uint8_t recv_data[recv_len];
                    if (read(fd_, recv_data, recv_len) <= 0)
                    {
                        cout << "Failed to read data from id: " << id << endl;
                        return;
                    }
                    vector<uint8_t> recv_data_vec(recv_len);
                    for (int i = 0; i < recv_len; i++)
                    {
                        recv_data_vec[i] = recv_data[i];
                    }
                    if (!inspire_hands_[i].decode(GetForceSet, recv_data_vec))
                    {
                        cout << "Failed to get force set" << endl;
                    }
                    break;
                }
                else
                {
                    cout << "Failed to find the id when get force set" << endl;
                }
            }
        }
        InspireHand::fdb *getInspireHand(uint8_t id)
        {
            for (int i = 0; i < inspire_hands_.size(); i++)
            {
                if (inspire_hands_[i].id_ == id)
                {
                    return &inspire_hands_[i].fdb_;
                }
                else
                {
                    cout << "Failed to find the id when get inspire hand" << endl;
                }
            }
        }

    private:
        // serial
        int fd_;
        int baudrate_;
        string port_;
        vector<InspireHand> inspire_hands_;
    };
}

#endif
