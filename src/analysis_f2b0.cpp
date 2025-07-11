
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <map>
#include <cstdint>
#include <cmath>
#include <cstring>
#include "analysis_f2b0.h"

using namespace std;
using namespace F2B0;



// 解析函数
NavTime AnalysisF2B0::parse_nav_time(const uint8_t* payload, size_t length) {
    if (length != 16) {
        throw std::runtime_error("Invalid payload length for NAV-TIME");
    }

    NavTime result;
    memcpy(&result.nav_sys, payload, 1);
    memcpy(&result.flag, payload + 1, 1);
    memcpy(&result.fractow, payload + 2, 2);
    memcpy(&result.ref_tow, payload + 4, 4);
    memcpy(&result.week, payload + 8, 2);
    memcpy(&result.leap_sec, payload + 10, 2);
    memcpy(&result.time_err, payload + 12, 4);

    return result;
}

NavClock AnalysisF2B0::parse_nav_clock(const uint8_t* payload, size_t length) {
    if (length != 20) {
        throw std::runtime_error("Invalid payload length for NAV-CLOCK");
    }

    NavClock result;
    memcpy(&result.iTow, payload, 4);
    memcpy(&result.clkB, payload + 4, 4);
    memcpy(&result.clkD, payload + 8, 4);
    memcpy(&result.tAcc, payload + 12, 4);
    memcpy(&result.fAcc, payload + 16, 4);

    return result;
}

NavTimeUTC AnalysisF2B0::parse_nav_time_utc(const uint8_t* payload, size_t length) {
    if (length != 20) {
        throw std::runtime_error("Invalid payload length for NAV-TIMEUTC");
    }

    NavTimeUTC result;
    memcpy(&result.iTow, payload, 4);
    memcpy(&result.tAcc, payload + 4, 4);
    memcpy(&result.nano, payload + 8, 4);
    memcpy(&result.year, payload + 12, 2);
    memcpy(&result.month, payload + 14, 1);
    memcpy(&result.day, payload + 15, 1);
    memcpy(&result.hour, payload + 16, 1);
    memcpy(&result.minute, payload + 17, 1);
    memcpy(&result.second, payload + 18, 1);
    memcpy(&result.valid, payload + 19, 1);

    return result;
}

NavSvInfo AnalysisF2B0::parse_nav_svinfo(const uint8_t* payload, size_t length) {
    if (length < 8 || (length - 8) % 24 != 0) {
        throw std::runtime_error("Invalid payload length for NAV-SVINFO");
    }

    NavSvInfo result;
    memcpy(&result.iTow, payload, 4);
    memcpy(&result.numCh, payload + 4, 4);

    size_t numSv = (length - 8) / 24;
    for (size_t i = 0; i < numSv; ++i) {
        const uint8_t* svData = payload + 8 + i * 24;
        SvInfo sv;
        memcpy(&sv.svid, svData, 2);
        memcpy(&sv.flags, svData + 2, 1);
        memcpy(&sv.quality, svData + 3, 1);
        memcpy(&sv.cno, svData + 4, 1);
        memcpy(&sv.elev, svData + 5, 1);
        memcpy(&sv.azim, svData + 6, 1);
        memcpy(&sv.prRes, svData + 7, 2);
        memcpy(&sv.pseudorangeRate, svData + 9, 4);
        memcpy(&sv.pseudorange, svData + 13, 8);
        
        result.svInfo[i] = sv;
    }

    return result;
}

NavClock2 AnalysisF2B0::parse_nav_clock2(const uint8_t* payload, size_t length) {
    if (length < 8 || (length - 8) % 12 != 0) {
        throw std::runtime_error("Invalid payload length for NAV-CLOCK2");
    }

    NavClock2 result;
    memcpy(&result.iTow, payload, 4);
    memcpy(&result.numClk, payload + 4, 4);

    size_t numClk = (length - 8) / 12;
    for (size_t i = 0; i < numClk; ++i) {
        const uint8_t* clkData = payload + 8 + i * 12;
        SatClk cd;
        memcpy(&cd.sysmask, clkData, 4);
        memcpy(&cd.clkB, clkData + 4, 4);
        memcpy(&cd.tAcc, clkData + 8, 4);
        
        result.satClk[i] = cd;
    }

    return result;
}

NavSvState AnalysisF2B0::parse_nav_svstate(const uint8_t* payload, size_t length) {
    if (length < 8 || (length - 8) % 4 != 0) {
        throw std::runtime_error("Invalid payload length for NAV-SVSTATE");
    }

    NavSvState result;
    uint32_t numSV_rev;
    memcpy(&result.iTow, payload, 4);
    memcpy(&numSV_rev, payload + 4, 4);
    
    result.numSv = numSV_rev & 0xFFFFFF;
    result.rev = (numSV_rev >> 24) & 0xFF;

    size_t numSv = (length - 8) / 4;
    for (size_t i = 0; i < numSv; ++i) {
        const uint8_t* svData = payload + 8 + i * 4;
        SvState sv;
        memcpy(&sv.svid, svData, 2);
        memcpy(&sv.eph_state, svData + 2, 1);
        memcpy(&sv.alm_state, svData + 3, 1);
        
        result.svState[i] = sv;
    }

    return result;
}

NavPVT AnalysisF2B0::parse_nav_pvt(const uint8_t* payload, size_t length) {
    if (length != 84) {
        throw std::runtime_error("Invalid payload length for NAV-PVT");
    }

    NavPVT result;
    memcpy(&result.iTow, payload, 4);
    memcpy(&result.year, payload + 4, 2);
    memcpy(&result.month, payload + 6, 1);
    memcpy(&result.day, payload + 7, 1);
    memcpy(&result.hour, payload + 8, 1);
    memcpy(&result.minute, payload + 9, 1);
    memcpy(&result.second, payload + 10, 1);
    memcpy(&result.valid, payload + 11, 1);
    memcpy(&result.tAcc, payload + 12, 4);
    memcpy(&result.nano, payload + 16, 4);
    memcpy(&result.fixType, payload + 20, 1);
    memcpy(&result.numSV, payload + 23, 1);
    
    int32_t lon, lat;
    memcpy(&lon, payload + 24, 4);
    memcpy(&lat, payload + 28, 4);
    result.lon = lon * 1e-7;
    result.lat = lat * 1e-7;
    
    memcpy(&result.height, payload + 32, 4);
    memcpy(&result.hMSL, payload + 36, 4);
    memcpy(&result.hAcc, payload + 40, 4);
    memcpy(&result.vAcc, payload + 44, 4);
    memcpy(&result.velN, payload + 48, 4);
    memcpy(&result.velE, payload + 52, 4);
    memcpy(&result.velD, payload + 56, 4);
    memcpy(&result.gSpeed, payload + 60, 4);
    
    int32_t headMot;
    memcpy(&headMot, payload + 64, 4);
    result.headMot = headMot * 1e-5;
    
    memcpy(&result.sAcc, payload + 68, 4);
    
    uint32_t headAcc;
    memcpy(&headAcc, payload + 72, 4);
    result.headAcc = headAcc * 1e-5;
    
    uint16_t pDop;
    memcpy(&pDop, payload + 76, 2);
    result.pDOP = pDop * 0.01;
    
    int32_t headVeh;
    memcpy(&headVeh, payload + 80, 4);
    result.headVeh = headVeh * 1e-5;

    return result;
}

PephBDS AnalysisF2B0::parse_peph_bds(const uint8_t* payload, size_t length) {
    if (length != 92) {
        throw std::runtime_error("Invalid payload length for BeiDou Ephemeris");
    }

    PephBDS result;
    memcpy(&result.svid, payload + 1, 1);
    
    uint32_t sqrtA_raw;
    memcpy(&sqrtA_raw, payload + 2, 4);
    result.sqrtA = sqrtA_raw * pow(2, -19);
    
    uint32_t e_raw;
    memcpy(&e_raw, payload + 6, 4);
    result.e = e_raw * pow(2, -33);
    
    int32_t M0_raw;
    memcpy(&M0_raw, payload + 10, 4);
    result.M0 = M0_raw * pow(2, -31) * M_PI;
    
    int16_t Delta_n_raw;
    memcpy(&Delta_n_raw, payload + 14, 2);
    result.DeltaN = Delta_n_raw * pow(2, -43) * M_PI;
    
    int32_t toe_raw;
    memcpy(&toe_raw, payload + 16, 4);
    result.toe = toe_raw * pow(2, 3);
    
    int32_t i0_raw;
    memcpy(&i0_raw, payload + 20, 4);
    result.i0 = i0_raw * pow(2, -31) * M_PI;
    
    int32_t iDet_raw;
    memcpy(&iDet_raw, payload + 24, 4);
    result.iDot = iDet_raw * pow(2, -43) * M_PI;
    
    int32_t Omega0_raw;
    memcpy(&Omega0_raw, payload + 28, 4);
    result.Omega0 = Omega0_raw * pow(2, -31) * M_PI;
    
    int32_t OmegaDet_raw;
    memcpy(&OmegaDet_raw, payload + 32, 4);
    result.OmegaDot = OmegaDet_raw * pow(2, -43) * M_PI;
    
    int32_t w_raw;
    memcpy(&w_raw, payload + 36, 4);
    result.w = w_raw * pow(2, -31) * M_PI;
    
    int32_t Cuc_raw;
    memcpy(&Cuc_raw, payload + 40, 4);
    result.Cuc = Cuc_raw * pow(2, -31);
    
    int32_t Cus_raw;
    memcpy(&Cus_raw, payload + 44, 4);
    result.Cus = Cus_raw * pow(2, -31);
    
    int32_t Crc_raw;
    memcpy(&Crc_raw, payload + 48, 4);
    result.Crc = Crc_raw * pow(2, -6);
    
    int32_t Crs_raw;
    memcpy(&Crs_raw, payload + 52, 4);
    result.Crs = Crs_raw * pow(2, -6);
    
    int32_t Cic_raw;
    memcpy(&Cic_raw, payload + 56, 4);
    result.Cic = Cic_raw * pow(2, -31);
    
    int32_t Cis_raw;
    memcpy(&Cis_raw, payload + 60, 4);
    result.Cis = Cis_raw * pow(2, -31);
    
    int32_t toe_clk_raw;
    memcpy(&toe_clk_raw, payload + 64, 4);
    result.toe = toe_clk_raw * pow(2, 3);
    
    int32_t afl_raw;
    memcpy(&afl_raw, payload + 68, 4);
    result.af0 = afl_raw * pow(2, -33);
    
    int32_t af1_raw;
    memcpy(&af1_raw, payload + 72, 4);
    result.af1 = af1_raw * pow(2, -50);
    
    int32_t af2_raw;
    memcpy(&af2_raw, payload + 76, 4);
    result.af2 = af2_raw * pow(2, -66);
    
    int16_t tGD_raw;
    memcpy(&tGD_raw, payload + 80, 2);
    result.tGD = tGD_raw * 0.1;
    
    int16_t Alpha0_raw;
    memcpy(&Alpha0_raw, payload + 82, 2);
    result.Alpha0 = Alpha0_raw * pow(2, -30);
    
    int8_t Alpha1_raw;
    memcpy(&Alpha1_raw, payload + 84, 1);
    result.Alpha1 = Alpha1_raw * pow(2, -27);
    
    int8_t Alpha2_raw;
    memcpy(&Alpha2_raw, payload + 85, 1);
    result.Alpha2 = Alpha2_raw * pow(2, -24);
    
    int8_t Alpha3_raw;
    memcpy(&Alpha3_raw, payload + 86, 1);
    result.Alpha3 = Alpha3_raw * pow(2, -24);
    
    int16_t Beta0_raw;
    memcpy(&Beta0_raw, payload + 87, 2);
    result.Beta0 = Beta0_raw * pow(2, 11);
    
    int16_t Beta1_raw;
    memcpy(&Beta1_raw, payload + 89, 2);
    result.Beta1 = Beta1_raw * pow(2, 14);
    
    uint16_t weeknum_raw;
    memcpy(&weeknum_raw, payload + 91, 2);
    result.weeknum = weeknum_raw;
    
    uint16_t IODC_raw;
    memcpy(&IODC_raw, payload + 93, 2);
    result.IODC = IODC_raw;
    
    uint8_t IODE_raw;
    memcpy(&IODE_raw, payload + 95, 1);
    result.IODE = IODE_raw;
    
    uint8_t ura_raw;
    memcpy(&ura_raw, payload + 96, 1);
    result.ura = ura_raw;
    
    uint8_t health_raw;
    memcpy(&health_raw, payload + 97, 1);
    result.health = health_raw;

    return result;
}

PalmBDS AnalysisF2B0::parse_palm_bds(const uint8_t* payload, size_t length) {
    if (length != 35) {
        throw std::runtime_error("Invalid payload length for BeiDou Almanac");
    }

    PalmBDS result;
    memcpy(&result.svid, payload, 1);
    
    uint8_t toa_raw;
    memcpy(&toa_raw, payload + 1, 1);
    result.toa = toa_raw * 4096;
    
    uint16_t health_raw;
    memcpy(&health_raw, payload + 2, 2);
    result.health = health_raw;
    
    uint32_t sqrtA_raw;
    memcpy(&sqrtA_raw, payload + 4, 4);
    result.sqrtA = sqrtA_raw * pow(2, -11);
    
    uint32_t e_raw;
    memcpy(&e_raw, payload + 8, 4);
    result.e = e_raw * pow(2, -21);
    
    
    int32_t w_raw;
    memcpy(&w_raw, payload + 24, 4);
    result.w = w_raw * pow(2, -23) * M_PI;
    
    int32_t di_raw;
    memcpy(&di_raw, payload + 28, 4);
    result.di = di_raw * pow(2, -19) * M_PI;
    
    uint8_t WN_raw;
    memcpy(&WN_raw, payload + 40, 1);
    result.WN = WN_raw;

    return result;
}


ParseResult AnalysisF2B0::parse_mixed_file(const string& input_file, const string& output_file) {
    ParseResult result;
    ifstream fin(input_file, ios::binary);
    ofstream fout(output_file);
    
    if (!fin.is_open()) {
        result.errors.push_back("无法打开输入文件: " + input_file);
        return result;
    }
    
    if (!fout.is_open()) {
        result.errors.push_back("无法打开输出文件: " + output_file);
        return result;
    }

    int seq = 0;
    while (fin) {
        // 读取消息头
        uint8_t header[2];
        fin.read(reinterpret_cast<char*>(header), 2);
        
        if (fin.gcount() < 2) {
            break;  // 文件结束
        }

        // 检查是否为二进制消息 (0xF1 0xD9)
        if (header[0] == 0xF1 && header[1] == 0xD9) {
            // 读取消息ID
            uint8_t msg_id[2];
            fin.read(reinterpret_cast<char*>(msg_id), 2);
            
            if (fin.gcount() < 2) {
                result.errors.push_back("二进制消息头不完整");
                continue;
            }

            // 读取消息长度 (小端)
            uint16_t length;
            fin.read(reinterpret_cast<char*>(&length), 2);
            
            if (fin.gcount() < 2) {
                result.errors.push_back("无法读取消息长度");
                continue;
            }

            // 读取消息体
            vector<uint8_t> payload(length);
            fin.read(reinterpret_cast<char*>(payload.data()), length);
            
            if (static_cast<size_t>(fin.gcount()) < length) {
                result.errors.push_back("消息体不完整");
                continue;
            }

            try {
                // 根据消息ID调用相应的解析函数
                if (msg_id[0] == NAV_TIME_ID[0] && msg_id[1] == NAV_TIME_ID[1]) {
                    if (length != 16) {
                        result.errors.push_back("NAV-TIME消息长度错误: " + to_string(length));
                        continue;
                    }
                    auto nav_time = parse_nav_time(payload.data(), length);
                    result.nav_time_messages[seq] = nav_time;
                    fout << "NAV-TIME: " << nav_time.ref_tow << endl;
                    seq++;
                }
                else if (msg_id[0] == NAV_TIMEUTC_ID[0] && msg_id[1] == NAV_TIMEUTC_ID[1]) {
                    if (length != 20) {
                        result.errors.push_back("NAV-TIMEUTC消息长度错误: " + to_string(length));
                        continue;
                    }
                    auto nav_timeutc = parse_nav_time_utc(payload.data(), length);
                    result.nav_timeutc_messages[seq] = nav_timeutc;
                    fout << "NAV-TIMEUTC: " << nav_timeutc.iTow << endl;
                    seq++;
                }
                else if (msg_id[0] == NAV_CLOCK_ID[0] && msg_id[1] == NAV_CLOCK_ID[1]) {
                    if (length != 20) {
                        result.errors.push_back("NAV-CLOCK消息长度错误: " + to_string(length));
                        continue;
                    }
                    auto nav_clock = parse_nav_clock(payload.data(), length);
                    result.nav_clock_messages[seq] = nav_clock;
                    fout << "NAV-CLOCK: " << nav_clock.iTow << endl;
                    seq++;
                }
                else if (msg_id[0] == NAV_CLOCK2_ID[0] && msg_id[1] == NAV_CLOCK2_ID[1]) {
                    if (length < 8 || (length - 8) % 12 != 0) {
                        result.errors.push_back("NAV-CLOCK2消息长度错误: " + to_string(length));
                        continue;
                    }
                    auto nav_clock2 = parse_nav_clock2(payload.data(), length);
                    result.nav_clock2_messages[seq] = nav_clock2;
                    fout << "NAV-CLOCK2: " << nav_clock2.iTow << endl;
                    seq++;
                }
                else if (msg_id[0] == NAV_SVINFO_ID[0] && msg_id[1] == NAV_SVINFO_ID[1]) {
                    if (length < 8 || (length - 8) % 24 != 0) {
                        result.errors.push_back("NAV-SVINFO消息长度错误: " + to_string(length));
                        continue;
                    }
                    auto nav_svinfo = parse_nav_svinfo(payload.data(), length);
                    result.nav_svinfo_messages[seq] = nav_svinfo;
                    fout << "NAV-SVINFO: " << nav_svinfo.iTow << endl;
                    seq++;
                }
                else if (msg_id[0] == NAV_SVSTATE_ID[0] && msg_id[1] == NAV_SVSTATE_ID[1]) {
                    if (length < 8 || (length - 8) % 4 != 0) {
                        result.errors.push_back("NAV-SVSTATE消息长度错误: " + to_string(length));
                        continue;
                    }
                    auto nav_svstate = parse_nav_svstate(payload.data(), length);
                    result.nav_svstate_messages[seq] = nav_svstate;
                    fout << "NAV-SVSTATE: " << nav_svstate.iTow << endl;
                    seq++;
                }
                else if (msg_id[0] == NAV_PVT_ID[0] && msg_id[1] == NAV_PVT_ID[1]) {
                    if (length != 84) {
                        result.errors.push_back("NAV-PVT消息长度错误: " + to_string(length));
                        continue;
                    }
                    auto nav_pvt = parse_nav_pvt(payload.data(), length);
                    result.nav_pvt_messages[seq] = nav_pvt;
                    fout << "NAV-PVT: " << nav_pvt.iTow << endl;
                    seq++;
                }
                else if (msg_id[0] == AID_EPH_BDS_ID[0] && msg_id[1] == AID_EPH_BDS_ID[1]) {
                    if (length != 92) {
                        result.errors.push_back("AID-EPH-BDS消息长度错误: " + to_string(length));
                        continue;
                    }
                    auto aid_eph_bds = parse_peph_bds(payload.data(), length);
                    result.aid_eph_bds_messages[seq] = aid_eph_bds;
                    fout << "AID-EPH-BDS: " << aid_eph_bds.svid << endl;
                    seq++;
                }
                else if (msg_id[0] == AID_ALM_BDS_ID[0] && msg_id[1] == AID_ALM_BDS_ID[1]) {
                    if (length != 35) {
                        result.errors.push_back("AID-ALM-BDS消息长度错误: " + to_string(length));
                        continue;
                    }
                    auto aid_alm_bds = parse_palm_bds(payload.data(), length);
                    result.aid_alm_bds_messages[seq] = aid_alm_bds;
                    fout << "AID-ALM-BDS: " << aid_alm_bds.svid << endl;
                    seq++;
                }
                else {
                    stringstream ss;
                    ss << "未知消息ID: 0x" << hex << setw(2) << setfill('0') 
                       << static_cast<int>(msg_id[0]) << " 0x" 
                       << static_cast<int>(msg_id[1]);
                    result.errors.push_back(ss.str());
                    continue;
                }
            } catch (const exception& e) {
                result.errors.push_back(string("解析错误: ") + e.what());
                continue;
            }
        }
        // 检查是否为NMEA文本消息 ($B或$G开头)
        else if (header[0] == '$' && (header[1] == 'B' || header[1] == 'G')) {
            string line;
            line += static_cast<char>(header[0]);
            line += static_cast<char>(header[1]);
            
            // 读取行剩余部分直到换行符
            getline(fin, line);
            
            result.text_messages[seq] = line;
            fout << line << endl;
            seq++;
        }
        // 其他情况跳过
        else {
            // 可以记录未知头信息
            stringstream ss;
            ss << "未知头信息: 0x" << hex << setw(2) << setfill('0') 
               << static_cast<int>(header[0]) << " 0x" 
               << static_cast<int>(header[1]);
            result.errors.push_back(ss.str());
        }
    }

    fin.close();
    fout.close();
    
    return result;
}

// 辅助函数：将ParseResult写入文本文件
void write_result_to_txtfile(const ParseResult& result, const string& outfile) {
    ofstream fout(outfile);
    if (!fout.is_open()) {
        cerr << "无法打开输出文件: " << outfile << endl;
        return;
    }

    // 写入文本消息
    for (const auto& [seq, msg] : result.text_messages) {
        fout << seq << ", " << msg << endl;
    }

    // 写入NAV-TIME消息
    for (const auto& [seq, msg] : result.nav_time_messages) {
        fout << seq << ", NAV-TIME: " << msg.ref_tow << endl;
    }

    // 写入其他消息类型...
    // 可以根据需要添加更多消息类型的输出

    // 写入错误信息
    if (!result.errors.empty()) {
        fout << "\n=== 错误信息 ===\n";
        for (const auto& err : result.errors) {
            fout << err << endl;
        }
    }

    fout.close();
}