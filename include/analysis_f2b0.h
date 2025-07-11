
#ifndef _ANALYSIS_F2B0_H_
#define _ANALYSIS_F2B0_H_


#include <vector>

#include "f2b0_type.h"

using namespace std;
using namespace F2B0;

// 定义消息ID常量
const std::vector<uint8_t> NAV_TIME_ID = {0x01, 0x05};
const std::vector<uint8_t> NAV_TIMEUTC_ID = {0x01, 0x21};
const std::vector<uint8_t> NAV_CLOCK_ID = {0x01, 0x22};
const std::vector<uint8_t> NAV_CLOCK2_ID = {0x01, 0x23};
const std::vector<uint8_t> NAV_SVINFO_ID = {0x01, 0x30};
const std::vector<uint8_t> NAV_SVSTATE_ID = {0x01, 0x32};
const std::vector<uint8_t> NAV_PVT_ID = {0x01, 0xC1};
const std::vector<uint8_t> AID_EPH_BDS_ID = {0x0B, 0x33};    
const std::vector<uint8_t> AID_ALM_BDS_ID = {0x0B, 0x23};

// 结果类型定义
struct ParseResult {
    map<int, string> text_messages;
    map<int, NavTime> nav_time_messages;
    map<int, NavTimeUTC> nav_timeutc_messages;
    map<int, NavClock> nav_clock_messages;
    map<int, NavClock2> nav_clock2_messages;
    map<int, NavSvInfo> nav_svinfo_messages;
    map<int, NavSvState> nav_svstate_messages;
    map<int, NavPVT> nav_pvt_messages;
    map<int, PephBDS> aid_eph_bds_messages;
    map<int, PalmBDS> aid_alm_bds_messages;
    vector<string> errors;
};

class AnalysisF2B0
{
private:
public:
    AnalysisF2B0();
    NavTime parse_nav_time(const uint8_t* payload, size_t length);
    NavTimeUTC parse_nav_time_utc(const uint8_t* payload, size_t length);
    NavClock parse_nav_clock(const uint8_t* payload, size_t length);
    SatClk parse_sat_clk(const uint8_t* payload, size_t length);
    NavClock2 parse_nav_clock2(const uint8_t* payload, size_t length);
    SvInfo parse_sv_info(const uint8_t* payload, size_t length);
    NavSvInfo parse_nav_svinfo(const uint8_t* payload, size_t length);
    SvState parse_sv_state(const uint8_t* payload, size_t length);
    NavSvState parse_nav_svstate(const uint8_t* payload, size_t length);
    NavPVT parse_nav_pvt(const uint8_t* payload, size_t length);
    PephBDS parse_peph_bds(const uint8_t* payload, size_t length);
    PalmBDS parse_palm_bds(const uint8_t* payload, size_t length);
    ParseResult parse_mixed_file(const string& input_file, const string& output_file);
};


#endif  /* _ANALYSIS_F2B0_H_ */