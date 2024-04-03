#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/mission_raw/mission_raw.h>
#include <mavsdk/plugins/transponder/transponder.h>

#include <iostream>
#include <future>
#include <memory>
#include <thread>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;


void usage(const std::string& bin_name);
MissionRaw::MissionItem create_mission_item(uint32_t _seq, uint32_t _frame, uint32_t _command, uint32_t _current, uint32_t _autocontinue,
                                            float _param1, float _param2, float _param3, float _param4, 
                                            float _x, float _y, float _z, uint32_t _mission_type);

std::vector<MissionRaw::MissionItem> create_mission_raw();

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    Mavsdk mavsdk;
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

    // Instantiate plugins.
    auto telemetry = Telemetry{system.value()};
    auto action = Action{system.value()};
    auto mission_raw = MissionRaw{system.value()};

    // The plugin to set up parameters.
    auto param = Param{system.value()};

    // Instantiate Transponder plugin.
    auto transponder = Transponder{system.value()};

    // TODO: This param setting does not work after 5 retries.
    // param.set_param_int("RCx_OPTION", 38);

    // Enable Lua scripting
    param.set_param_int("SCR_ENABLE", 1);

    param.set_param_int("AVD_ENABLE", 1);
    param.set_param_int("AVD_F_DIST_XY", 10);
    param.set_param_int("AVD_F_DIST_Z", 10);
    param.set_param_int("AVD_F_TIME", 2);
    // Controls how the vehicle should respond to a projected near-miss 
    // (i.e. 2:Climb Or Descend, 3:Move Horizontally, 4:Move Perpendicularly in 3D, 5:RTL or 6:Hover)
    param.set_param_int("AVD_F_ACTION", 5);
    param.set_param_int("AVD_F_RCVRY", 1);

    // Set the number of simulated drones sending ADS-B signals in the SITL simulation.
    param.set_param_int("SIM_ADSB_COUNT", 10);
    // This param must be set to a non-zero value or ADS-B feature will be disabled.
    param.set_param_int("ADSB_TYPE", 1);
    
    // We want to listen to the transponder of the drone at 1 Hz.
    std::cout << "Setting transponder update rate\n";
    const Transponder::Result set_rate_result = transponder.set_rate_transponder(1.0);
    if (set_rate_result != Transponder::Result::Success) {
        std::cerr << "Setting rate failed:" << set_rate_result << '\n';
        return 1;
    }

    // Set up callback to monitor transponder activity
    std::cout << "Setting transponder subscription\n";
    transponder.subscribe_transponder([](Transponder::AdsbVehicle adsbVehicle) {
        std::cout << "ICAO Address: " << adsbVehicle.icao_address << '\n'
                  << "Latitude: " << adsbVehicle.latitude_deg << " deg\n"
                  << "Longitude: " << adsbVehicle.longitude_deg << " deg\n"
                  << "Absolute Altitude: " << adsbVehicle.absolute_altitude_m << " m\n"
                  << "Heading: " << adsbVehicle.heading_deg << " deg\n"
                  << "Horizontal Velocity: " << adsbVehicle.horizontal_velocity_m_s << " m/s\n"
                  << "Vertical Velocity: " << adsbVehicle.vertical_velocity_m_s << " m/s\n"
                  << "Call Sign: " << adsbVehicle.callsign << '\n'
                  << "Emitter Type: " << adsbVehicle.emitter_type << '\n'
                  << "Squawk: " << adsbVehicle.squawk << '\n';
    });

    // Search for aircraft transponders
    sleep_for(seconds(120));
    std::cout << "Finished...\n";

    return 0;

}

std::vector<MissionRaw::MissionItem> create_mission_raw()
{
    std::vector<MissionRaw::MissionItem> mission_raw_items;

    // _seq, _frame, _command, _current, _autocontinue, _param1, _param2, _param3, _param4, _x,  _y,  _z,  _mission_type
    // Add Home Position
    mission_raw_items.push_back(create_mission_item(0, 0, 16, 1, 1, 0, 0, 0, 0, 47.397742, 8.545594, 488, 0));

    // Add Takeoff
    mission_raw_items.push_back(create_mission_item(1, 3, 22, 0, 1, 0, 0, 0, 0, 0, 0,0 , 0));

    // Add Mission Item 2-3
    mission_raw_items.push_back(create_mission_item(2, 3, 16, 0, 1, 0, 0, 0, 0, 47.39776911820642, 8.545794816614517, 30, 0));
    mission_raw_items.push_back(create_mission_item(3, 3, 16, 0, 1, 0, 0, 0, 0, 47.39814478901126, 8.544659618054993, 30, 0));

    // Return to Launch
    mission_raw_items.push_back(create_mission_item(4, 3, 20, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0));
    return mission_raw_items;
}

MissionRaw::MissionItem create_mission_item(uint32_t _seq, uint32_t _frame, uint32_t _command, uint32_t _current, uint32_t _autocontinue,
                                            float _param1, float _param2, float _param3, float _param4, 
                                            float _x, float _y, float _z, 
                                            uint32_t _mission_type)
{
    MissionRaw::MissionItem new_raw_item_nav{};
    new_raw_item_nav.seq = _seq;
    new_raw_item_nav.frame = _frame; // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT 
    new_raw_item_nav.command = _command; // MAV_CMD_NAV_WAYPOINT
    new_raw_item_nav.current = _current;
    new_raw_item_nav.autocontinue = _autocontinue;
    new_raw_item_nav.param1 = _param1; // Hold
    new_raw_item_nav.param2 = _param2; // Accept Radius
    new_raw_item_nav.param3 = _param3; // Pass Radius
    new_raw_item_nav.param4 = _param4; // Yaw
    new_raw_item_nav.x = int32_t(std::round(_x * 1e7));
    new_raw_item_nav.y = int32_t(std::round(_y * 1e7));
    new_raw_item_nav.z = _z;
    new_raw_item_nav.mission_type = 0;
    return new_raw_item_nav;
}
