//
// Demonstrates how to Add & Upload geofence using MAVSDK.
// The example is summarised below:
// 1. Adds points to geofence.
// 2. Uploads the geofence mission.
//

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/geofence/geofence.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/mission_raw/mission_raw.h>

#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <thread>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

static Geofence::Point add_point(double latitude_deg, double longitude_deg);
static std::vector<MissionRaw::MissionItem> create_mission_plan();
static MissionRaw::MissionItem create_mission_item(
    uint32_t _seq, uint32_t _frame, uint32_t _command, uint32_t _current, uint32_t _autocontinue,
    float _param1, float _param2, float _param3, float _param4, 
    float _x, float _y, float _z, 
    uint32_t _mission_type);

bool offb_ctrl_pos_global(mavsdk::Offboard& offboard, mavsdk::Telemetry& telemetry);

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

    // For both Geofence MAVLink: warning: Mode change to GUIDED failed: requires position (system_impl.cpp:248)
    auto offboard = Offboard{system.value()};
    auto geofence = Geofence{system.value()};

    sleep_for(seconds(5));
    // offb_ctrl_pos_global(offboard, telemetry);

    telemetry.subscribe_flight_mode([](Telemetry::FlightMode flight_mode) {
        std::cout << "Flight mode: " << flight_mode << '\n';
    });

    // Upload Mission before arming or the takeoff will fail
    auto mission_plan = create_mission_plan();
    const MissionRaw::Result _uploadResult = mission_raw.upload_mission(mission_plan);
    if (_uploadResult != MissionRaw::Result::Success) {
        std::cerr << "Upload Mission failed: " << _uploadResult << '\n';
        return 0;
    }
    else {
        std::cout << "Upload Mission Succeed." << "\n";
    }

    // Arm and Takeoff, this is needed in APM before mission can be started.
    std::cout << "Setting takeoff altitude\n";
    action.set_takeoff_altitude(50.0f);

    // Arming the Vehicle.
    const Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cout << "Arming Result: " << arm_result << '\n';
        return 1;
    }
    
    // Taking Off
    std::cout << "Taking off...\n";
    const Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
        return 1;
    }

    sleep_for(seconds(5));

    // This part is not needed for APM and not working.
    // while (!telemetry.health_all_ok()) {
    //     std::cout << "Waiting for system to be ready\n";
    //     sleep_for(seconds(1));
    // }

    std::cout << "System ready\n";
    std::cout << "Creating and uploading geofences\n";

    std::vector<Geofence::Polygon> polygons;
    std::vector<Geofence::Circle> circles;

    // Polygon inclusion
    {
        std::vector<Geofence::Point> points;
        points.emplace_back(add_point(47.40029181357694, 8.540208324993841));
        points.emplace_back(add_point(47.400173434578114, 8.547689686153717));
        points.emplace_back(add_point(47.39677781722575, 8.547444838745832));
        points.emplace_back(add_point(47.39709351430107, 8.539753608416305));
        Geofence::Polygon new_polygon{};
        new_polygon.fence_type = Geofence::FenceType::Inclusion;
        new_polygon.points = points;
        polygons.push_back(new_polygon);
    }

    // Polygon exclusion
    {
        std::vector<Geofence::Point> points;
        points.emplace_back(add_point(47.39845295869903, 8.543682820794851));
        points.emplace_back(add_point(47.39837403681116, 8.545066315854541));
        points.emplace_back(add_point(47.39759073790796, 8.544926413920791));
        points.emplace_back(add_point(47.39762230688655, 8.543531259263398));
        Geofence::Polygon new_polygon{};
        new_polygon.fence_type = Geofence::FenceType::Exclusion;
        new_polygon.points = points;
        polygons.push_back(new_polygon);
    }

    // Circle inclusion
    {
        Geofence::Point center = add_point(47.39867310982157, 8.54379160368353);
        Geofence::Circle new_circle{};
        new_circle.point = center;
        new_circle.fence_type = Geofence::FenceType::Inclusion;
        new_circle.radius = 319.54F;
        circles.push_back(new_circle);
    }

    // Circle exclusion
    {
        Geofence::Point center = add_point(47.39928080314044, 8.54540060087578);
        Geofence::Circle new_circle{};
        new_circle.point = center;
        new_circle.fence_type = Geofence::FenceType::Exclusion;
        new_circle.radius = 49.52F;
        circles.push_back(new_circle);
    }

    Geofence::GeofenceData geofence_data{};
    geofence_data.polygons = polygons;
    geofence_data.circles = circles;

    std::cout << "Uploading geofence...\n";

    const Geofence::Result result = geofence.upload_geofence(geofence_data);

    if (result != Geofence::Result::Success) {
        std::cerr << "Geofence upload failed: " << result << ", exiting.\n";
        return 1;
    }
    std::cout << "Geofence uploaded.\n";


    // Mission Start
    std::cout << "Starting Mission \n ";
    const MissionRaw::Result _startResult = mission_raw.start_mission();
    if (_startResult != MissionRaw::Result::Success) {
        std::cerr << "Starting mission failed: " << _startResult << '\n';
        return 1;
    }
    else {
        std::cout << "Start Mission Succeeded." << "\n";
    }

    // Subscribe to mission progress.
    auto prom = std::promise<void>();
    auto fut = prom.get_future();
    mission_raw.subscribe_mission_progress([&](MissionRaw::MissionProgress progress) {
        std::cout << "Progress: " << progress.current << "/" << progress.total;
        if (progress.current == progress.total) {
            mission_raw.subscribe_mission_progress(nullptr);
            prom.set_value();
        }
    });

     telemetry.subscribe_position([](Telemetry::Position position) {
        std::cout << "Position: " << position << " m\n";
    });

    fut.wait_for(std::chrono::seconds(120)) == std::future_status::ready;
    fut.get();

    return 0;
}

std::vector<MissionRaw::MissionItem> create_mission_plan() {
    uint32_t command = MAV_CMD_NAV_WAYPOINT; 	//213 (MAV_CMD_DO_SET_POSITION_YAW_THRUST)
    uint32_t frame = 0; // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT cause invalid argument
    uint32_t current = 1;  // Simon set it to 2 and it is invalid.
    uint32_t autocontinue = 1; // Simon set it to 0.
    float param1 = 10;	//angle (centridegree) [-4500 - 4500]
	float param2 = 10;	//angle (centridegree) [-4500 - 4500]
	float param3 = 0;			 	//angle (centridegree) [-4500 - 4500]
	float param4 = 0;			 	//angle (centridegree) [-4500 - 4500]
    // The type should be float? 
    // int32_t x = (int) (-35.313553 * 10000000.0);	 //speed normalized [0 - 1]
    // int32_t y = (int) (149.162057 * 10000000.0);	 //speed normalized [0 - 1]
    float x = -35.313553; // Remove or error occurs. * 10000000.0;	 //speed normalized [0 - 1]
    float y = 149.162057; // Remove or error occurs. * 10000000.0;	 //speed normalized [0 - 1]
    float z = 20;	

    std::vector<MissionRaw::MissionItem> mission_raw_items;
    auto item1 = create_mission_item(0, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z, 0);
    mission_raw_items.push_back(item1);

    // mission_raw_items.push_back(create_mission_item(0, 0, 16, 1, 1, 0, 0, 0, 0, 47.397742, 8.545594, 488, 0));

    // Add Takeoff
    mission_raw_items.push_back(create_mission_item(1, 3, 22, 0, 1, 0, 0, 0, 0, 0, 0,0 , 0));

    // Add Mission Item 2-3
    mission_raw_items.push_back(create_mission_item(2, 3, 16, 0, 1, 0, 0, 0, 0, 47.39776911820642, 8.545794816614517, 30, 0));
    mission_raw_items.push_back(create_mission_item(3, 3, 16, 0, 1, 0, 0, 0, 0, 47.39814478901126, 8.544659618054993, 30, 0));

    // Return to Launch
    mission_raw_items.push_back(create_mission_item(4, 3, 20, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0));
    return mission_raw_items;
}

MissionRaw::MissionItem create_mission_item(
    uint32_t _seq, uint32_t _frame, uint32_t _command, uint32_t _current, uint32_t _autocontinue,
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

Geofence::Point add_point(double latitude_deg, double longitude_deg)
{
    Geofence::Point new_point;
    new_point.latitude_deg = latitude_deg;
    new_point.longitude_deg = longitude_deg;
    return new_point;
}

//
// Does Offboard control using Global (Latitude, Longitude, relative altitude) co-ordinates.
//
// returns true if everything went well in Offboard control
//
bool offb_ctrl_pos_global(mavsdk::Offboard& offboard, mavsdk::Telemetry& telemetry)
{
    std::cout << "Reading home position in Global coordinates\n";

    const auto res_and_gps_origin = telemetry.get_gps_global_origin();
    if (res_and_gps_origin.first != Telemetry::Result::Success) {
        std::cerr << "Telemetry failed: " << res_and_gps_origin.first << '\n';
    }
    Telemetry::GpsGlobalOrigin origin = res_and_gps_origin.second;
    std::cerr << "Origin (lat, lon, alt amsl):\n " << origin << '\n';

    std::cout << "Starting Offboard position control in Global coordinates\n";

    // Send it once before starting offboard, otherwise it will be rejected.
    // this is a step north about 10m, using the default altitude type (altitude relative to home)
    const Offboard::PositionGlobalYaw north{
        origin.latitude_deg + 0.0001, origin.longitude_deg, 20.0f, 0.0f};
    offboard.set_position_global(north);

    Offboard::Result offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << '\n';
        return false;
    }

    std::cout << "Offboard started\n";
    std::cout << "Going North at 20m relative altitude\n";
    sleep_for(seconds(10));

    // here we use an explicit altitude type (relative to home)
    const Offboard::PositionGlobalYaw east{
        origin.latitude_deg + 0.0001,
        origin.longitude_deg + 0.0001,
        15.0f,
        90.0f,
        Offboard::PositionGlobalYaw::AltitudeType::RelHome};
    offboard.set_position_global(east);
    std::cout << "Going East at 15m relative altitude\n";
    sleep_for(seconds(10));

    // here we use the above mean sea level altitude
    const Offboard::PositionGlobalYaw home{
        origin.latitude_deg,
        origin.longitude_deg,
        origin.altitude_m + 10.0f,
        180.0f,
        Offboard::PositionGlobalYaw::AltitudeType::Amsl};
    offboard.set_position_global(home);
    std::cout << "Going Home facing south at " << (origin.altitude_m + 10.0f)
              << "m AMSL altitude\n";
    sleep_for(seconds(10));

    offboard_result = offboard.stop();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard stop failed: " << offboard_result << '\n';
        return false;
    }
    std::cout << "Offboard stopped\n";

    return true;
}