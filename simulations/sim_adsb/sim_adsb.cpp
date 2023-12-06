#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <time.h>
#include <unistd.h>
#include <mavlink/ardupilotmega/mavlink.h>


void receive_some(int socket_fd, struct sockaddr_in* src_addr, socklen_t* src_addr_len, bool* src_addr_set);
void send_some(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void initial_command(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);
void test_send_command(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len);


int radius = 500;
int altitude = 20;

int main(int argc, char* argv[])
{
    // Open UDP socket
    const int socket_fd = socket(PF_INET, SOCK_DGRAM, 0);

    if (socket_fd < 0) {
        printf("socket error: %s\n", strerror(errno));
        return -1;
    }

    // Bind to port
    struct sockaddr_in addr = {};
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, "0.0.0.0", &(addr.sin_addr)); // listen on all network interfaces
    addr.sin_port = htons(14550); // default port on the ground

    if (bind(socket_fd, (struct sockaddr*)(&addr), sizeof(addr)) != 0) {
        printf("bind error: %s\n", strerror(errno));
        return -2;
    }
    printf("binded");
    // We set a timeout at 100ms to prevent being stuck in recvfrom for too
    // long and missing our chance to send some stuff.
    // struct timeval tv;
    // tv.tv_sec = 0;
    // tv.tv_usec = 100000;
    // if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    //     printf("setsockopt error: %s\n", strerror(errno));
    //     return -3;
    // }

    struct sockaddr_in src_addr = {};
    socklen_t src_addr_len = sizeof(src_addr);
    bool src_addr_set = false;

    int init_cmd_req_count = 1;
    while (init_cmd_req_count > 0) {
        receive_some(socket_fd, &src_addr, &src_addr_len, &src_addr_set);
        // command that only needs to be executed once
        initial_command(socket_fd, &src_addr, src_addr_len);
        // sleep(1);
        init_cmd_req_count = init_cmd_req_count - 1;
    }

    while (true) {
        // For illustration purposes we don't bother with threads or async here
        // and just interleave receiving and sending.
        // This only works  if receive_some returns every now and then.
        receive_some(socket_fd, &src_addr, &src_addr_len, &src_addr_set);

        if (src_addr_set) {
            send_some(socket_fd, &src_addr, src_addr_len);
        }
    }

    return 0;
}


void receive_some(int socket_fd, struct sockaddr_in* src_addr, socklen_t* src_addr_len, bool* src_addr_set)
{
    // printf("Receiving ...\n");
    // We just receive one UDP datagram and then return again.
    char buffer[2048]; // enough for MTU 1500 bytes

    const int ret = recvfrom(
            socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)(src_addr), src_addr_len);

    if (ret < 0) {
        printf("recvfrom error: %s\n", strerror(errno));
    } else if (ret == 0) {
        // peer has done an orderly shutdown
        return;
    } 

    *src_addr_set = true;

    mavlink_message_t message;
    mavlink_status_t status;
    for (int i = 0; i < ret; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status) == 1) {

            switch (message.msgid) {
                default:
                    break;
            }

        }
    }
}

// https://mavlink.io/en/messages/common.html#FENCE_BREACH
void handle_fence_breach(const mavlink_message_t* message)
{
    mavlink_fence_status_t fence_status;
    // mavlink_msg_heartbeat_decode(message, &heartbeat);
    mavlink_msg_fence_status_decode(message, &fence_status);

    printf("Got Fence Status: ");
    printf("Fence Status: %d\n", fence_status.breach_type);
    switch(fence_status.breach_type) {
        case FENCE_BREACH_NONE:
            printf("No breach\n");
            break;
        case FENCE_BREACH_MINALT:
            printf("Minimum Altitude Breach\n");
            break;

        case FENCE_BREACH_MAXALT:
            printf("Maximum Altitude Breach\n");
            break;  

        case FENCE_BREACH_BOUNDARY:
            printf("Boundary Breach\n");
            break;
    }
}

void send_some_init(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    // Whenever a second has passed, we send a heartbeat.
    static time_t last_time = 0;
    time_t current_time = time(NULL);
    if (current_time - last_time >= 1) {
        initial_command(socket_fd, src_addr, src_addr_len);

        last_time = current_time;
    }
}

void send_some(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    // Whenever a second has passed, we send a heartbeat.
    static time_t last_time = 0;
    time_t current_time = time(NULL);
    if (current_time - last_time >= 1) {
        // send_heartbeat(socket_fd, src_addr, src_addr_len);
        test_send_command(socket_fd, src_addr, src_addr_len);

        last_time = current_time;
    }
}


// note that parameter 7 determines the takeoff altitude
void send_adsb_signal(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;

    mavlink_adsb_vehicle_t sp = {0};
    sp.ICAO_address = 4000; /*<  ICAO address*/
    sp.lat = (int) (-35.36277334 * 10000000.0);	 // specify the latitude and longitude here, speed normalized [0 - 1]
    sp.lon = (int) (149.16536671 * 10000000.0);	 /*< [degE7] Latitude*/
    sp.altitude = 604000; /* altitude is in militers above sea level < [mm] Altitude(ASL)*/
    sp.heading = 0; /*< [cdeg] Course over ground*/
    sp.hor_velocity = 10; /*< [cm/s] The horizontal velocity*/
    sp.ver_velocity = 0; /*< [cm/s] The vertical velocity. Positive is up*/
    sp.flags = 
              ADSB_FLAGS_VALID_COORDS |
                ADSB_FLAGS_VALID_ALTITUDE |
                ADSB_FLAGS_VALID_HEADING |
                ADSB_FLAGS_VALID_VELOCITY |
                ADSB_FLAGS_VALID_CALLSIGN |
                ADSB_FLAGS_VALID_SQUAWK |
                ADSB_FLAGS_SIMULATED |
                ADSB_FLAGS_VERTICAL_VELOCITY_VALID |
                ADSB_FLAGS_BARO_VALID;

    sp.squawk = 5000; /*<  Squawk code*/
    sp.altitude_type =  ADSB_ALTITUDE_TYPE_PRESSURE_QNH;

    strcpy(sp.callsign, "SIM5743"); //
    sp.emitter_type = ADSB_EMITTER_TYPE_UAV; /*<  ADSB emitter type.*/
    sp.tslc = 1; 

	// Encode:
	mavlink_msg_adsb_vehicle_encode(1, 255, &message, &sp);

    // write to the socket
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("SENDING ADSB SIGNAL\n");
    }
}


// executed every second
void test_send_command(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len) {
    send_adsb_signal(socket_fd, src_addr, src_addr_len);
}


void initial_command(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    send_adsb_signal(socket_fd, src_addr, src_addr_len);
}
