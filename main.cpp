#include<iostream>
#include<arpa/inet.h>
#include<unistd.h>
#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include <errno.h>
#include <fcntl.h>

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <time.h>

#include <mavlink.h>
//#include <mavlink_msg_mission_count.h>

#define BUFFER_LENGTH 2041

using namespace std;

void error( char *msg)
{
 perror(msg);
 exit(EXIT_FAILURE);
}
int main()
{
 int sock;
 sock = socket(AF_INET,SOCK_DGRAM,0);
 if(sock < 0)
 {
     printf("Socket invalido\n");
 }
 struct sockaddr_in serv;
 
    if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    {
            fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
            close(sock);
            exit(EXIT_FAILURE);
    }
  
 
 socklen_t fromlen;
 
 serv.sin_family = AF_INET;
 serv.sin_port = htons(5556);
 serv.sin_addr.s_addr = inet_addr("127.0.0.1");

 uint8_t buf[BUFFER_LENGTH];
 socklen_t m = sizeof(serv);
 	
	uint16_t len;
	int i = 0;
        int bytes_sent=0;
        int recsize=0;
        unsigned int temp = 0;
        mavlink_message_t msg;
        mavlink_status_t status;
//        cout << "Heart" << endl;
 if(bind(sock,(struct sockaddr *)&serv, sizeof(struct sockaddr)))
 {
    perror("error bind failed");
    close(sock);
    exit(EXIT_FAILURE);
 }

 while(true)
 {
        //mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
        //mavlink_msg_heartbeat_pack(1, 0, &msg, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
        
       // mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_IMU, &msg, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
        
        //len = mavlink_msg_heartbeat_encode(1,0,&msg,&mh);
        //len = mavlink_msg_to_send_buffer(buf, &msg);
        //bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&serv, sizeof(struct sockaddr_in));

        /*printf("Senviando count\n");
        mavlink_msg_mission_count_pack(1,MAV_COMP_ID_MISSIONPLANNER, &msg,1,MAV_COMP_ID_MISSIONPLANNER,0);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&serv, sizeof(struct sockaddr_in));
        printf("Count enviado\n");
      */
        mavlink_msg_mission_request_list_pack(1, MAV_COMP_ID_MISSIONPLANNER, &msg,1,MAV_COMP_ID_MISSIONPLANNER);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&serv, sizeof(struct sockaddr_in));
        
        
        mavlink_msg_mission_count_pack(1,MAV_COMP_ID_MISSIONPLANNER, &msg,1,190,255);
                    
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&serv, sizeof(struct sockaddr_in));
                    printf("Count enviado\n"); 
        
        
     /*   
      * 
        printf("Enviando get\n");
        mavlink_msg_mission_current_pack(1,MAV_COMP_ID_MISSIONPLANNER,&msg,0);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&serv, sizeof(struct sockaddr_in));
        printf("Enviado get\n"); */
        
        /*mavlink_mission_item_t itm;
        itm.autocontinue=0;
        itm.command=MAV_CMD_DO_SET_ROI;
        itm.current=0;
        itm.param1=MAV_ROI_LOCATION;
        itm.param2=MAV_ROI_LOCATION;
        itm.param3=1;
        itm.x=-35.364043;
        itm.y=149.162853;
        itm.z=100;
        
        len = mavlink_msg_to_send_buffer(buf,&msg);
        
        
        
        //mavlink_msg_mission_item_encode(1,MAV_COMP_ID_IMU,&msg,&itm);
        bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&serv, sizeof(struct sockaddr_in));
        */  
     recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&serv, &fromlen);
     if (recsize > 0)
     {
        for (i = 0; i < recsize; ++i)
        {
            temp = buf[i];
            //printf("%02x ", (unsigned char)temp);
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
            {
                // Packet received
                //printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);

                //printf("Received packet: MSG ID: %d\n", msg.msgid);
                if(msg.msgid == MAVLINK_MSG_ID_MISSION_ACK)
                {
                    printf("MISSION ACK\n");
                }
                else if (msg.msgid == MAVLINK_MSG_ID_MISSION_COUNT)
                {
                    printf("MISSION COUNT\n");
                    mavlink_mission_count_t aaa;
                    
                    mavlink_msg_mission_count_decode(&msg,&aaa);
                    printf("Total: %d - Component: %d - System: %d\n",aaa.count,aaa.target_component,aaa.target_system);
                   
                }
                else if (msg.msgid == MAVLINK_MSG_ID_PING)
                {
                    printf("MISSION PING\n");
                }
                else if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
                {
                    printf("Heartbeat recebidox\n");
                }                                        
                else if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_LIST)
                {
                    /*printf("Mission_Req_list\n");
                    mavlink_mission_request_list_t packet;
                    mavlink_msg_mission_request_list_decode(&msg, &packet);
                    
                    printf("Senviando count\n");                    
                    mavlink_msg_mission_count_pack(1,MAV_COMP_ID_MISSIONPLANNER, &msg,packet.target_system,packet.target_component,1);
                    
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&serv, sizeof(struct sockaddr_in));
                    printf("Count enviado\n"); */
                }
                else if(msg.msgid == MAVLINK_MSG_ID_MISSION_CURRENT)
                {
                    printf("Mission Current\n");
                    mavlink_mission_current_t mc;
                    mavlink_msg_mission_current_decode(&msg,&mc);
                    printf("Item da missao: %d\n", mc.seq);
                }                                            
                else if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM)
                {
                    printf("Mission item\n");
                }
                else if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM_REACHED)
                {
                    printf("Item Reached\n");
                }
                else if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT)
                {
                    printf("Mission Item\n");
                }
                else if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST)
                {
                    printf("Mission Request\n");
                }
                else if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT)
                {
                    printf("Mission Request Int\n");
                }
                else if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST)
                {
                    printf("Mission Partial List\n");
                }
                else if (msg.msgid == MAVLINK_MSG_ID_MISSION_SET_CURRENT)
                {
                    printf("Mission Set Current\n");
                }
                else if (msg.msgid == MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST)
                {
                    printf("Mission Write Partial\n");
                }
                else if (msg.msgid == MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE)
                {
                    printf("Mission Data Transmission Handshake\n");
                }
                else if (msg.msgid == MAVLINK_MSG_ID_MISSION_CLEAR_ALL)
                {
                    printf("Mission Clear\n");
                }
                else if(msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM_REACHED)
                {
                    printf("ITEM REACHED\n");
                }
                /*MAVLINK_MSG_ID_REQUEST_DATA_STREAM
                        SET_MODE
                        MAVLINK_MSG_ID_MISSION_REQUEST
                        MAVLINK_MSG_ID_PARAM_REQUEST_LIST
                  */      


                
                
            }
         }
     }
     else
     {
         cout << "Sem dados" << endl;
     }
    memset(buf, 0, BUFFER_LENGTH);
    sleep(1); // Sleep one second
     //cout << "Aguardado" << endl;
     //cout << "Aguardando" << endl;
 }
}