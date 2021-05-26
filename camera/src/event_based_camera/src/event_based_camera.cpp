#include <atomic>
#include <csignal>
#include </usr/include/libcaercpp/devices/edvs.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <chrono>
#include <ctime>
#include <cstdlib>
// #include "tensor.hpp"
#include "buf.hpp"
#include <time.h>
#include <ros/ros.h>
#include "event_based_camera/Control.h"
#include <std_msgs/Int8.h>

using namespace std;


void EventBasedCamera(const event_based_camera::Control& cmd_msg){

    int32_t start_stop = cmd_msg.sos;
    if (start_stop == 1){
        libcaer::devices::edvs edvsHandle
            = libcaer::devices::edvs(1, "/dev/ttyUSB0", CAER_HOST_CONFIG_SERIAL_BAUD_RATE_12M);

        struct caer_edvs_info edvs_info = edvsHandle.infoGet();

        printf("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d.\n", edvs_info.deviceString, edvs_info.deviceID,
        edvs_info.deviceIsMaster, edvs_info.dvsSizeX, edvs_info.dvsSizeY);

        edvsHandle.sendDefaultConfig();

        edvsHandle.dataStart(nullptr,nullptr,nullptr,nullptr,nullptr);
        edvsHandle.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
        while (1!=0){
            std::vector<uint16_t> Stream;
            
            // int Buffersize = 10;
            // Stream.resize(Buffersize);
            
            std::unique_ptr<libcaer::events::EventPacketContainer> packetContainer = edvsHandle.dataGet();
            for (auto &packet : *packetContainer){
                if (packet == nullptr){
                    continue;
                }
                if (packet->getEventType() == POLARITY_EVENT) {
                    circular_buffer<int> buff_x(packet->getEventNumber());
                    circular_buffer<int> buff_y(packet->getEventNumber());
                    circular_buffer<int> buff_ts(packet->getEventNumber());
                    circular_buffer<int> buff_polarity(packet->getEventNumber());


                    std::shared_ptr<const libcaer::events::PolarityEventPacket> polarity = std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet);
                    const libcaer::events::EventPacketContainer &TimeStamp = (*packetContainer);
                    int64_t ls = TimeStamp.getLowestEventTimestamp();
                    int64_t hs = TimeStamp.getHighestEventTimestamp();
                    // printf("ls: %d ",ls);
                    // printf("hs: %d ",hs);
                    // printf("hs-ls: %d ",hs-ls);
                    

                    for (int i=0;i<packet->getEventNumber();i++){
                        const libcaer::events::PolarityEvent &Event = (*polarity)[i];
                        int64_t ts = Event.getTimestamp64(*polarity);
                        uint16_t x = Event.getX();                    
                        uint16_t y = Event.getY();
                        bool pol   = Event.getPolarity();
                        // printf("Event - ts: %ld **** %ld, x: %d, y: %d, pol: %d.\n", ts/1000000, ts,x, y, 2*(pol)-1);
                        // printf("{ts: %ld \t x: %d }",ts,x);
                        buff_x.put(x);
                        buff_y.put(y);
                        buff_ts.put(ts);
                        buff_polarity.put( 2*(pol)-1);                
                    }
                    
                    while(!buff_ts.empty())
                        {
                            std::cout << buff_x.get() << "\t";
                            std::cout << buff_y.get() << "\t";
                            std::cout << buff_ts.get() << "\t";
                            std::cout << buff_polarity.get() << "\t";
                        }
                    printf("New: %d \n",packet->getEventNumber());
                    
                }
                
                // const libcaer::events::EventPacketContainer &Event = (*packetContainer);
                // int64_t lt = Event.getLowestEventTimestamp();
                // int64_t ht = Event.getHighestEventTimestamp();
                // int32_t en = Event.getEventsNumber();
                // printf("%ld \n",lt);
                // printf("%ld \n",ht);
                // printf("%d \n",en);
                // printf("****************************************************************************************************************************************************************");


            }
        }
    }else{
        ROS_INFO("No events from Camera");
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "new");
    ros::NodeHandle nh;
    ros::Subscriber event_camera = nh.subscribe("/cam", 10, EventBasedCamera);
	// int n;
	// cout<<"Enter value of n."<<endl;
	// cin>>n;
	// int milli_seconds=n*1000;
	// time_t start, end;
	// start=time(0);
	// while(1)
	// {
		
	// 	if(time(0)-start==n)
	// 	{
	// 	EventBasedCamera();
	// 	start=start+n;
	// 	}
	// }
	// printf("*************************");

	while (ros::ok())
    {
        usleep(8 * 1000);

        ros::spin();
    }


}
	

