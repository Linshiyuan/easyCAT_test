#include "imu.h"
#include "serial/serial.h"
serial::Serial imuPort;
int main(int argc, char *argv[])
{
    atomImu imu;
    imu.geekname="ok";
    imu.printname();





    imuPort.setPort("/dev/ttyUSB0");
    imuPort.setBaudrate(460800);
    // ros_ser.setBaudrate(2000000);
    // ros_ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 1000
    bool level=true;
    // ros_ser.setRTS(level);
    // ros_ser.setDTR(level);
    imuPort.setTimeout(to);
    // ros_ser.setParity(serial::parity_even);
    
    std::string bufferS;
    uint8_t *buffer8;
    imuPort.open();
     while (1) {
          int n ;
    n= imuPort.available();
    if(n>0){
        std::vector<uint8_t> buffer;
    auto readout = imuPort.read(buffer, n);
      for (int i=0; i<n; i++) {
      // std::cout<< std::hex <<buffer[i];
  
      }
     std::cout<<  "read n: "<<n<<std::endl;// std::hex <<
    }

     }
    return 0;
}