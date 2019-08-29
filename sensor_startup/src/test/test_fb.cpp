#include "sensor_msgs/JointState.h"
#include "sensor_startup/motor_reader.h"

class TestFb : public MotorReader {
  public:
    TestFb() : MotorReader() {}
    virtual ~TestFb() {}
    void FeedbackLoop();
};

void TestFb::FeedbackLoop() {


  int frame_num = 2500;
  sensor_msgs::JointState state;
  state.velocity.resize(8);
  state.position.resize(8);

  VelPosiFeedbackReq(true, false);
  PVCI_CAN_OBJ obj = new VCI_CAN_OBJ[frame_num];
  VCI_Receive(4, 0, 0, obj, frame_num, 1);
  GetFeedback(state, 

  

}
