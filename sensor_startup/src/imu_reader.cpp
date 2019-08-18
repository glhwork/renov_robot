#include "sensor_startup/imu_reader.h"

using mobile_base::ImuReader;

ImuReader::ImuReader() {
  n_private = ros::NodeHandle("imu");
  ParamInit();
  SerialInit();
  Setup();

  std::cout << port_id << std::endl;
  std::cout << baud_rate << std::endl;
  std::cout << imu_frame_id << std::endl;
  std::cout << imu_pub_topic << std::endl;
}

void ImuReader::SerialInit() {
  try {
    imu_ser.setPort(port_id);
    imu_ser.setBaudrate(baud_rate);
    imu_ser.setParity(serial::parity_none);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(1000);
    imu_ser.setTimeout(time_out);
    imu_ser.open();
  } catch (const serial::IOException& e) {
    ROS_ERROR("Unable to open port on %s -> [%s]", port_id, e.what());
    return;
  }
  if (imu_ser.isOpen()) {
    ROS_INFO("Open serial port successfully");

    if (use_request) {
      imu_ser.write(cmd.OUTPUT_FREQUENCY_00HZ,
                    sizeof(cmd.OUTPUT_FREQUENCY_00HZ));
    } else {
      switch (output_freq) {
        case 5: {
          imu_ser.write(cmd.OUTPUT_FREQUENCY_05HZ,
                        sizeof(cmd.OUTPUT_FREQUENCY_05HZ));
          break;
        }
        case 15: {
          imu_ser.write(cmd.OUTPUT_FREQUENCY_15HZ,
                        sizeof(cmd.OUTPUT_FREQUENCY_15HZ));
          break;
        }
        case 25: {
          imu_ser.write(cmd.OUTPUT_FREQUENCY_25HZ,
                        sizeof(cmd.OUTPUT_FREQUENCY_25HZ));
          break;
        }
        case 35: {
          imu_ser.write(cmd.OUTPUT_FREQUENCY_35HZ,
                        sizeof(cmd.OUTPUT_FREQUENCY_35HZ));
          break;
        }
        case 50: {
          imu_ser.write(cmd.OUTPUT_FREQUENCY_50HZ,
                        sizeof(cmd.OUTPUT_FREQUENCY_50HZ));
          break;
        }
        case 100: {
          imu_ser.write(cmd.OUTPUT_FREQUENCY_100HZ,
                        sizeof(cmd.OUTPUT_FREQUENCY_100HZ));
          break;
        }
        default: {
          ROS_WARN("incorrect frequency number");
          exit(1);
          break;
        }
      }
    }
    /*
    std::vector<uint8_t> reply;
    reply.clear();
    std::cout << imu_ser.available() << std::endl;
    imu_ser.read(reply, imu_ser.available());
    if (0 == reply[reply.size() - 2]) {
      ROS_INFO("Set imu reply frequency successfully");
    } else if (255 == reply[reply.size() - 2]) {
      ROS_INFO("Set imu reply frequency failed");
    }
    */

  } else {
    ROS_WARN("Port opening failed!");
    return;
  }
}

void ImuReader::ParamInit() {
  if (!n_private.getParam("port_id", port_id)) {
    port_id = "/dev/ttyUSB0";
  }
  if (!n_private.getParam("baud_rate", baud_rate)) {
    baud_rate = 115200;
  }
  if (!n_private.getParam("imu_frame_id", imu_frame_id)) {
    imu_frame_id = "base_imu";
  }
  if (!n_private.getParam("imu_pub_topic", imu_pub_topic)) {
    imu_pub_topic = "imu";
  }
  if (!n_private.getParam("use_request", use_request)) {
    use_request = true;
  }
  if (!n_private.getParam("output_freq", output_freq)) {
    output_freq = 50;
  }
}

void ImuReader::Setup() {
  imu_pub = n_private.advertise<sensor_msgs::Imu>(imu_pub_topic, 100);
}

void ImuReader::ReadData() {
  if (use_request) {
    imu_ser.write(cmd.ASK_FOR_DATA, sizeof(cmd.ASK_FOR_DATA));

    std::vector<uint8_t> imu_data;
    std::vector<uint8_t> empty_data;
    if (imu_ser.available()) {
      imu_data.clear();
      imu_ser.read(imu_data, 32);
      imu_ser.read(empty_data, imu_ser.available());
      DataParser(imu_data);
    }
  } else {
    std::vector<uint8_t> imu_data;
    imu_ser.read(imu_data, 32);
    // imu_ser.flushInput();
    DataParser(imu_data);
  }
}

void ImuReader::DataParser(const std::vector<uint8_t>& data) {
  // vec = {roll/pitch/yaw, acc_x/_y/_z, angular_vx/_vy/_vz}
  Eigen::VectorXd vec(9);
  if (data.size() == 32) {
    vec(0) = (double)Converter(data[4], data[5], data[6]) / 100.0;
    vec(1) = (double)Converter(data[7], data[8], data[9]) / 100.0;
    vec(2) = (double)Converter(data[10], data[11], data[12]) / 100.0;

    // attention : computing acceleration should divide 1000
    vec(3) = (double)Converter(data[13], data[14], data[15]) / 1000.0;
    vec(4) = (double)Converter(data[16], data[17], data[18]) / 1000.0;
    vec(5) = (double)Converter(data[19], data[20], data[21]) / 1000.0;

    vec(6) = (double)Converter(data[22], data[23], data[24]) / 100.0;
    vec(7) = (double)Converter(data[25], data[26], data[27]) / 100.0;
    vec(8) = (double)Converter(data[28], data[29], data[30]) / 100.0;

    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = imu_frame_id;
    imu_msg.header.stamp = ros::Time::now();

    // get orientation in quaternion form
    Eigen::Quaterniond q = Eigen::AngleAxisd(vec(0), Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(vec(1), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(vec(2), Eigen::Vector3d::UnitZ());
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();
    imu_msg.orientation_covariance = {1e-5, 0, 0, 0, 1e-5, 0, 0, 0, 1e-5};

    // get angular velocity
    imu_msg.angular_velocity.x = vec(6);
    imu_msg.angular_velocity.y = vec(7);
    imu_msg.angular_velocity.z = vec(8);
    imu_msg.angular_velocity_covariance = {1e-5, 0, 0, 0, 1e-5, 0, 0, 0, 1e-5};

    // get linear acceleration (value * g)
    imu_msg.linear_acceleration.x = vec(3);
    imu_msg.linear_acceleration.y = vec(4);
    imu_msg.linear_acceleration.z = vec(5);
    imu_msg.linear_acceleration_covariance = {1e-5, 0, 0, 0, 1e-5, 0, 0, 0, 1e-5};

    // printf("angular -> roll: %.2f, pitch: %.2f, yaw: %.2f \n", vec(0),
    // vec(1), vec(2)); printf("quaternion -> x: %.2f, y: %.2f, z: %.2f, w: %.2f
    // \n", q.x(), q.y(), q.z(), q.w()); ROS_INFO("Data : \n angular velocity
    // [x: %.2f, y: %.2f, z: %.2f]\n linear acc [x: %.2f, y: %.2f, z: %.2f]\n
    // orientation [roll: %.2f, pitch: %.2f, yaw: %.2f]",
    //          vec(6), vec(7), vec(8), vec(3), vec(4), vec(5), vec(0), vec(1),
    //          vec(2));

    imu_pub.publish(imu_msg);
  } else {
    ROS_WARN("imu data size = %d is not correct", (int)data.size());
  }
}

int ImuReader::Converter(const uint8_t a, const uint8_t b, const uint8_t c) {
  std::stringstream int2str, str2int;
  int2str << (int)a % 16 << (int)b / 16 << (int)b % 16 << (int)c / 16
          << (int)c % 16;
  int int_result;
  std::string str_result;
  int2str >> str_result;
  str2int << str_result;
  str2int >> int_result;

  if (1 == (int)a / 16) {
    int_result = -int_result;
  } else {
    int_result = abs(int_result);
  }

  return int_result;
}