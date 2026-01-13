#include <boost/asio.hpp>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;
using namespace boost;

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0), io_(), serial_(io_)
    {
        try {
            // Configure the serial port (replace with your port name)
            configureSerialPort(serial_, "/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_9553034373435120C231-if00", 115200);
        }
        catch (const std::exception& e) {
            cerr << "Error configuring serial port: " << e.what() << endl;
            return;
        }

        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    // Function to configure the serial port
    void configureSerialPort(asio::serial_port& serial,
                             const string& portname,
                             unsigned int baud_rate)
    {
        // Open the specified serial port
        serial.open(portname);
        // Set the baud rate
        serial.set_option(asio::serial_port_base::baud_rate(baud_rate));
    }

    // Function to read data from the serial port until '\n'
    string readFromSerialPort(asio::serial_port& serial)
    {
        asio::streambuf buffer; // Buffer to store incoming data
        system::error_code ec;

        // Read data from the serial port until '\n'
        asio::read_until(serial, buffer, '\n', ec);

        if (ec && ec != asio::error::eof) {
            cerr << "Error reading from serial port: " << ec.message() << endl;
            return "";
        }

        // Convert the buffer into a string
        istream is(&buffer);
        string line;
        getline(is, line); // Extract the line from the buffer

        return line;
    }

    // Function to write data to the serial port
    void writeToSerialPort(asio::serial_port& serial,
                           const string& message)
    {
        system::error_code ec;
        // Write data to the serial port
        asio::write(serial, asio::buffer(message), ec);
        if (ec) {
            cerr << "Error writing to serial port: " << ec.message() << endl;
        }
    }

    void timer_callback()
    {
        string response = readFromSerialPort(serial_);
        // if (!response.empty()) {
        //     cout << "Response received: " << response << endl;
        // }

        auto message = std_msgs::msg::String();
        message.data = response;
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    asio::io_service io_; // IO service object
    asio::serial_port serial_; // Serial port object
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}