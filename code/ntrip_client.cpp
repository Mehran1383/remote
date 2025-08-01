#include <iostream>
#include <fstream>
#include <string>
#include <mqtt/client.h>
#include <SerialStream.h>

#define ADDRESS     "ppntrip.services.u-blox.com"
#define CLIENTID    "YourDeviceID"
#define TOPIC       "/pp/your-topic/spartn" // Your specific topic from u-blox
#define QOS         1

using namespace std;
using namespace LibSerial;

SerialStream serial;

// Callback when correction data is received
class callback : public virtual mqtt::callback {
public:
    void message_arrived(mqtt::const_message_ptr msg) override {
        cout << "[MQTT] Correction received: " << msg->get_payload().size() << " bytes" << endl;
        // Forward to serial port
        serial.write(msg->get_payload().c_str(), msg->get_payload().size());
        serial.flush();
    }
};

int main() {
    try {
        // Open serial port to ZED-F9R
        serial.Open("/dev/ttyACM0");
        serial.SetBaudRate(BaudRate::BAUD_115200);
        serial.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        serial.SetParity(Parity::PARITY_NONE);
        serial.SetStopBits(StopBits::STOP_BITS_1);
        serial.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

        mqtt::client client(ADDRESS, CLIENTID);
        callback cb;
        client.set_callback(cb);

        mqtt::connect_options connOpts;
        connOpts.set_keep_alive_interval(20);
        connOpts.set_clean_session(true);
        connOpts.set_user_name("pp-username-from-u-blox");
        connOpts.set_password("pp-password-or-token");

        client.connect(connOpts);
        //client.subscribe(TOPIC, QOS);

        cout << "Connected to MQTT and subscribed to PointPerfect" << endl;

        // Wait for messages
        while (true) {
            this_thread::sleep_for(chrono::seconds(1));
        }

        client.disconnect();
        serial.Close();

    } catch (const mqtt::exception& e) {
        cerr << "MQTT Error: " << e.what() << endl;
        return 1;
    } catch (const exception& e) {
        cerr << "General Error: " << e.what() << endl;
        return 1;
    }

    return 0;
}
