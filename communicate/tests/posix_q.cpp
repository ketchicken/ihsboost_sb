#include "posixqcommunicator.hpp"
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>

using namespace std;
using namespace chrono;

int main(int argc, const char *argv[])
{
    if (argc != 2)
    {
        cerr << "need a second argument to be provided" << endl;
        cerr << "either `talker` or `listener`" << endl;
        return -1;
    }

    const char *name = "/my_queue";
    if (string(argv[1]) == "talker")
    {
        cout << "talker" << endl;
        PosixQCommunicator talker(name);

        int count = 0;
        ostringstream s;
        while (1)
        {
            s << "message " << count;

            cout << "sending message : " << s.str() << endl;

            talker.send_msg(s.str());
            this_thread::sleep_for(milliseconds(500));

            s.str("");
            s.clear();
            ++count;
        }
    }

    if (string(argv[1]) == "listener")
    {
        cout << "listener" << endl;
        PosixQCommunicator listener(name);

        while (1)
        {
            cout << "received: " << listener.receive_msg() << endl;
        }
    }

    return 0;
}