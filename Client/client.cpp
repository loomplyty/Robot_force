#include <Robot_Client.h>

using namespace std;

int main(int argc, char *argv[])
{
    Robots::SendRequest(argc, argv, "/usr/Robots/CMakeDemo/Robot_III/resource/HexapodIII_Move.xml");

	return 0;
}
