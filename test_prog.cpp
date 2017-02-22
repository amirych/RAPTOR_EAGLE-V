#include <iostream>

#include <eagle_camera.h>
#include <thread>
#include <chrono>

using namespace std;

int main(int argc, char* argv[])
{

    try {
            EagleCamera cam;
//        EagleCamera cam("/home/timur/raptor_eagle-v.fmt");
//        EagleCamera cam("/home/timur/zz1");

        if ( argc > 1 ) cam.setLogLevel(EagleCamera::LOG_LEVEL_ERROR);
        cam.initCamera(1, &std::cout);

        double texp = 1.33;
        texp = 7.777;
        std::cout << "\n\nSET EXP: " << texp << "\n";
        cam.setExposure(texp);

        std::cout << "READ EXP FROM CAMERA: " << cam.getExposure() << "\n";

        std::cout << "READ FRAME RATE FROM CAMERA: " << cam.getFrameRate() << "\n";


//        cam.setCCD_Temperature(20.0);
//        cam.enableTEC();
//        std::this_thread::sleep_for(std::chrono::seconds(2));

//        for ( int i = 0; i < 5; ++i ) {
//            std::cout << "\n\ntemp: " << cam.getCCD_Temperature() << "\n";
//            std::cout << "TEC PCB temp: " << cam.getPCB_Temperature() << "\n";
//            std::this_thread::sleep_for(std::chrono::seconds(2));
//        }
//        std::cout << "TEC set point: " << cam.getTEC_SetPoint() << "\n";
//        cam.enableTEC(false);
        std::cout << "\n\ntemp: " << cam.getCCD_Temperature() << "\n";
        std::cout << "TEC PCB temp: " << cam.getPCB_Temperature() << "\n";

        cam.setShutterState(EagleCamera::ShutterExp);
        cam.startExposure();

        cam.stopExposure();
        puts("GET SHUTTER:");
        std::cout << "shutter: " << cam.getShutterState() << "\n";

    } catch (EagleCamera_Exception &ex) {
        cout << "Eagle EXP: " << ex.what() << endl;
    } catch (XCLIB_Exception &ex) {
        cout << "XCLIB EXP: " << ex.what() << "   (err = " << ex.getError() << ")" << endl;
        cout << "XCLIB err msg: " << pxd_mesgErrorCode(ex.getError()) << "\n";
    }

    return 0;
}
