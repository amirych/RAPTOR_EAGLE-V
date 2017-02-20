#include <iostream>

#include <eagle_camera.h>

using namespace std;

int main()
{
    EagleCamera cam;
//    cam.setLogLevel(EagleCamera::LOG_LEVEL_ERROR);

    try {
        cam.initCamera(1, &std::cout);

    } catch (EagleCamera_Exception &ex) {
        cout << "Eagle EXP: " << ex.what() << endl;
    } catch (XCLIB_Exception &ex) {
        cam.logToFile(ex);
//        cout << "XCLIB EXP: " << ex.what() << "   (err = " << ex.getError() << ")" << endl;
    }

    return 0;
}
