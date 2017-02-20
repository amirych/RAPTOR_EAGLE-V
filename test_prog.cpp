#include <iostream>

#include <eagle_camera.h>

using namespace std;

int main()
{
    try {
        EagleCamera cam;

        cam.initCamera(1, &cout);
    } catch (EagleCamera_Exception &ex) {

    } catch (XCLIB_Exception &ex) {

    }

    return 0;
}
