#include <eagle_camera.h>


                    /**********************************************************
                    *                                                         *
                    *      EagleCamera_exception CLASS IMPLEMENTATION         *
                    *                                                         *
                    **********************************************************/


EagleCamera_Exception::EagleCamera_Exception(int err_code, const std::string &context):
    std::exception(), errCode(err_code), _context(context)
{

}

EagleCamera_Exception::EagleCamera_Exception(int err_code, const char *context):
    EagleCamera_Exception(err_code, std::string(context))
{

}


int EagleCamera_Exception::getError() const
{
    return errCode;
}


const char* EagleCamera_Exception::what() const NOEXCEPT_DECL
{
    return _context.c_str();
}
