#include <eagle_camera.h>


                    /**********************************************************
                    *                                                         *
                    *      EagleCamera_Exception CLASS IMPLEMENTATION         *
                    *                                                         *
                    **********************************************************/

EagleCamera_Exception::EagleCamera_Exception(const EagleCamera::Error err, const std::string &context, const char contr_ans):
    std::exception(), error(err), _context(context), controllerAnswer(contr_ans)
{
}


EagleCamera_Exception::EagleCamera_Exception(const EagleCamera::Error err, const char *context, const char contr_ans):
    EagleCamera_Exception(err, std::string(context), contr_ans)
{
}


EagleCamera::Error EagleCamera_Exception::getError() const
{
    return error;
}


char EagleCamera_Exception::getControllerAnswer() const
{
    return controllerAnswer;
}


const char* EagleCamera_Exception::what() const NOEXCEPT_DECL
{
    return _context.c_str();
}



                    /****************************************************
                    *                                                   *
                    *      XCLIB_Exception CLASS IMPLEMENTATION         *
                    *                                                   *
                    ****************************************************/


XCLIB_Exception::XCLIB_Exception(int err_code, const std::string &context):
    std::exception(), errCode(err_code), _context(context)
{

}

XCLIB_Exception::XCLIB_Exception(int err_code, const char *context):
    XCLIB_Exception(err_code, std::string(context))
{

}


int XCLIB_Exception::getError() const
{
    return errCode;
}


const char* XCLIB_Exception::what() const NOEXCEPT_DECL
{
    return _context.c_str();
}
